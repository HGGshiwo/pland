#include "pland_controller.hpp"
#include <ros_param_sync/param_sync.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <cmath>
#include <fstream>
#include <memory>
#include <yaml-cpp/yaml.h>

class PlandControllerNode {
public:
  PlandControllerNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
      : nh_(nh), pnh_(pnh), control_enabled_(false) {
    // 1. 读取节点连接参数与静态话题 (在 launch 中指定)
    std::string odom_topic;
    std::string rangefinder_topic;
    std::string rel_alt_topic;
    std::string target_pose_topic;
    std::string target_vel_topic;
    std::string inject_target_pose_topic;
    std::string inject_target_vel_topic;
    std::string cmd_vel_topic;
    std::string setpoint_raw_topic;
    std::string command_service;
    std::string set_mode_service;
    std::string start_topic;
    std::string cancel_topic;
    std::string state_topic;
    std::string param_config_path;
    double control_rate = 50.0;

    pnh_.param<std::string>("odom_topic", odom_topic,
                           "/mavros/local_position/odom");
    pnh_.param<std::string>("rangefinder_topic", rangefinder_topic,
                           "/mavros/distance_sensor/rangefinder_pub");
    pnh_.param<std::string>("rel_alt_topic", rel_alt_topic,
                           "/mavros/global_position/rel_alt");
    pnh_.param<std::string>("target_pose_topic", target_pose_topic,
                           "/pland/target_pose");
    pnh_.param<std::string>("target_vel_topic", target_vel_topic,
                           "/pland/target_vel");
    pnh_.param<std::string>("inject_target_pose_topic", inject_target_pose_topic,
                           "/pland/inject_target_pose");
    pnh_.param<std::string>("inject_target_vel_topic", inject_target_vel_topic,
                           "/pland/inject_target_vel");
    pnh_.param<std::string>("cmd_vel_topic", cmd_vel_topic,
                           "/pland/cmd_vel");
    pnh_.param<std::string>("setpoint_raw_topic", setpoint_raw_topic,
                           "/mavros/setpoint_raw/local");
    pnh_.param<std::string>("command_service", command_service,
                           "/mavros/cmd/command");
    pnh_.param<std::string>("set_mode_service", set_mode_service,
                           "/mavros/set_mode");
    pnh_.param<std::string>("start_topic", start_topic,
                           "/pland/start");
    pnh_.param<std::string>("cancel_topic", cancel_topic,
                           "/pland/cancel");
    pnh_.param<std::string>("state_topic", state_topic,
                           "/pland/state");
    pnh_.param<std::string>("param_config_path", param_config_path, "");
    std::string drone_config_path;
    pnh_.param<std::string>("drone_config_path", drone_config_path, "");
    pnh_.param<double>("control_rate", control_rate, 50.0);

    // 2. 初始化核心控制器
    controller_ = std::make_unique<PlandController>(nh_, pnh_);

    // 2.1 静态加载飞机硬件物理配置 (不参与 ParamSync 动态同步)
    if (!drone_config_path.empty()) {
      std::ifstream fin(drone_config_path);
      if (fin.good()) {
        try {
          YAML::Node doc = YAML::LoadFile(drone_config_path);
          if (doc["frame_height"]) {
            double fh = doc["frame_height"].as<double>();
            controller_->set_frame_height(fh);
            ROS_INFO("[PlandControllerNode] Loaded static drone hardware config from '%s': frame_height=%.2fm",
                     drone_config_path.c_str(), fh);
          }
        } catch (const std::exception &e) {
          ROS_WARN("[PlandControllerNode] Failed to parse drone_config YAML at '%s': %s",
                   drone_config_path.c_str(), e.what());
        }
      } else {
        ROS_INFO("[PlandControllerNode] Static drone config '%s' not found. Using default frame_height=0.0m (Copy from drone_config.template.yaml to customize).",
                 drone_config_path.c_str());
      }
    }

    // 3. 初始化动态参数同步器 (与 pland_detector 一致)
    if (!param_config_path.empty()) {
      sync_.init(pnh_, param_config_path);

      // 绑定控制器内部的动态控制参数
      controller_->bind_dynamic_params(sync_);

      // 启动 1s 轮询监控与自动写回
      sync_.start(1.0);
    }

    // 4. 状态发布器
    state_pub_ = nh_.advertise<std_msgs::String>(state_topic, 10);

    // 5. 订阅 MAVROS 里程计、测距仪与相对高度
    sub_odom_ =
        nh_.subscribe(odom_topic, 10, &PlandControllerNode::odomCallback, this);
    sub_rangefinder_ = nh_.subscribe(
        rangefinder_topic, 10, &PlandControllerNode::rangefinderCallback, this);
    sub_rel_alt_ = nh_.subscribe(
        rel_alt_topic, 10, &PlandControllerNode::relAltCallback, this);

    // 6. 订阅视觉检测目标 (PoseStamped & TwistStamped)
    sub_target_pose_ = nh_.subscribe(
        target_pose_topic, 10, &PlandControllerNode::targetPoseCallback, this);
    sub_target_vel_ = nh_.subscribe(
        target_vel_topic, 10, &PlandControllerNode::targetVelCallback, this);

    // 7. 订阅 GPS 注入目标 (NavSatFix 经纬高 & TwistStamped 速度)
    sub_inject_target_pose_ = nh_.subscribe(
        inject_target_pose_topic, 10,
        &PlandControllerNode::injectTargetGpsCallback, this);
    sub_inject_target_vel_ = nh_.subscribe(
        inject_target_vel_topic, 10,
        &PlandControllerNode::injectTargetVelCallback, this);

    // 8. 订阅控制启停话题 (Start & Cancel)
    sub_start_ =
        nh_.subscribe(start_topic, 10, &PlandControllerNode::startCallback, this);
    sub_cancel_ =
        nh_.subscribe(cancel_topic, 10, &PlandControllerNode::cancelCallback, this);

    // 9. 启动控制循环定时器
    double period = 1.0 / std::max(1.0, control_rate);
    timer_ = nh_.createTimer(ros::Duration(period),
                             &PlandControllerNode::controlLoop, this);

    ROS_INFO("[PlandControllerNode] Initialized at rate: %.1f Hz.", control_rate);
    ROS_INFO("[PlandControllerNode] Subscribing to odom: %s", odom_topic.c_str());
    ROS_INFO("[PlandControllerNode] Subscribing to target_pose: %s",
             target_pose_topic.c_str());
    ROS_INFO("[PlandControllerNode] Subscribing to target_vel: %s",
             target_vel_topic.c_str());
    ROS_INFO("[PlandControllerNode] Subscribing to inject_pose (GPS NavSatFix): %s",
             inject_target_pose_topic.c_str());
    ROS_INFO("[PlandControllerNode] Subscribing to start: %s", start_topic.c_str());
    ROS_INFO("[PlandControllerNode] Subscribing to cancel: %s", cancel_topic.c_str());
    ROS_INFO("[PlandControllerNode] Publishing state to: %s", state_topic.c_str());
    ROS_INFO("[PlandControllerNode] Initial control status: %s",
             control_enabled_ ? "ENABLED" : "DISABLED (waiting for start command)");
  }

private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    double t = msg->header.stamp.toSec();
    if (t <= 0.0) {
      t = ros::Time::now().toSec();
    }

    pos_enu_ << msg->pose.pose.position.x, msg->pose.pose.position.y,
        msg->pose.pose.position.z;
    orientation_ =
        Eigen::Quaterniond(msg->pose.pose.orientation.w,
                           msg->pose.pose.orientation.x,
                           msg->pose.pose.orientation.y,
                           msg->pose.pose.orientation.z);
    vel_enu_ << msg->twist.twist.linear.x, msg->twist.twist.linear.y,
        msg->twist.twist.linear.z;

    controller_->update_drone_state(t, pos_enu_, orientation_, vel_enu_);
  }

  void rangefinderCallback(const sensor_msgs::Range::ConstPtr &msg) {
    double t = msg->header.stamp.toSec();
    if (t <= 0.0) {
      t = ros::Time::now().toSec();
    }
    bool is_valid = (!std::isnan(msg->range) && !std::isinf(msg->range) &&
                     msg->range >= msg->min_range && msg->range <= msg->max_range);
    controller_->update_rangefinder(t, msg->range, is_valid);
  }

  void relAltCallback(const std_msgs::Float64::ConstPtr &msg) {
    double t = ros::Time::now().toSec();
    if (!std::isnan(msg->data) && !std::isinf(msg->data)) {
      controller_->update_rel_alt(t, msg->data);
    }
  }

  void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    double t = msg->header.stamp.toSec();
    if (t <= 0.0) {
      t = ros::Time::now().toSec();
    }

    Eigen::Vector3d target_pos_body(msg->pose.position.x,
                                    msg->pose.position.y,
                                    msg->pose.position.z);
    Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x,
                         msg->pose.orientation.y, msg->pose.orientation.z);
    double target_yaw_body = std::atan2(
        2.0 * (q.w() * q.z() + q.x() * q.y()),
        1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

    controller_->update_detector_target(t, target_pos_body,
                                       last_detector_vel_enu_, target_yaw_body);
  }

  void targetVelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    last_detector_vel_enu_ << msg->twist.linear.x, msg->twist.linear.y,
        msg->twist.angular.z; // z 分量记录偏航角速度
  }

  void injectTargetGpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    if (msg->status.status < sensor_msgs::NavSatStatus::STATUS_FIX) {
      return;
    }
    double t = msg->header.stamp.toSec();
    if (t <= 0.0) {
      t = ros::Time::now().toSec();
    }
    last_inject_stamp_ = t;
    last_inject_lat_lon_alt_ << msg->latitude, msg->longitude, msg->altitude;
    has_inject_target_ = true;

    controller_->update_inject_target(last_inject_stamp_, last_inject_lat_lon_alt_,
                                      last_inject_vel_);
  }

  void injectTargetVelCallback(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    last_inject_vel_ << msg->twist.linear.x, msg->twist.linear.y,
        msg->twist.linear.z;
    if (has_inject_target_) {
      controller_->update_inject_target(last_inject_stamp_, last_inject_lat_lon_alt_,
                                        last_inject_vel_);
    }
  }

  void startCallback(const std_msgs::Empty::ConstPtr &) {
    ROS_INFO("[PlandControllerNode] Start command received on topic. Precision landing ENABLED.");
    controller_->reset();
    controller_->change_state(ControllerState::IDLE);
    control_enabled_ = true;
  }

  void cancelCallback(const std_msgs::Empty::ConstPtr &) {
    ROS_WARN("[PlandControllerNode] Cancel command received on topic. Precision landing DISABLED.");
    control_enabled_ = false;
    controller_->reset();
    controller_->change_state(ControllerState::IDLE);
  }

  void controlLoop(const ros::TimerEvent &) {
    if (!control_enabled_) {
      return;
    }

    controller_->step();

    // 状态外发供地面站和上层状态机监控
    if (state_pub_.getNumSubscribers() > 0) {
      std_msgs::String msg;
      switch (controller_->get_state()) {
      case ControllerState::IDLE:
        msg.data = "IDLE";
        break;
      case ControllerState::TRACING_GPS:
        msg.data = "TRACING_GPS";
        break;
      case ControllerState::TRACING_DETECTOR:
        msg.data = "TRACING_DETECTOR";
        break;
      case ControllerState::TARGET_LOST:
        msg.data = "TARGET_LOST";
        break;
      case ControllerState::BLIND_DROP:
        msg.data = "BLIND_DROP";
        break;
      case ControllerState::LANDED:
        msg.data = "LANDED";
        break;
      }
      state_pub_.publish(msg);
    }
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::unique_ptr<PlandController> controller_;
  ros_param_sync::ParamSync sync_;

  ros::Subscriber sub_odom_;
  ros::Subscriber sub_rangefinder_;
  ros::Subscriber sub_rel_alt_;
  ros::Subscriber sub_target_pose_;
  ros::Subscriber sub_target_vel_;
  ros::Subscriber sub_inject_target_pose_;
  ros::Subscriber sub_inject_target_vel_;
  ros::Subscriber sub_start_;
  ros::Subscriber sub_cancel_;

  ros::Publisher state_pub_;
  ros::Timer timer_;

  bool control_enabled_ = false;

  // 状态缓存
  Eigen::Vector3d pos_enu_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d vel_enu_ = Eigen::Vector3d::Zero();

  Eigen::Vector3d last_detector_vel_enu_ = Eigen::Vector3d::Zero();

  bool has_inject_target_ = false;
  double last_inject_stamp_ = 0.0;
  Eigen::Vector3d last_inject_lat_lon_alt_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_inject_vel_ = Eigen::Vector3d::Zero();
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "pland_controller_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  PlandControllerNode node(nh, pnh);

  ros::spin();
  return 0;
}
