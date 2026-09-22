#include "pland_detector.hpp"
#include <ros_param_sync/param_sync.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

#include <algorithm>
#include <cstdio>
#include <memory>
#include <optional>

class PlandDetectorNode {
public:
  PlandDetectorNode(ros::NodeHandle &nh, ros::NodeHandle &pnh)
      : nh_(nh), pnh_(pnh), detection_enabled_(true) {
    // 1. 读取节点连接参数
    std::string odom_topic;
    std::string image_topic;
    std::string detect_topic;
    std::string reset_topic;
    std::string state_topic;
    std::string gimbal_roll_topic;
    std::string gimbal_pitch_topic;
    std::string gimbal_yaw_topic;
    std::string param_config_path;
    std::string drone_config_path;

    // 默认从 MAVROS 读取里程计与位姿数据
    pnh_.param<std::string>("odom_topic", odom_topic, "/mavros/local_position/odom");
    pnh_.param<std::string>("image_topic", image_topic, "/roscam/cam/image_raw");
    pnh_.param<std::string>("detect_topic", detect_topic, "/pland/detect");
    pnh_.param<std::string>("reset_topic", reset_topic, "/pland/reset");
    pnh_.param<std::string>("state_topic", state_topic, "/pland/state");
    pnh_.param<std::string>("param_config_path", param_config_path, "");
    pnh_.param<std::string>("drone_config_path", drone_config_path, "");

    pnh_.param<std::string>("gimbal_roll_topic", gimbal_roll_topic, "/gimbal/roll");
    pnh_.param<std::string>("gimbal_pitch_topic", gimbal_pitch_topic, "/gimbal/current_pitch");
    pnh_.param<std::string>("gimbal_yaw_topic", gimbal_yaw_topic, "/gimbal/yaw");

    // 2. 初始化核心检测器
    detector_ = std::make_unique<PlandDetector>(nh_, pnh_);

    // 3. 加载硬件/机架静态配置文件 (drone_config.yaml: 相机外参偏移等)
    if (!drone_config_path.empty()) {
      try {
        YAML::Node config = YAML::LoadFile(drone_config_path);
        double ox = config["offset_x"] ? config["offset_x"].as<double>() : 0.0;
        double oy = config["offset_y"] ? config["offset_y"].as<double>() : 0.0;
        double oz = config["offset_z"] ? config["offset_z"].as<double>() : 0.0;
        detector_->set_camera_offset(ox, oy, oz);
        ROS_INFO("[PlandDetectorNode] Loaded camera offset from drone_config: [%.3f, %.3f, %.3f] m",
                 ox, oy, oz);
      } catch (const std::exception &e) {
        ROS_WARN("[PlandDetectorNode] Failed to load drone_config at %s: %s",
                 drone_config_path.c_str(), e.what());
      }
    }

    // 4. 初始化动态参数同步器 (仅动态参数进行绑定与持久化)
    if (!param_config_path.empty()) {
      sync_.init(pnh_, param_config_path);

      // 节点层动态参数 (从 YAML / rosparam 读取，未提供则报错跳过)
      sync_.bind("enable_detection", detection_enabled_, [&](bool enabled) {
        ROS_INFO("[PlandDetectorNode] enable_detection updated: %s",
                 enabled ? "ENABLED" : "DISABLED");
        if (!enabled) {
          detector_->reset();
        }
      });
      sync_.bind("publish_debug_image", publish_debug_image_);
      sync_.bind("gimbal_in_degrees", gimbal_in_degrees_);

      // 检测器内部动态参数 (velocity_deadzone, gimbal_abs)
      detector_->bind_dynamic_params(sync_);

      // 启动 1s 轮询监控与自动写回
      sync_.start(1.0);
    } else {
      pnh_.param<bool>("enable_detection", detection_enabled_, true);
      pnh_.param<bool>("publish_debug_image", publish_debug_image_, true);
      pnh_.param<bool>("gimbal_in_degrees", gimbal_in_degrees_, true);
    }

    // 4. 发布调试标注图像
    if (publish_debug_image_) {
      debug_image_pub_ = nh_.advertise<sensor_msgs::Image>(detect_topic, 10);
    }

    // 5. 订阅 MAVROS 里程计 (位置、四元数、机体系角速度)
    sub_odom_ = nh_.subscribe(odom_topic, 10, &PlandDetectorNode::odomCallback, this);

    // 6. 订阅云台数据
    sub_gimbal_roll_ = nh_.subscribe(gimbal_roll_topic, 10,
                                     &PlandDetectorNode::gimbalRollCallback, this);
    sub_gimbal_pitch_ = nh_.subscribe(gimbal_pitch_topic, 10,
                                      &PlandDetectorNode::gimbalPitchCallback, this);
    sub_gimbal_yaw_ = nh_.subscribe(gimbal_yaw_topic, 10,
                                    &PlandDetectorNode::gimbalYawCallback, this);

    // 7. 订阅图像输入
    sub_image_ = nh_.subscribe(image_topic, 1, &PlandDetectorNode::imageCallback, this);

    // 8. Reset 话题订阅
    sub_reset_ = nh_.subscribe(reset_topic, 10, &PlandDetectorNode::resetTopicCallback, this);

    // 9. 订阅控制器状态机状态 (用于在调试图像 OSD 中显示当前任务阶段)
    sub_state_ = nh_.subscribe(state_topic, 10, &PlandDetectorNode::stateCallback, this);

    ROS_INFO("[PlandDetectorNode] Initialized.");
    ROS_INFO("[PlandDetectorNode] Subscribing to odom: %s", odom_topic.c_str());
    ROS_INFO("[PlandDetectorNode] Subscribing to image: %s", image_topic.c_str());
    ROS_INFO("[PlandDetectorNode] Subscribing to reset: %s", reset_topic.c_str());
    ROS_INFO("[PlandDetectorNode] Subscribing to gimbal: roll=%s, pitch=%s, yaw=%s",
             gimbal_roll_topic.c_str(), gimbal_pitch_topic.c_str(), gimbal_yaw_topic.c_str());
    ROS_INFO("[PlandDetectorNode] Initial detection status: %s",
             detection_enabled_ ? "ENABLED" : "DISABLED");
  }

private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    double t = msg->header.stamp.toSec();
    if (t <= 0.0) {
      t = ros::Time::now().toSec();
    }

    Eigen::Vector3d pos(msg->pose.pose.position.x,
                        msg->pose.pose.position.y,
                        msg->pose.pose.position.z);
    Eigen::Quaterniond quat(msg->pose.pose.orientation.w,
                            msg->pose.pose.orientation.x,
                            msg->pose.pose.orientation.y,
                            msg->pose.pose.orientation.z);
    Eigen::Vector3d vel_ang(msg->twist.twist.angular.x,
                            msg->twist.twist.angular.y,
                            msg->twist.twist.angular.z);

    detector_->update_pose(t, pos);
    detector_->update_quat(t, quat);
    detector_->update_angular_rate(vel_ang);
  }

  void gimbalRollCallback(const std_msgs::Float64::ConstPtr &msg) {
    double r = msg->data;
    if (gimbal_in_degrees_) {
      r = r * (M_PI / 180.0);
    }
    gimbal_roll_ = r;
    detector_->update_gimbal(ros::Time::now().toSec(), gimbal_roll_, gimbal_pitch_, gimbal_yaw_);
  }

  void gimbalPitchCallback(const std_msgs::Float64::ConstPtr &msg) {
    double p = msg->data;
    if (gimbal_in_degrees_) {
      p = p * (M_PI / 180.0);
    }
    gimbal_pitch_ = p;
    detector_->update_gimbal(ros::Time::now().toSec(), gimbal_roll_, gimbal_pitch_, gimbal_yaw_);
  }

  void gimbalYawCallback(const std_msgs::Float64::ConstPtr &msg) {
    double y = msg->data;
    if (gimbal_in_degrees_) {
      y = y * (M_PI / 180.0);
    }
    gimbal_yaw_ = y;
    detector_->update_gimbal(ros::Time::now().toSec(), gimbal_roll_, gimbal_pitch_, gimbal_yaw_);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
    if (!detection_enabled_) {
      return;
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception &e) {
      ROS_ERROR_STREAM_THROTTLE(2.0, "[PlandDetectorNode] cv_bridge exception: " << e.what());
      return;
    }

    double t = msg->header.stamp.toSec();
    if (t <= 0.0) {
      t = ros::Time::now().toSec();
    }

    detector_->update_image(t, cv_ptr->image);

    DetectorResult result;
    detector_->detect(result);

    if (publish_debug_image_ && debug_image_pub_.getNumSubscribers() > 0 && !result.detected.empty()) {
      draw_osd(result.detected, result);
      sensor_msgs::ImagePtr out_msg =
          cv_bridge::CvImage(msg->header, "bgr8", result.detected).toImageMsg();
      debug_image_pub_.publish(out_msg);
    }
  }

  void resetTopicCallback(const std_msgs::Empty::ConstPtr &) {
    ROS_INFO("[PlandDetectorNode] Reset command received on topic. Resetting filters and tracker.");
    detector_->reset();
  }

  void stateCallback(const std_msgs::String::ConstPtr &msg) {
    if (!msg->data.empty()) {
      fsm_state_ = msg->data;
    }
  }

  // 在调试图像上绘制 OSD 状态面板 (状态机阶段 / 目标运动状态 / 速度 / 高度 / 对准误差)
  void draw_osd(cv::Mat &img, const DetectorResult &result) {
    if (img.empty()) {
      return;
    }

    const int font_face = cv::FONT_HERSHEY_SIMPLEX;
    const double scale = std::max(0.4, img.cols / 640.0 * 0.5);
    const int thickness = std::max(1, static_cast<int>(std::round(scale * 1.6)));
    const int line_h = static_cast<int>(std::round(24 * scale)) + 10;
    const int margin_x = static_cast<int>(std::round(10 * scale)) + 2;
    const int margin_y = static_cast<int>(std::round(8 * scale)) + 2;

    const cv::Scalar white(255, 255, 255);
    const cv::Scalar green(80, 220, 80);
    const cv::Scalar orange(0, 165, 255);
    const cv::Scalar red(60, 60, 255);
    const cv::Scalar cyan(200, 220, 80);

    char buf[96];
    std::vector<std::string> lines;
    std::vector<cv::Scalar> colors;

    // 1. 控制器状态机阶段
    lines.emplace_back("FSM: " + fsm_state_);
    colors.push_back(cyan);

    // 2. 目标运动状态与速度 (CTRV EKF 估计)
    if (result.is_valid) {
      std::snprintf(buf, sizeof(buf), "TARGET: %s  SPD: %.2f m/s",
                    result.target_moving ? "MOVING" : "STATIONARY",
                    result.target_speed);
      lines.emplace_back(buf);
      colors.push_back(result.target_moving ? orange : green);
    } else {
      lines.emplace_back("TARGET: LOST");
      colors.push_back(red);
    }

    // 3. 当前飞行高度
    std::snprintf(buf, sizeof(buf), "ALT: %.2f m", result.drone_z);
    lines.emplace_back(buf);
    colors.push_back(white);

    // 4. 与目标的对准误差 (机体系 FLU: 水平误差 / 垂直高度差 / 偏航误差)
    if (result.is_valid) {
      double err_xy =
          Eigen::Vector2d(result.target_pos_body.x(), result.target_pos_body.y()).norm();
      std::snprintf(buf, sizeof(buf), "ERR_XY: %.2f m  ERR_H: %.2f m", err_xy,
                    result.target_pos_body.z());
      lines.emplace_back(buf);
      colors.push_back(white);

      std::snprintf(buf, sizeof(buf), "YAW_ERR: %.1f deg",
                    result.target_yaw_body * 180.0 / M_PI);
      lines.emplace_back(buf);
      colors.push_back(white);
    } else {
      lines.emplace_back("ERR_XY: --  ERR_H: --");
      colors.push_back(white);
      lines.emplace_back("YAW_ERR: --");
      colors.push_back(white);
    }

    // 面板尺寸随文本自适应
    int text_w = 0;
    int text_h = 0;
    for (const auto &line : lines) {
      cv::Size s = cv::getTextSize(line, font_face, scale, thickness, nullptr);
      text_w = std::max(text_w, s.width);
      text_h = std::max(text_h, s.height);
    }
    int panel_w = std::min(text_w + 2 * margin_x, img.cols);
    int panel_h = std::min(margin_y * 2 + text_h +
                               static_cast<int>(lines.size() - 1) * line_h,
                           img.rows);

    // 半透明黑色背景，保证亮暗画面下文字均可读
    cv::Mat roi = img(cv::Rect(0, 0, panel_w, panel_h));
    cv::Mat overlay(roi.size(), roi.type(), cv::Scalar(0, 0, 0));
    cv::addWeighted(overlay, 0.55, roi, 0.45, 0.0, roi);

    for (size_t i = 0; i < lines.size(); ++i) {
      cv::Point org(margin_x,
                    margin_y + text_h + static_cast<int>(i) * line_h);
      cv::putText(img, lines[i], org, font_face, scale, colors[i], thickness,
                  cv::LINE_AA);
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  std::unique_ptr<PlandDetector> detector_;
  ros_param_sync::ParamSync sync_;

  ros::Subscriber sub_odom_;
  ros::Subscriber sub_image_;
  ros::Subscriber sub_gimbal_roll_;
  ros::Subscriber sub_gimbal_pitch_;
  ros::Subscriber sub_gimbal_yaw_;
  ros::Subscriber sub_reset_;
  ros::Subscriber sub_state_;

  ros::Publisher debug_image_pub_;

  std::string fsm_state_ = "UNKNOWN";

  bool gimbal_in_degrees_ = true;
  bool publish_debug_image_ = true;
  bool detection_enabled_ = true;

  std::optional<double> gimbal_roll_;
  std::optional<double> gimbal_pitch_;
  std::optional<double> gimbal_yaw_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "pland_detector_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  PlandDetectorNode node(nh, pnh);

  ros::spin();
  return 0;
}
