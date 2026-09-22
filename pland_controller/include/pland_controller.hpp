#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <ros/ros.h>
#include <ros_param_sync/param_sync.hpp>
#include "velocity_smoother.hpp"

enum class ControllerState {
  IDLE,             // 不进行处理
  TRACING_GPS,      // 使用GPS追踪
  TRACING_DETECTOR, // 使用检测结果追踪
  TARGET_LOST,      // 目标丢失
  BLIND_DROP,       // 盲降
  LANDED            // 顺利降落
};

class PlandController {
private:
  // --- 动态控制参数 (支持运行时修改并持久化，初始值为中性默认值，具体数值由YAML/rosparam加载) ---
  // 降落高度与速度阈值
  double touchdown_velocity_ = 0.0;      // 低于touchdown_z_thresh时使用的固定下降速度 (m/s)
  double touchdown_z_thresh_ = 0.0;      // 触发touchdown固定下降速度的高度阈值 (m)
  double lost_target_alt_ = 0.0;         // 目标丢失后的安全悬停高度 (m)

  // 加速度与速度限制
  double max_acc_xy_ = 0.0;              // 水平最大加速度 (m/s^2)
  double max_devel_xy_ = 0.0;            // 水平最大减加速度 (m/s^2)
  double max_speed_xy_ = 0.0;            // 水平最大反馈速度 (m/s)
  double max_vel_z_ = 0.0;               // 垂直最大下降速度 (m/s)
  double decay_start_z_ = 0.0;           // 反馈增益开始衰减的高度 (m)
  double limit_start_z_ = 0.0;           // 速度开始收紧限制的高度 (m)

  // 反馈与控制增益
  double gamma_yaw_ = 0.0;               // 偏航角反馈增益
  double max_gamma_xy_ = 0.0;            // 水平位置反馈增益上限
  double min_gamma_xy_ = 0.0;            // 水平位置反馈增益下限
  double max_gamma_z_ = 0.0;             // 垂直位置反馈增益上限
  double vision_kp_ = 0.0;               // 纯视觉相对速度控制增益

  // 对齐容差与阈值
  double xy_align_thresh_ = 0.0;         // 水平对齐允许下降的误差门限 (m)
  double yaw_align_thresh_ = 0.0;        // 航向对齐允许下降的误差门限 (rad)

  // 盲降与判决高度
  double blind_drop_xy_thresh_ = 0.0;    // 允许切换为盲降的水平对齐容差 (m)
  double blind_drop_alt_ = 0.0;          // 触发盲降的高度 (m)
  double exit_alt_ = 0.0;                // 最终触地关机或切降落的高度 (m)

  // 悬停降落距离自适应漏斗
  double max_hold_dist_thresh_ = 0.0;    // 高空允许下降的最大水平偏差门限 (m)
  double min_hold_dist_thresh_ = 0.0;    // 低空允许下降的最小水平偏差门限 (m)
  double min_hold_dist_thresh_alt_ = 0.0;// 采用最小门限的临界高度 (m)
  double max_funnel_radius_ = 0.0;       // 进入漏斗允许下降的最大漏斗半径 (m)
  double funnel_radius_k_ = 0.0;         // 漏斗半径斜率系数 (k * z)

  // 前馈与爬升
  double max_ff_vel_ = 0.0;              // 前馈速度限幅 (m/s)
  double lost_climb_vel_ = 0.0;          // 目标丢失后的爬升速度 (m/s)

  // 行为与模式开关
  bool use_disarm_ = false;              // 触地后是否直接上锁
  bool use_ff_vel_ = false;              // 是否使用目标前馈速度
  double target_timeout_ = 0.0;          // 视觉目标丢失超时间隔 (s)
  double target_distance_ = 0.0;         // 允许切入视觉精准降落的目标距离 (m)
  double target_distance_hysteresis_ = 0.0; // 退出视觉切回GPS的迟滞距离带宽 (m)

  // --- 静态配置参数 (在 launch / 构造函数中指定，不动态绑定) ---
  std::string target_pose_topic_ = "/pland/target_pose";
  std::string target_vel_topic_ = "/pland/target_vel";
  std::string cmd_vel_topic_ = "/pland/cmd_vel";
  std::string setpoint_raw_topic_ = "/mavros/setpoint_raw/local";
  std::string command_service_ = "/mavros/cmd/command";
  std::string set_mode_service_ = "/mavros/set_mode";
  std::string gps_topic_ = "/mavros/global_position/global";

  ros::NodeHandle nh_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher setpoint_raw_pub_;
  ros::ServiceClient command_client_;
  ros::ServiceClient set_mode_client_;
  ros::Subscriber gps_sub_;

  // --- 状态成员变量 ---
  ControllerState current_state_ = ControllerState::IDLE;
  mutable std::mutex state_mtx_;

  // 飞机本体状态 (ENU)
  Eigen::Vector3d pos_enu_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d vel_enu_ = Eigen::Vector3d::Zero();
  double yaw_enu_ = 0.0;

  // 飞机本体 GPS 状态 (lat_lon_alt: (0) 纬度, (1) 经度, (2) 高度)
  bool has_drone_gps_ = false;
  Eigen::Vector3d drone_lat_lon_alt_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d drone_pos_enu_at_gps_ = Eigen::Vector3d::Zero();

  // GPS 注入目标 (原始 GPS 经纬高 与 计算出的本地 ENU 坐标)
  bool has_inject_gps_target_ = false;
  bool inject_target_converted_ = false;
  Eigen::Vector3d inject_target_lat_lon_alt_ = Eigen::Vector3d::Zero();

  Eigen::Vector3d inject_target_pos_enu_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d inject_target_vel_enu_ = Eigen::Vector3d::Zero();
  double inject_target_stamp_ = 0.0;

  // 视觉检测目标
  Eigen::Vector3d detector_target_pos_body_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d detector_target_vel_enu_ = Eigen::Vector3d::Zero(); // z 分量为 yaw_rate
  double detector_target_yaw_body_ = 0.0;
  double detector_target_stamp_ = 0.0;

  double last_step_time_ = 0.0;
  VelocitySmoother velocity_smoother_;
  bool land_triggered_ = false;

  // 测距仪与相对高度状态
  bool has_rangefinder_ = false;
  double rangefinder_alt_ = -1.0;
  double rangefinder_stamp_ = 0.0;

  bool has_rel_alt_ = false;
  double rel_alt_ = 0.0;
  double rel_alt_stamp_ = 0.0;

  // 机架静态硬件配置 (静态加载，不参与 ParamSync 动态同步)
  double frame_height_ = 0.0;

private:
  double get_current_z() const {
    double now = ros::Time::now().toSec();
    // 基础高度优先使用相对高度 rel_alt_ (若有效且未超时 1.0s)，否则退化到 pos_enu_.z()
    double base_z = std::abs(pos_enu_.z());
    if (has_rel_alt_ && (now - rel_alt_stamp_ < 1.0)) {
      base_z = std::abs(rel_alt_);
    }

    // 检查测距仪是否有效 (有效、数据大于0、未超时 0.5s)
    bool rf_valid = has_rangefinder_ && (rangefinder_alt_ > 0.0) &&
                    (now - rangefinder_stamp_ < 0.5);
    if (!rf_valid) {
      return base_z;
    }

    double min_alt = 1.0;
    double max_alt = 3.0;
    // 测距仪进行机架物理高度补偿：起落架底端距离平面的真实高度
    double rf_z = std::max(0.0, rangefinder_alt_ - frame_height_);

    if (base_z < min_alt) {
      return rf_z;
    } else if (base_z > max_alt) {
      return base_z;
    } else {
      double ratio = (base_z - min_alt) / (max_alt - min_alt);
      return (1.0 - ratio) * rf_z + ratio * base_z;
    }
  }

  void state_enter();
  void state_exit();

  bool inject_target_valid() const;
  bool detector_target_valid() const;

  Eigen::Vector3d get_ff_vel_body() const;
  Eigen::Vector4d get_tracing_detector_target_vel();

  void cmd_vel(const Eigen::Vector4d &vel_body);
  void fly_to(const Eigen::Vector3d &pos_enu, const Eigen::Vector3d &vel_enu);
  void trigger_land();
  void trigger_disarm();

  void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);
  void compute_inject_target_enu();

public:
  PlandController();
  PlandController(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  ~PlandController() = default;

  void init(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  void reset();

  /**
   * @brief 绑定支持运行时动态修改并持久化的控制参数
   */
  void bind_dynamic_params(ros_param_sync::ParamSync &sync);

  // 状态机流转与执行
  void change_state(ControllerState target);
  void step();

  // 状态与观测数据更新接口
  void update_drone_state(double stamp, const Eigen::Vector3d &pos_enu,
                          const Eigen::Quaterniond &orientation,
                          const Eigen::Vector3d &vel_enu);
  void update_drone_gps(double stamp, const Eigen::Vector3d &drone_lat_lon_alt);
  void update_rangefinder(double stamp, double range, bool is_valid);
  void update_rel_alt(double stamp, double rel_alt);

  void update_detector_target(double stamp,
                              const Eigen::Vector3d &target_pos_body,
                              const Eigen::Vector3d &target_vel_enu,
                              double target_yaw_body);

  /**
   * @brief 更新注入的 GPS 目标（仅在目标坐标更新时结合此时无人机 GPS 及 odom 转换为本地 ENU 坐标）
   * @param target_lat_lon_alt 目标经纬高 [纬度(deg), 经度(deg), 高度(m)]
   * @param target_vel_enu 目标在 ENU 坐标系下的速度
   */
  void update_inject_target(double stamp,
                            const Eigen::Vector3d &target_lat_lon_alt,
                            const Eigen::Vector3d &target_vel_enu);

  /**
   * @brief 兼容直接传入 ENU 坐标注入接口
   */
  void update_inject_target_enu(double stamp,
                                const Eigen::Vector3d &target_pos_enu,
                                const Eigen::Vector3d &target_vel_enu);

  /**
   * @brief GPS 经纬高转换为本地 ENU 坐标
   * @param drone_lat_lon_alt 无人机参考时刻 GPS [纬度, 经度, 高度]
   * @param cur_pos_enu 无人机参考时刻在本地 ENU 坐标系下的位置
   * @param target_lat_lon_alt 目标 GPS [纬度, 经度, 高度]
   */
  static Eigen::Vector3d gps_to_enu(const Eigen::Vector3d &drone_lat_lon_alt,
                                    const Eigen::Vector3d &cur_pos_enu,
                                    const Eigen::Vector3d &target_lat_lon_alt);

  // Getters & Setters for static hardware config
  void set_frame_height(double h) {
    std::lock_guard<std::mutex> lk(state_mtx_);
    frame_height_ = h;
  }
  double get_frame_height() const {
    std::lock_guard<std::mutex> lk(state_mtx_);
    return frame_height_;
  }

  ControllerState get_state() const {
    std::lock_guard<std::mutex> lk(state_mtx_);
    return current_state_;
  }
};
