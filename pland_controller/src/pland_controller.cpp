#include "pland_controller.hpp"
#include <GeographicLib/UTMUPS.hpp>

PlandController::PlandController() {
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  init(nh, pnh);
}

PlandController::PlandController(ros::NodeHandle &nh, ros::NodeHandle &pnh) {
  init(nh, pnh);
}

void PlandController::init(ros::NodeHandle &nh, ros::NodeHandle &pnh) {
  nh_ = nh;

  // 1. 读取仅用于构造与通信连接的静态配置 (直接在 launch 中指定)
  pnh.param<std::string>("gps_topic", gps_topic_,
                         "/mavros/global_position/global");
  // 指令交付坐标系: "enu"=FRAME_LOCAL_NED 直发(默认), "body"=转回机体系 FRAME_BODY_NED
  pnh.param<std::string>("command_frame", command_frame_, "enu");
  if (command_frame_ != "enu" && command_frame_ != "body") {
    ROS_WARN("[PlandController] invalid command_frame '%s', fallback to 'enu'",
             command_frame_.c_str());
    command_frame_ = "enu";
  }
  pnh.param<std::string>("target_pose_topic", target_pose_topic_,
                         "/pland/target_pose");
  pnh.param<std::string>("target_vel_topic", target_vel_topic_,
                         "/pland/target_vel");
  pnh.param<std::string>("cmd_vel_topic", cmd_vel_topic_,
                         "/pland/cmd_vel");
  pnh.param<std::string>("setpoint_raw_topic", setpoint_raw_topic_,
                         "/mavros/setpoint_raw/local");
  pnh.param<std::string>("command_service", command_service_,
                         "/mavros/cmd/command");
  pnh.param<std::string>("set_mode_service", set_mode_service_,
                         "/mavros/set_mode");

  // 2. 初始化发布者与服务客户端
  cmd_vel_pub_ =
      nh_.advertise<geometry_msgs::TwistStamped>(cmd_vel_topic_, 10);
  setpoint_raw_pub_ =
      nh_.advertise<mavros_msgs::PositionTarget>(setpoint_raw_topic_, 10);
  command_client_ = nh_.serviceClient<mavros_msgs::CommandLong>(command_service_);
  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>(set_mode_service_);

  // 3. 订阅飞控 GPS 话题
  gps_sub_ = nh_.subscribe(gps_topic_, 10, &PlandController::gpsCallback, this);

  reset();
}

void PlandController::bind_dynamic_params(ros_param_sync::ParamSync &sync) {
  // 动态参数：直接绑定变量，从 YAML/rosparam 读取，未提供则报错跳过，不硬编码默认值
  // 降落高度与速度阈值
  sync.bind("touchdown_velocity", touchdown_velocity_);
  sync.bind("touchdown_z_thresh", touchdown_z_thresh_);
  sync.bind("lost_target_alt", lost_target_alt_);

  // 加速度与速度限制
  sync.bind("max_acc_xy", max_acc_xy_);
  sync.bind("max_devel_xy", max_devel_xy_);
  sync.bind("max_speed_xy", max_speed_xy_);
  sync.bind("max_vel_z", max_vel_z_);
  sync.bind("decay_start_z", decay_start_z_);
  sync.bind("limit_start_z", limit_start_z_);

  // 反馈与控制增益
  sync.bind("gamma_yaw", gamma_yaw_);
  sync.bind("max_gamma_xy", max_gamma_xy_);
  sync.bind("min_gamma_xy", min_gamma_xy_);
  sync.bind("max_gamma_z", max_gamma_z_);
  sync.bind("vision_kp", vision_kp_);
  sync.bind("vision_kd", vision_kd_);
  sync.bind("max_yaw_rate", max_yaw_rate_);
  sync.bind("yaw_ff_tau", yaw_ff_tau_);

  // 对齐容差与阈值
  sync.bind("xy_align_thresh", xy_align_thresh_);
  sync.bind("yaw_align_thresh", yaw_align_thresh_);

  // 盲降与判决高度
  sync.bind("blind_drop_xy_thresh", blind_drop_xy_thresh_);
  sync.bind("blind_drop_alt", blind_drop_alt_);
  sync.bind("exit_alt", exit_alt_);

  // 悬停降落距离自适应漏斗
  sync.bind("max_hold_dist_thresh", max_hold_dist_thresh_);
  sync.bind("min_hold_dist_thresh", min_hold_dist_thresh_);
  sync.bind("min_hold_dist_thresh_alt", min_hold_dist_thresh_alt_);
  sync.bind("max_funnel_radius", max_funnel_radius_);
  sync.bind("funnel_radius_k", funnel_radius_k_);

  // 前馈与爬升
  sync.bind("max_ff_vel", max_ff_vel_);
  sync.bind("lost_climb_vel", lost_climb_vel_);

  // 行为与模式开关
  sync.bind("use_disarm", use_disarm_);
  sync.bind("use_ff_vel", use_ff_vel_);
  sync.bind("baseline_mode", baseline_mode_);
  sync.bind("target_timeout", target_timeout_);
  sync.bind("target_distance", target_distance_);
  sync.bind("target_distance_hysteresis", target_distance_hysteresis_);
}

void PlandController::reset() {
  std::lock_guard<std::mutex> lk(state_mtx_);
  current_state_ = ControllerState::IDLE;
  has_inject_gps_target_ = false;
  inject_target_converted_ = false;
  inject_target_stamp_ = 0.0;
  detector_target_stamp_ = 0.0;
  last_step_time_ = 0.0;
  ff_omega_filtered_ = 0.0;
  last_ff_omega_time_ = ros::Time(0);
  velocity_smoother_.reset();
  land_triggered_ = false;
  has_rangefinder_ = false;
  rangefinder_alt_ = -1.0;
  rangefinder_stamp_ = 0.0;
  has_rel_alt_ = false;
  rel_alt_ = 0.0;
  rel_alt_stamp_ = 0.0;
}

void PlandController::change_state(ControllerState target) {
  state_exit();
  current_state_ = target;
  state_enter();
}

void PlandController::state_enter() {
  switch (current_state_) {
  case ControllerState::IDLE:
    ROS_INFO("[PlandController] State -> IDLE");
    break;
  case ControllerState::TRACING_GPS:
    ROS_INFO("[PlandController] State -> TRACING_GPS");
    break;
  case ControllerState::TRACING_DETECTOR:
    ROS_INFO("[PlandController] State -> TRACING_DETECTOR");
    velocity_smoother_.reset(Eigen::Vector4d::Zero(), yaw_enu_);
    break;
  case ControllerState::TARGET_LOST:
    ROS_WARN("[PlandController] State -> TARGET_LOST");
    break;
  case ControllerState::BLIND_DROP:
    ROS_WARN("[PlandController] State -> BLIND_DROP");
    break;
  case ControllerState::LANDED:
    ROS_INFO("[PlandController] State -> LANDED");
    if (!land_triggered_) {
      land_triggered_ = true;
      if (use_disarm_) {
        trigger_disarm();
      } else {
        trigger_land();
      }
      cmd_vel(Eigen::Vector4d::Zero());
    }
    break;
  }
}

void PlandController::state_exit() {
  // 状态退出清理钩子
}

bool PlandController::inject_target_valid() const {
  if (inject_target_stamp_ <= 0.0) return false;
  return (ros::Time::now().toSec() - inject_target_stamp_) < target_timeout_;
}

bool PlandController::detector_target_valid() const {
  if (detector_target_stamp_ <= 0.0) return false;
  return (ros::Time::now().toSec() - detector_target_stamp_) < target_timeout_;
}

Eigen::Vector3d PlandController::get_ff_vel_enu() const {
  if (!use_ff_vel_) {
    return Eigen::Vector3d::Zero();
  }

  // detector 的目标速度本就是 ENU, 直接限幅返回 (无需机体系投影)
  Eigen::Vector3d ff_vel_enu = detector_target_vel_enu_;
  if (max_ff_vel_ > 0.0 && ff_vel_enu.norm() > max_ff_vel_) {
    ff_vel_enu = ff_vel_enu.normalized() * max_ff_vel_;
  }
  return ff_vel_enu;
}

Eigen::Vector2d PlandController::get_drone_vel_enu_xy() const {
  // vel_enu_ 已在 update_drone_state 中由机体系 twist 经全姿态旋转得到:
  // vel_enu_ = orientation_ * vel_body (ArduPilot 与真机 mavros 的 odom twist 均为机体系)
  return vel_enu_.head<2>();
}

Eigen::Vector4d PlandController::get_tracing_detector_target_vel() {
  // 水平环全程在 ENU 世界系合成:
  // 误差 e_enu 由 detector 在图像时刻锚定 (仅旋转, 不随机头漂移);
  // 阻尼速度 v_enu 由机体系 twist 经全姿态旋转得到 (update_drone_state);
  // 前馈本就是 ENU (detector EKF)。合成结果按 command_frame 交付 (LOCAL_NED 或转回 body)。
  Eigen::Vector2d err_enu = detector_err_enu_.head<2>();
  double err_yaw = detector_target_yaw_body_;
  double ff_omega = detector_target_vel_enu_.z(); // 目标偏航角速度前馈
  double current_z = get_current_z();

  // 1. 偏航前馈一阶惯性滞后滤波 (Low-Pass Lag Filter)
  // 保持稳态大弯无损跟踪 (增益 1.0)，平滑过滤高频蛇形甩头与机动抖动
  double ff_omega_raw = detector_target_vel_enu_.z();
  ros::Time now = ros::Time::now();
  double dt_ff = last_ff_omega_time_.isZero() ? 0.033 : (now - last_ff_omega_time_).toSec();
  last_ff_omega_time_ = now;
  dt_ff = std::clamp(dt_ff, 0.001, 0.2);

  double alpha = dt_ff / std::max(0.01, yaw_ff_tau_ + dt_ff);
  ff_omega_filtered_ += alpha * (ff_omega_raw - ff_omega_filtered_);

  // 低空盲降保护：低于盲降门限时锁死偏航角与偏航角速度，只进行平移修正与触地盲降
  double yaw_lock_alt = std::max(0.2, blind_drop_alt_);
  if (current_z < yaw_lock_alt) {
    err_yaw = 0.0;
    ff_omega_filtered_ = 0.0;
  }

  Eigen::Vector2d ff_vel_enu = get_ff_vel_enu().head<2>();

  double xy_error_norm = err_enu.norm();

  Eigen::Vector2d vel_xy_enu;
  double omega_z = 0.0;
  double descent_vel = 0.0;

  if (baseline_mode_) {
    // === 常规 PID 模式 (Conventional PID Baseline) ===
    // 1. 无机体速度微分阻尼 (Kd = 0): 纯比例位置反馈 + 目标前馈
    Eigen::Vector2d fb_vel_enu = vision_kp_ * err_enu;
    if (max_speed_xy_ > 0.0 && fb_vel_enu.norm() > max_speed_xy_) {
      fb_vel_enu = fb_vel_enu.normalized() * max_speed_xy_;
    }
    vel_xy_enu = fb_vel_enu + ff_vel_enu;

    // 2. 紧耦合偏航 (无解耦, 权重恒为 1.0)
    double Kp_yaw = gamma_yaw_ > 0.0 ? gamma_yaw_ : 1.5;
    double max_w = max_yaw_rate_ > 0.0 ? max_yaw_rate_ : 1.2;
    omega_z = std::clamp(Kp_yaw * err_yaw + ff_omega_filtered_, -max_w, max_w);

    // 3. 常规恒速下降 (无动态漏斗约束)
    descent_vel = 0.5;
  } else {
    // === 本文提出的解耦 + 阻尼 + 漏斗模式 (Proposed Method) ===

    // 2. 偏航-平移解耦控制 (位置优先，防止远距离大旋转产生离心画圈)

    // 水平速度指令 (加入相对速度阻尼项 Kd_xy，消除左右晃动与超调)
    double Kp_xy = vision_kp_;
    double Kd_xy = vision_kd_;
    double max_v_xy = max_speed_xy_;

    Eigen::Vector2d v_drone_enu_xy = get_drone_vel_enu_xy();
    Eigen::Vector2d v_rel_xy = v_drone_enu_xy - ff_vel_enu;

    Eigen::Vector2d fb_vel_enu = Kp_xy * err_enu - Kd_xy * v_rel_xy;
    if (max_v_xy > 0.0 && fb_vel_enu.norm() > max_v_xy) {
      fb_vel_enu = fb_vel_enu.normalized() * max_v_xy;
    }

    vel_xy_enu = fb_vel_enu + ff_vel_enu;

    // 3. 偏航角速度指令：平滑自适应位置解耦 (远距离全力平移对中，正上方全额对齐机头)
    double yaw_weight = 1.0;
    double max_decouple_radius = std::max(0.8, current_z * 0.15); // 随着高度平滑自适应放宽
    double min_decouple_radius = 0.35;

    if (xy_error_norm > max_decouple_radius) {
      yaw_weight = 0.0; // 远距离严禁偏航自转，平飞平移直冲靶心，消除离心画圈
    } else if (xy_error_norm > min_decouple_radius) {
      yaw_weight = 1.0 - (xy_error_norm - min_decouple_radius) /
                             (max_decouple_radius - min_decouple_radius);
    } else {
      yaw_weight = 1.0; // 接近靶标正上方，全额开启偏航对齐
    }

    double Kp_yaw = gamma_yaw_ > 0.0 ? gamma_yaw_ : 1.5;
    omega_z = (Kp_yaw * err_yaw + ff_omega_filtered_) * yaw_weight;
    double max_w = max_yaw_rate_ > 0.0 ? max_yaw_rate_ : 1.2;
    omega_z = std::clamp(omega_z, -max_w, max_w);

    // 4. 垂直下降速度计算 (动态漏斗对齐控制)

    double align_dist_thresh =
        std::max(max_funnel_radius_, current_z * funnel_radius_k_);
    double hold_dist_height_base = 10.0;

    double hold_dist_thresh = 0.0;
    if (current_z <= min_hold_dist_thresh_alt_) {
      hold_dist_thresh = min_hold_dist_thresh_;
    } else {
      double factor =
          (std::min(current_z, hold_dist_height_base) - min_hold_dist_thresh_alt_) /
          std::max(0.01, hold_dist_height_base - min_hold_dist_thresh_alt_);
      hold_dist_thresh = min_hold_dist_thresh_ +
                         factor * (max_hold_dist_thresh_ - min_hold_dist_thresh_);
    }

    double xy_descent_factor = 1.0;
    if (xy_error_norm > hold_dist_thresh) {
      xy_descent_factor = 0.0;
    } else if (xy_error_norm > align_dist_thresh) {
      xy_descent_factor =
          1.0 - (xy_error_norm - align_dist_thresh) /
                    std::max(0.001, hold_dist_thresh - align_dist_thresh);
    }

    double abs_yaw_err_deg = std::abs(err_yaw) * 180.0 / M_PI;
    double yaw_descent_min_deg = 15.0;
    double yaw_descent_max_deg = 35.0;
    double yaw_descent_factor = 1.0;
    if (abs_yaw_err_deg > yaw_descent_max_deg) {
      yaw_descent_factor = 0.0;
    } else if (abs_yaw_err_deg > yaw_descent_min_deg) {
      yaw_descent_factor =
          1.0 - (abs_yaw_err_deg - yaw_descent_min_deg) /
                    (yaw_descent_max_deg - yaw_descent_min_deg);
    }

    double descent_factor = xy_descent_factor * yaw_descent_factor;
    if (descent_factor > 0.05) {
      double takeoff_alt = 10.0;
      double alt_ratio =
          std::min(1.0, std::abs(current_z / std::max(takeoff_alt, 1.0)));
      double base_descent_vel = 0.2 + alt_ratio * 0.8;
      descent_vel = base_descent_vel * descent_factor;
    }
  }

  if (max_vel_z_ > 0.0) {
    descent_vel = std::min(descent_vel, max_vel_z_);
  }

  Eigen::Vector4d vel_cmd;
  vel_cmd.x() = vel_xy_enu.x();
  vel_cmd.y() = vel_xy_enu.y();
  vel_cmd.z() = -descent_vel; // FLU/ENU 下负向为下降
  vel_cmd.w() = omega_z;      // 偏航角速度 (系不变)
  return vel_cmd;
}

void PlandController::cmd_vel(const Eigen::Vector4d &vel_body) {
  // 发布至 mavros_msgs::PositionTarget
  // MAVROS 的 setpoint_raw 插件在指定 FRAME_BODY_NED 时，输入要求为 ROS 标准机体系 (base_link / FLU)。
  // MAVROS 内部会自动调用 ftf::transform_frame_baselink_aircraft 转换为飞控底层的 FRD 系 (X->X, Y->-Y, Z->-Z)。
  // 因此此处必须直接赋值 FLU 速度 (前向+X, 左向+Y, 向上+Z, 逆时针+W)，切不可手动取反，否则会导致方向完全相反！
  mavros_msgs::PositionTarget cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "base_link";
  cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
  cmd.velocity.x = vel_body.x();   // 前向速度 (Forward: +X_flu)
  cmd.velocity.y = vel_body.y();   // 左向速度 (Left:    +Y_flu)
  cmd.velocity.z = vel_body.z();   // 垂直速度 (Up:      +Z_flu, 爬升为正，下降为负)
  cmd.yaw_rate = vel_body.w();     // 偏航角速度 (Yaw rate: +W_flu, 逆时针为正)

  cmd.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                  mavros_msgs::PositionTarget::IGNORE_AFY |
                  mavros_msgs::PositionTarget::IGNORE_AFZ |
                  mavros_msgs::PositionTarget::IGNORE_PX |
                  mavros_msgs::PositionTarget::IGNORE_PY |
                  mavros_msgs::PositionTarget::IGNORE_PZ |
                  mavros_msgs::PositionTarget::IGNORE_YAW;
  setpoint_raw_pub_.publish(cmd);

  // 同步发布至 TwistStamped (仅供外部监控与调试展示，避免未转换前与 MAVROS 产生双流冲突)
  if (cmd_vel_pub_.getNumSubscribers() > 0) {
    geometry_msgs::TwistStamped twist;
    twist.header.stamp = ros::Time::now();
    twist.header.frame_id = "base_link";
    twist.twist.linear.x = vel_body.x();
    twist.twist.linear.y = vel_body.y();
    twist.twist.linear.z = vel_body.z();
    twist.twist.angular.z = vel_body.w();
    cmd_vel_pub_.publish(twist);
  }
}

// ENU 世界系速度指令直接以 FRAME_LOCAL_NED 交付 (方案⑤)
// MAVROS 与 dankong 桥对 FRAME_LOCAL_NED 均约定 ROS 侧输入 ENU, 由其完成 ENU->NED 转换
void PlandController::cmd_vel_enu(const Eigen::Vector4d &vel_enu) {
  mavros_msgs::PositionTarget cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "map";
  cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  cmd.velocity.x = vel_enu.x();   // East
  cmd.velocity.y = vel_enu.y();   // North
  cmd.velocity.z = vel_enu.z();   // Up (下降为负)
  cmd.yaw_rate = vel_enu.w();     // 偏航角速度 (系不变)

  cmd.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                  mavros_msgs::PositionTarget::IGNORE_AFY |
                  mavros_msgs::PositionTarget::IGNORE_AFZ |
                  mavros_msgs::PositionTarget::IGNORE_PX |
                  mavros_msgs::PositionTarget::IGNORE_PY |
                  mavros_msgs::PositionTarget::IGNORE_PZ |
                  mavros_msgs::PositionTarget::IGNORE_YAW;
  setpoint_raw_pub_.publish(cmd);

  // 调试镜像 (ENU)
  if (cmd_vel_pub_.getNumSubscribers() > 0) {
    geometry_msgs::TwistStamped twist;
    twist.header.stamp = ros::Time::now();
    twist.header.frame_id = "map";
    twist.twist.linear.x = vel_enu.x();
    twist.twist.linear.y = vel_enu.y();
    twist.twist.linear.z = vel_enu.z();
    twist.twist.angular.z = vel_enu.w();
    cmd_vel_pub_.publish(twist);
  }
}

void PlandController::fly_to(const Eigen::Vector3d &pos_enu,
                             const Eigen::Vector3d &vel_enu) {
  mavros_msgs::PositionTarget cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "map";
  cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
  cmd.position.x = pos_enu.x();
  cmd.position.y = pos_enu.y();
  cmd.position.z = pos_enu.z();
  cmd.velocity.x = vel_enu.x();
  cmd.velocity.y = vel_enu.y();
  cmd.velocity.z = vel_enu.z();
  cmd.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                  mavros_msgs::PositionTarget::IGNORE_AFY |
                  mavros_msgs::PositionTarget::IGNORE_AFZ |
                  mavros_msgs::PositionTarget::IGNORE_YAW |
                  mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
  setpoint_raw_pub_.publish(cmd);
}

void PlandController::trigger_land() {
  mavros_msgs::SetMode srv;
  // 优先尝试 ArduPilot 模式 (LAND)
  srv.request.custom_mode = "LAND";
  if (set_mode_client_.call(srv) && srv.response.mode_sent) {
    ROS_INFO("[PlandController] LAND command sent successfully.");
    return;
  }
  // 备选尝试 PX4 模式 (AUTO.LAND)
  srv.request.custom_mode = "AUTO.LAND";
  if (set_mode_client_.call(srv) && srv.response.mode_sent) {
    ROS_INFO("[PlandController] AUTO.LAND command sent successfully.");
    return;
  }
  ROS_WARN("[PlandController] Failed to send LAND / AUTO.LAND command.");
}

void PlandController::trigger_disarm() {
  // 发送 MAV_CMD_COMPONENT_ARM_DISARM (400) 携带 param2=21196 进行强制锁定 (Force Disarm)
  // 该指令能无视飞控在移动平台上的地速安全保护检测，直接切断电机动力
  mavros_msgs::CommandLong cmd;
  cmd.request.broadcast = false;
  cmd.request.command = 400; // MAV_CMD_COMPONENT_ARM_DISARM
  cmd.request.confirmation = 0;
  cmd.request.param1 = 0.0f; // 0: Disarm
  cmd.request.param2 = 21196.0f; // 21196: Force disarm magic number
  cmd.request.param3 = 0.0f;
  cmd.request.param4 = 0.0f;
  cmd.request.param5 = 0.0f;
  cmd.request.param6 = 0.0f;
  cmd.request.param7 = 0.0f;

  if (command_client_.call(cmd) && cmd.response.success) {
    ROS_INFO("[PlandController] Force Disarm sent successfully (result: %d).", cmd.response.result);
  } else {
    ROS_WARN("[PlandController] Failed to send Force Disarm command (result: %d).", cmd.response.result);
  }
}

void PlandController::step() {
  std::lock_guard<std::mutex> lk(state_mtx_);

  double now = ros::Time::now().toSec();
  double dt = (last_step_time_ > 0.0) ? (now - last_step_time_) : 0.02;
  dt = std::clamp(dt, 0.001, 0.1);
  last_step_time_ = now;

  double decel_xy = (max_devel_xy_ > 0.0) ? max_devel_xy_ : max_acc_xy_;
  double acc_xy = (max_acc_xy_ > 0.0) ? max_acc_xy_ : 1.0;

  switch (current_state_) {
  case ControllerState::IDLE: {
    if (detector_target_valid()) {
      change_state(ControllerState::TRACING_DETECTOR);
      return;
    }
    if (inject_target_valid()) {
      change_state(ControllerState::TRACING_GPS);
      return;
    }
    
    change_state(ControllerState::TARGET_LOST);
    return;
  }

  case ControllerState::TRACING_GPS: {
    if (!inject_target_valid()) {
      if (detector_target_valid()) {
        change_state(ControllerState::TRACING_DETECTOR);
      } else {
        change_state(ControllerState::TARGET_LOST);
      }
      return;
    }

    double dist = (inject_target_pos_enu_ - pos_enu_).head<2>().norm();
    if (dist <= target_distance_ && detector_target_valid()) {
      change_state(ControllerState::TRACING_DETECTOR);
      return;
    }

    // 执行 GPS 目标点定点巡航，飞行高度使用 TARGET_LOST 目标高度
    Eigen::Vector3d target_pos = inject_target_pos_enu_;
    target_pos.z() = lost_target_alt_;
    Eigen::Vector3d vel = inject_target_vel_enu_;
    vel.z() = 0.0;
    fly_to(target_pos, vel);
    return;
  }

  case ControllerState::TRACING_DETECTOR: {
    // 迟滞距离保护：如果 GPS 注入目标有效，检查是否超出退出距离门限
    double dist_to_gps = (inject_target_pos_enu_ - pos_enu_).head<2>().norm();
    double exit_distance =
        target_distance_ +
        (target_distance_hysteresis_ > 0.0 ? target_distance_hysteresis_ : 1.0);

    double current_z = get_current_z();

    if (!detector_target_valid()) {
      // 关键保护：低空 (< blind_drop_alt) 丢失视觉时，强制切入 BLIND_DROP 盲降触地，严禁向上爬升！
      if (current_z < blind_drop_alt_) {
        ROS_WARN("[PlandController] Target lost at low altitude (z=%.2f < %.2f). Enforcing BLIND_DROP!",
                 current_z, blind_drop_alt_);
        change_state(ControllerState::BLIND_DROP);
      } else if (inject_target_valid()) {
        change_state(ControllerState::TRACING_GPS);
      } else {
        change_state(ControllerState::TARGET_LOST);
      }
      return;
    }

    if (inject_target_valid() && dist_to_gps > exit_distance) {
      ROS_WARN(
          "[PlandController] Exceeded exit distance (%.2f > %.2f). Fallback to "
          "TRACING_GPS.",
          dist_to_gps, exit_distance);
      change_state(ControllerState::TRACING_GPS);
      return;
    }

    // 触地检测：只要视觉有效，持续全闭环对齐修正到底，触地直接停机 (彻底消除开环侧漂)
    if (current_z < exit_alt_) {
      change_state(ControllerState::LANDED);
      return;
    }

    double xy_error_norm = detector_err_enu_.head<2>().norm();

    auto raw_target_vel = get_tracing_detector_target_vel();
    double total_max_speed_xy =
        max_speed_xy_ + (use_ff_vel_ ? max_ff_vel_ : 0.0);
    double max_w = max_yaw_rate_ > 0.0 ? max_yaw_rate_ : 1.2;
    auto smooth_vel = velocity_smoother_.apply_constraints(
        raw_target_vel, yaw_enu_, dt,
        total_max_speed_xy, max_vel_z_, max_w,
        acc_xy, decel_xy,
        1000.0, 2.0, 1000.0, 3.0,
        /*world_frame=*/true);
    if (command_frame_ == "body") {
      // 方案④: ENU 合成结果转回机体系交付 (残差 span = 一个控制周期)
      double cy = std::cos(yaw_enu_);
      double sy = std::sin(yaw_enu_);
      Eigen::Vector4d vel_body;
      vel_body.x() =  cy * smooth_vel.x() + sy * smooth_vel.y();
      vel_body.y() = -sy * smooth_vel.x() + cy * smooth_vel.y();
      vel_body.z() = smooth_vel.z();
      vel_body.w() = smooth_vel.w();
      cmd_vel(vel_body);
    } else {
      // 方案⑤: 直接以 FRAME_LOCAL_NED 交付 ENU 速度
      cmd_vel_enu(smooth_vel);
    }
    return;
  }

  case ControllerState::BLIND_DROP: {
    // 若在下沉过程中重新看清目标，无缝切回高精度视觉闭环
    if (detector_target_valid()) {
      change_state(ControllerState::TRACING_DETECTOR);
      return;
    }

    double current_z = get_current_z();
    if (current_z > std::max(1.0, blind_drop_alt_ + 0.5)) {
      change_state(ControllerState::TARGET_LOST);
      return;
    }

    // 触地检测
    if (current_z < exit_alt_) {
      change_state(ControllerState::LANDED);
      return;
    }

    // 盲降期间保持前馈水平速度，以固定触地速度下沉
    Eigen::Vector3d ff_vel = get_ff_vel_enu();
    Eigen::Vector4d raw_vel(ff_vel.x(), ff_vel.y(), -touchdown_velocity_, 0.0);
    auto smooth_vel = velocity_smoother_.apply_constraints(
        raw_vel, yaw_enu_, dt,
        max_speed_xy_, max_vel_z_, 0.5,
        acc_xy, decel_xy,
        1000.0, 2.0, 1000.0, 3.0,
        /*world_frame=*/true);
    if (command_frame_ == "body") {
      double cy = std::cos(yaw_enu_);
      double sy = std::sin(yaw_enu_);
      Eigen::Vector4d vel_body;
      vel_body.x() =  cy * smooth_vel.x() + sy * smooth_vel.y();
      vel_body.y() = -sy * smooth_vel.x() + cy * smooth_vel.y();
      vel_body.z() = smooth_vel.z();
      vel_body.w() = smooth_vel.w();
      cmd_vel(vel_body);
    } else {
      cmd_vel_enu(smooth_vel);
    }
    return;
  }

  case ControllerState::TARGET_LOST: {
    if (detector_target_valid()) {
      change_state(ControllerState::TRACING_DETECTOR);
      return;
    }
    if (inject_target_valid()) {
      change_state(ControllerState::TRACING_GPS);
      return;
    }

    // 丢失后爬升至指定安全高度并闭环悬停 (Hover at lost_target_alt_)
    Eigen::Vector4d vel = Eigen::Vector4d::Zero();
    double z_err = lost_target_alt_ - get_current_z();
    double climb_vel = std::clamp(1.0 * z_err, -lost_climb_vel_, lost_climb_vel_);
    vel.z() = climb_vel;
    auto smooth_vel = velocity_smoother_.apply_constraints(
        vel, yaw_enu_, dt,
        max_speed_xy_, lost_climb_vel_, 0.5,
        acc_xy, decel_xy,
        1000.0, 2.0, 1000.0, 3.0,
        /*world_frame=*/true);
    if (command_frame_ == "body") {
      cmd_vel(smooth_vel);
    } else {
      cmd_vel_enu(smooth_vel);
    }
    return;
  }

  case ControllerState::LANDED: {
    if (!land_triggered_) {
      land_triggered_ = true;
      if (use_disarm_) {
        trigger_disarm();
      } else {
        trigger_land();
      }
      cmd_vel(Eigen::Vector4d::Zero());
    }
    return;
  }
  }
}

void PlandController::update_drone_state(double stamp,
                                         const Eigen::Vector3d &pos_enu,
                                         const Eigen::Quaterniond &orientation,
                                         const Eigen::Vector3d &vel_body) {
  std::lock_guard<std::mutex> lk(state_mtx_);
  pos_enu_ = pos_enu;
  orientation_ = orientation;
  // odom twist 为机体系 FLU (ArduPilot 与真机 mavros 一致), 经全姿态旋转到 ENU
  vel_enu_ = orientation * vel_body;

  // 提取 FLU 机体系相对 ENU 的偏航角
  yaw_enu_ = std::atan2(
      2.0 * (orientation.w() * orientation.z() + orientation.x() * orientation.y()),
      1.0 - 2.0 * (orientation.y() * orientation.y() + orientation.z() * orientation.z()));
}

void PlandController::update_detector_target(
    double stamp, const Eigen::Vector3d &target_err_enu,
    const Eigen::Vector3d &target_vel_enu, double target_yaw_body) {
  std::lock_guard<std::mutex> lk(state_mtx_);
  detector_target_stamp_ = stamp;
  detector_err_enu_ = target_err_enu; // ENU 方向轴误差 (detector 图像时刻仅旋转)
  detector_target_vel_enu_ = target_vel_enu;
  detector_target_yaw_body_ = target_yaw_body;
}

void PlandController::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
  if (msg->status.status < sensor_msgs::NavSatStatus::STATUS_FIX) {
    return;
  }
  std::lock_guard<std::mutex> lk(state_mtx_);
  drone_lat_lon_alt_ << msg->latitude, msg->longitude, msg->altitude;
  drone_pos_enu_at_gps_ = pos_enu_;
  has_drone_gps_ = true;

  // 若注入目标在无人机 GPS 到达前已设置且尚未转换过，则在此补转一次
  if (has_inject_gps_target_ && !inject_target_converted_) {
    compute_inject_target_enu();
  }
}

void PlandController::compute_inject_target_enu() {
  if (!has_drone_gps_ || !has_inject_gps_target_) {
    return;
  }
  inject_target_pos_enu_ =
      gps_to_enu(drone_lat_lon_alt_, drone_pos_enu_at_gps_, inject_target_lat_lon_alt_);
  inject_target_converted_ = true;
}

Eigen::Vector3d PlandController::gps_to_enu(
    const Eigen::Vector3d &drone_lat_lon_alt,
    const Eigen::Vector3d &cur_pos_enu,
    const Eigen::Vector3d &target_lat_lon_alt) {
  int drone_zone;
  bool is_north;
  double drone_x = 0.0, drone_y = 0.0;
  // drone_lat_lon_alt: (0) 纬度, (1) 经度, (2) 高度
  GeographicLib::UTMUPS::Forward(drone_lat_lon_alt(0), drone_lat_lon_alt(1),
                                 drone_zone, is_north, drone_x, drone_y);

  int target_zone;
  bool target_northp;
  double target_x = 0.0, target_y = 0.0;
  GeographicLib::UTMUPS::Forward(target_lat_lon_alt(0), target_lat_lon_alt(1),
                                 target_zone, target_northp, target_x, target_y, drone_zone);

  Eigen::Vector3d diff(target_x - drone_x, target_y - drone_y,
                       target_lat_lon_alt(2) - drone_lat_lon_alt(2));
  return cur_pos_enu + diff;
}

void PlandController::update_drone_gps(double stamp, const Eigen::Vector3d &drone_lat_lon_alt) {
  std::lock_guard<std::mutex> lk(state_mtx_);
  drone_lat_lon_alt_ = drone_lat_lon_alt;
  drone_pos_enu_at_gps_ = pos_enu_;
  has_drone_gps_ = true;

  if (has_inject_gps_target_ && !inject_target_converted_) {
    compute_inject_target_enu();
  }
}

void PlandController::update_inject_target(
    double stamp, const Eigen::Vector3d &target_lat_lon_alt,
    const Eigen::Vector3d &target_vel_enu) {
  std::lock_guard<std::mutex> lk(state_mtx_);
  has_inject_gps_target_ = true;
  inject_target_stamp_ = stamp;
  inject_target_lat_lon_alt_ = target_lat_lon_alt;
  inject_target_vel_enu_ = target_vel_enu;

  // 坐标更新：只有目标坐标更新时才进行坐标转换，转为 ENU 坐标系
  if (has_drone_gps_) {
    compute_inject_target_enu();
  }
}

void PlandController::update_inject_target_enu(
    double stamp, const Eigen::Vector3d &target_pos_enu,
    const Eigen::Vector3d &target_vel_enu) {
  std::lock_guard<std::mutex> lk(state_mtx_);
  has_inject_gps_target_ = false;
  inject_target_converted_ = true;
  inject_target_stamp_ = stamp;
  inject_target_pos_enu_ = target_pos_enu;
  inject_target_vel_enu_ = target_vel_enu;
}

void PlandController::update_rangefinder(double stamp, double range, bool is_valid) {
  std::lock_guard<std::mutex> lk(state_mtx_);
  if (is_valid && range > 0.0) {
    has_rangefinder_ = true;
    rangefinder_alt_ = range;
    rangefinder_stamp_ = stamp;
  }
}

void PlandController::update_rel_alt(double stamp, double rel_alt) {
  std::lock_guard<std::mutex> lk(state_mtx_);
  has_rel_alt_ = true;
  rel_alt_ = rel_alt;
  rel_alt_stamp_ = stamp;
}


