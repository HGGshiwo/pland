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

Eigen::Vector3d PlandController::get_ff_vel_body() const {
  if (!use_ff_vel_) {
    return Eigen::Vector3d::Zero();
  }

  double current_yaw = yaw_enu_;
  Eigen::Vector3d target_vel = detector_target_vel_enu_;

  // 旋转矩阵：将 ENU 速度投影到当前的机体前方 (X) 和左方/右方 (Y) (Body FLU 系)
  double cos_y = std::cos(current_yaw);
  double sin_y = std::sin(current_yaw);

  Eigen::Vector3d ff_vel_body = Eigen::Vector3d::Zero();
  ff_vel_body.x() = target_vel.x() * cos_y + target_vel.y() * sin_y;
  ff_vel_body.y() = -target_vel.x() * sin_y + target_vel.y() * cos_y;

  if (max_ff_vel_ > 0.0 && ff_vel_body.norm() > max_ff_vel_) {
    ff_vel_body = ff_vel_body.normalized() * max_ff_vel_;
  }
  return ff_vel_body;
}

Eigen::Vector4d PlandController::get_tracing_detector_target_vel() {
  Eigen::Vector3d err_body = detector_target_pos_body_;
  double err_yaw = detector_target_yaw_body_;
  double ff_omega = detector_target_vel_enu_.z(); // 目标偏航角速度前馈
  double current_z = get_current_z();

  // 1. 低空保护：低于盲降门限时锁死偏航角与偏航角速度，只进行平移修正，给无人机充足的高度完成偏航对齐
  double yaw_lock_alt = std::max(0.2, blind_drop_alt_);
  if (current_z < yaw_lock_alt) {
    err_yaw = 0.0;
    ff_omega = 0.0;
  }

  auto ff_vel_body = get_ff_vel_body();

  // 2. 偏航-平移协同降速：严格参考 dankong/include/features/pland/landing_controller.hpp 第 651-658 行
  // 当航向误差大时，衰减水平速度，优先自旋对齐机头，防止大自转与平移耦合产生切向画圈
  double abs_yaw_err_deg = std::abs(err_yaw) * 180.0 / M_PI;
  double yaw_penalty = 1.0;
  if (abs_yaw_err_deg > 20.0) {
    yaw_penalty = std::max(0.1, 1.0 - 0.9 * (abs_yaw_err_deg - 20.0) / 40.0);
  }

  // 水平速度指令 (反馈速度受 max_speed_xy * yaw_penalty 限幅，前馈速度受 max_ff_vel 限幅)
  double Kp_xy = vision_kp_;
  double max_v_xy = max_speed_xy_ * yaw_penalty;

  Eigen::Vector2d fb_vel_xy(Kp_xy * err_body.x(), Kp_xy * err_body.y());
  if (max_v_xy > 0.0 && fb_vel_xy.norm() > max_v_xy) {
    fb_vel_xy = fb_vel_xy.normalized() * max_v_xy;
  }

  Eigen::Vector2d vel_xy = fb_vel_xy + ff_vel_body.head<2>();

  // 3. 偏航角速度指令 (包含角速度前馈)
  double Kp_yaw = gamma_yaw_ > 0.0 ? gamma_yaw_ : 1.0;
  double omega_z = Kp_yaw * err_yaw + ff_omega;
  omega_z = std::clamp(omega_z, -0.5, 0.5);

  // 4. 垂直下降速度计算 (动态漏斗对齐控制)
  double xy_error_norm = err_body.head<2>().norm();

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
  double descent_vel = 0.0;
  if (descent_factor > 0.05) {
    double takeoff_alt = 10.0;
    double alt_ratio =
        std::min(1.0, std::abs(current_z / std::max(takeoff_alt, 1.0)));
    double base_descent_vel = 0.2 + alt_ratio * 0.8;
    descent_vel = base_descent_vel * descent_factor;
  }

  if (max_vel_z_ > 0.0) {
    descent_vel = std::min(descent_vel, max_vel_z_);
  }

  Eigen::Vector4d vel_cmd;
  vel_cmd.x() = vel_xy.x();
  vel_cmd.y() = vel_xy.y();
  vel_cmd.z() = -descent_vel; // FLU 下负向为下降
  vel_cmd.w() = omega_z;      // 偏航角速度
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

    if (!detector_target_valid()) {
      if (inject_target_valid()) {
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

    double current_z = get_current_z();
    double xy_error_norm = detector_target_pos_body_.head<2>().norm();

    // 盲降状态判定与切入
    if (current_z < blind_drop_alt_ && xy_error_norm < blind_drop_xy_thresh_) {
      change_state(ControllerState::BLIND_DROP);
      return;
    }

    auto raw_target_vel = get_tracing_detector_target_vel();
    double total_max_speed_xy =
        max_speed_xy_ + (use_ff_vel_ ? max_ff_vel_ : 0.0);
    auto smooth_vel = velocity_smoother_.apply_constraints(
        raw_target_vel, yaw_enu_, dt,
        total_max_speed_xy, max_vel_z_, 0.5,
        acc_xy, decel_xy);
    cmd_vel(smooth_vel);
    return;
  }

  case ControllerState::BLIND_DROP: {
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
    Eigen::Vector3d ff_vel = get_ff_vel_body();
    Eigen::Vector4d raw_vel(ff_vel.x(), ff_vel.y(), -touchdown_velocity_, 0.0);
    auto smooth_vel = velocity_smoother_.apply_constraints(
        raw_vel, yaw_enu_, dt,
        max_speed_xy_, max_vel_z_, 0.5,
        acc_xy, decel_xy);
    cmd_vel(smooth_vel);
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
        acc_xy, decel_xy);
    cmd_vel(smooth_vel);
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
                                         const Eigen::Vector3d &vel_enu) {
  std::lock_guard<std::mutex> lk(state_mtx_);
  pos_enu_ = pos_enu;
  orientation_ = orientation;
  vel_enu_ = vel_enu;

  // 提取 FLU 机体系相对 ENU 的偏航角
  yaw_enu_ = std::atan2(
      2.0 * (orientation.w() * orientation.z() + orientation.x() * orientation.y()),
      1.0 - 2.0 * (orientation.y() * orientation.y() + orientation.z() * orientation.z()));
}

void PlandController::update_detector_target(
    double stamp, const Eigen::Vector3d &target_pos_body,
    const Eigen::Vector3d &target_vel_enu, double target_yaw_body) {
  std::lock_guard<std::mutex> lk(state_mtx_);
  detector_target_stamp_ = stamp;
  detector_target_pos_body_ = target_pos_body;
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


