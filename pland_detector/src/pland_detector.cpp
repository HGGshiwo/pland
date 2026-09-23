#include "pland_detector.hpp"
#include <cmath>
#include <exception>

Eigen::Matrix3d PlandDetector::get_dynamic_camera_to_body_rotation(
    const Eigen::Matrix3d &hist_R_wb, std::optional<double> gimbal_roll,
    std::optional<double> gimbal_pitch, std::optional<double> gimbal_yaw,
    bool is_gimbal_absolute) const {
  // 默认相机朝下 (-90度)
  double def_roll = 0.0;
  double def_pitch = -M_PI_2;
  double def_yaw = 0.0;

  double g_roll = gimbal_roll.value_or(def_roll);
  double g_yaw = gimbal_yaw.value_or(def_yaw);
  double g_pitch;

  if (is_gimbal_absolute) {
    // 如果是绝对模式：输入的 gimbal_pitch 是相对大地的绝对下视角度
    double abs_pitch = gimbal_pitch.value_or(def_pitch);

    // 提取无人机当前的真实低头角度 (Nose Down)
    // 原理：取出 FLU 坐标系 X 轴在 ENU 世界中的投影向量
    Eigen::Vector3d forward_enu = hist_R_wb.col(0);
    double drone_pitch_down = std::atan2(
        -forward_enu.z(), std::sqrt(forward_enu.x() * forward_enu.x() +
                                    forward_enu.y() * forward_enu.y()));

    // 云台需要相对机身转动的角度 = 绝对目标角 + 飞机自身的低头角补偿
    g_pitch = abs_pitch + drone_pitch_down;
  } else {
    g_pitch = gimbal_pitch.value_or(def_pitch);
  }

  // 云台电机旋转矩阵 (在 FRD 坐标系下 Z-Y-X 旋转)
  Eigen::Matrix3d R_motor =
      (Eigen::AngleAxisd(g_yaw, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(g_pitch, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(g_roll, Eigen::Vector3d::UnitX()))
          .toRotationMatrix();

  // FRD 到 FLU 的静态转换
  Eigen::Matrix3d R_frd_to_flu;
  R_frd_to_flu << 1, 0, 0, 0, -1, 0, 0, 0, -1;

  // 相机光学系 -> FRD系 -> FLU机身系
  return R_frd_to_flu * R_motor * get_cam_to_frd_matrix();
}

PnpResultBody PlandDetector::solve_pose_by_pnp(
    const TargetPose *safe_pose, const Eigen::Matrix3d &hist_R_wb,
    std::optional<double> gimbal_roll, std::optional<double> gimbal_pitch,
    std::optional<double> gimbal_yaw, bool is_gimbal_absolute) {
  Eigen::Matrix3d R_bc = get_dynamic_camera_to_body_rotation(
      hist_R_wb, gimbal_roll, gimbal_pitch, gimbal_yaw, is_gimbal_absolute);

  Eigen::Vector3d t_bc = Eigen::Vector3d{offset_x_, offset_y_, offset_z_};

  PnpResultBody res;
  res.pos_body = t_bc + R_bc * safe_pose->t;
  res.R_tag_body = R_bc * safe_pose->R;
  res.yaw_body = std::atan2(res.R_tag_body(1, 0), res.R_tag_body(0, 0));

  return res;
}

bool PlandDetector::load_tag_config(const std::string &path) {
  if (path.empty()) return false;
  try {
    std::ifstream f(path);
    if (!f.is_open()) {
      ROS_WARN_STREAM("[PlandDetector] cannot open tag config file: " << path);
      return false;
    }
    nlohmann::json j = nlohmann::json::parse(f);
    using LayoutMap = std::map<std::string, std::vector<Eigen::Vector3d>>;
    LayoutMap layout_map;
    for (auto it = j.begin(); it != j.end(); ++it) {
      std::vector<Eigen::Vector3d> corners;
      for (const auto &pt : it.value()) {
        if (pt.is_array() && pt.size() >= 3) {
          corners.emplace_back(pt[0].get<double>(), pt[1].get<double>(), pt[2].get<double>());
        }
      }
      layout_map[it.key()] = std::move(corners);
    }
    current_pattern_ =
        std::make_unique<MultiArrayTagsPattern>(layout_map);
    tag_config_file_path_ = path;
    ROS_INFO_STREAM("[PlandDetector] success load tag config from " << path);
    return true;
  } catch (const std::exception &e) {
    ROS_ERROR_STREAM("[PlandDetector] load tag config from " << path << ", error: " << e.what());
    return false;
  } catch (...) {
    ROS_ERROR_STREAM("[PlandDetector] load tag config from " << path << ", error: Unknown");
    return false;
  }
}

PlandDetector::PlandDetector() {
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  init(nh, pnh);
}

PlandDetector::PlandDetector(ros::NodeHandle &nh, ros::NodeHandle &pnh) {
  init(nh, pnh);
}

void PlandDetector::init(ros::NodeHandle &nh, ros::NodeHandle &pnh) {
  nh_ = nh;

  // 1. 读取仅用于构造的静态配置 (直接在 launch 中指定)
  if (!pnh.getParam("tag_family", tag_family_) || tag_family_.empty()) {
    ROS_ERROR("[PlandDetector] 'tag_family' is not specified in launch / parameters!");
  }
  if (!pnh.getParam("tag_config_file_path", tag_config_file_path_) || tag_config_file_path_.empty()) {
    ROS_ERROR("[PlandDetector] 'tag_config_file_path' is not specified in launch / parameters!");
  }
  pnh.param<std::string>("target_pose_topic", target_pose_topic_, "/pland/target_pose");
  pnh.param<std::string>("target_vel_topic", target_vel_topic_, "/pland/target_vel");
  pnh.param<std::string>("debug_target_pose_topic", debug_target_pose_topic_, "/pland/target_pose_enu");
  pnh.param<std::string>("target_pixel_topic", target_pixel_topic_, "/pland/target_pixel");

  // 相机内参矩阵 (必须提供9个元素)
  std::vector<double> inner_matrix_vec;
  if (pnh.getParam("camera_inner_matrix", inner_matrix_vec) && inner_matrix_vec.size() == 9) {
    camera_inner_matrix_ << inner_matrix_vec[0], inner_matrix_vec[1], inner_matrix_vec[2],
                            inner_matrix_vec[3], inner_matrix_vec[4], inner_matrix_vec[5],
                            inner_matrix_vec[6], inner_matrix_vec[7], inner_matrix_vec[8];
  } else {
    ROS_ERROR("[PlandDetector] 'camera_inner_matrix' (9 elements) is required but missing or invalid!");
  }

  // 2. 构造底层对象
  if (!tag_family_.empty()) {
    tag_detector_ = std::make_unique<SafeAprilTagDetector>(tag_family_);
  }
  kf_xy_ = std::make_shared<KalmanFilterCTRV>();
  kf_yaw_ = std::make_shared<KalmanFilterYaw>();
  kf_abs_yaw_ = std::make_shared<KalmanFilterYaw>();
  target_tracker_ = std::make_shared<TargetTracker>();
  pose_history_ = std::make_shared<PoseHistory>();
  last_ekf_stamp_ = 0.0;

  target_pose_pub_ =
      nh_.advertise<geometry_msgs::PoseStamped>(target_pose_topic_, 10);
  target_vel_pub_ =
      nh_.advertise<geometry_msgs::TwistStamped>(target_vel_topic_, 10);
  debug_target_pose_pub_ =
      nh_.advertise<geometry_msgs::PoseStamped>(debug_target_pose_topic_, 10);
  target_pixel_pub_ =
      nh_.advertise<geometry_msgs::PointStamped>(target_pixel_topic_, 10);

  // 加载tag配置文件
  if (!tag_config_file_path_.empty()) {
    load_tag_config(tag_config_file_path_);
  }

  pnh.param<bool>("enable_c2f_enhancement", enable_c2f_enhancement_, true);
  clahe_ = cv::createCLAHE(2.0, cv::Size(8, 8));
}

void PlandDetector::bind_dynamic_params(ros_param_sync::ParamSync &sync) {
  // 动态参数：直接绑定变量，从 YAML/rosparam 读取，未提供则报错跳过
  sync.bind("velocity_deadzone", velocity_deadzone_);
  sync.bind("gimbal_abs", gimbal_abs_);
  sync.bind("enable_c2f_enhancement", enable_c2f_enhancement_);
}

void PlandDetector::reset() {
  kf_xy_->reset();
  kf_yaw_->reset();
  kf_abs_yaw_->reset();
  target_tracker_->reset();
  last_ekf_stamp_ = 0.0;
  has_last_valid_pos_ = false;
  last_valid_enu_pos_ = Eigen::Vector3d::Zero();
}

cv::Rect PlandDetector::find_texture_roi(const cv::Mat &gray) const {
  if (gray.empty()) {
    return cv::Rect(0, 0, 0, 0);
  }

  // 1. 降采样到 320x240 极速计算能量 (耗时 < 0.2ms)
  cv::Mat small;
  cv::resize(gray, small, cv::Size(320, 240), 0, 0, cv::INTER_AREA);

  // 2. Laplacian 边缘高频梯度计算
  cv::Mat lap, abs_lap;
  cv::Laplacian(small, lap, CV_16S, 3);
  cv::convertScaleAbs(lap, abs_lap);

  // 3. 盒状滤波平滑纹理能量
  cv::Mat blurred;
  cv::blur(abs_lap, blurred, cv::Size(15, 15));

  // 4. 寻找最大能量峰值
  cv::Point max_loc;
  cv::minMaxLoc(blurred, nullptr, nullptr, nullptr, &max_loc);

  int cx = max_loc.x * 2;
  int cy = max_loc.y * 2;
  int roi_size = 240;

  int x1 = std::max(0, std::min(gray.cols - roi_size, cx - roi_size / 2));
  int y1 = std::max(0, std::min(gray.rows - roi_size, cy - roi_size / 2));
  int w = std::min(roi_size, gray.cols - x1);
  int h = std::min(roi_size, gray.rows - y1);

  return cv::Rect(x1, y1, w, h);
}

void PlandDetector::detect(DetectorResult &output) {
  output.is_valid = false;

  // 1. 时间戳与历史状态获取 (时光机逻辑)
  auto stamp_tracker = image_stamp_;
  double now = ros::Time::now().toSec();
  if (stamp_tracker <= 0 || std::abs(stamp_tracker - last_ekf_stamp_) < 1e-6)
    return;

  output.stamp = stamp_tracker;

  double current_z = get_current_z();
  // OSD 状态快照：即使本帧检测失败，也携带最近一次估计的运动与高度状态
  output.drone_z = current_z;
  output.target_moving = target_tracker_ && target_tracker_->isMoving();
  output.target_speed = kf_xy_ ? kf_xy_->get_vel().norm() : 0.0;

  double elapsed = now - stamp_tracker;
  if (elapsed > 0.15) {
    ROS_WARN_STREAM("[PlandDetector] image stamp too late: " << elapsed);
    return;
  }

  Eigen::Vector3d hist_drone_pos;
  Eigen::Quaterniond hist_q;
  double min_diff = -1;
  if (!pose_history_ ||
      !pose_history_->get_pose_at(stamp_tracker, hist_drone_pos, hist_q,
                                  min_diff)) {
    hist_drone_pos = pos_enu_;
    hist_q = orientation_;
  }
  Eigen::Matrix3d hist_R_wb = hist_q.toRotationMatrix();

  // 2. 图像预处理与自适应多尺度检测
  cv::Mat current_img = pland_image_.clone();
  if (current_img.empty())
    return;

  output.detected = current_img.clone();
  SafeDetections detections(nullptr);

  if (enable_c2f_enhancement_ && current_z > 5.0) {
    // 【高空段 > 5.0m】：C2F 纹理粗检 240x240 ROI + 局域 2.0x 双三次插值超分与反锐化 (此时靶标完整落入240x240内)
    cv::Mat gray_orig;
    cv::cvtColor(current_img, gray_orig, cv::COLOR_BGR2GRAY);
    cv::Rect roi_rect = find_texture_roi(gray_orig);

    if (roi_rect.width > 0 && roi_rect.height > 0) {
      cv::Mat roi = current_img(roi_rect);
      cv::Mat roi_zoom, norm, gaussian, roi_enh, gray_zoom;
      cv::resize(roi, roi_zoom, cv::Size(), 2.0, 2.0, cv::INTER_CUBIC);
      cv::normalize(roi_zoom, norm, 10, 245, cv::NORM_MINMAX);
      cv::GaussianBlur(norm, gaussian, cv::Size(0, 0), 2.5);
      cv::addWeighted(norm, 1.8, gaussian, -0.8, 0, roi_enh);
      cv::cvtColor(roi_enh, gray_zoom, cv::COLOR_BGR2GRAY);

      detections = tag_detector_->detect(gray_zoom);

      // 将在 2x 局部 ROI (480x480) 上的角点与中心坐标精确逆映射回原图 (640x480) 空间
      for (int i = 0; i < detections.size(); ++i) {
        apriltag_detection_t* det = detections[i];
        if (det) {
          det->c[0] = det->c[0] / 2.0 + roi_rect.x;
          det->c[1] = det->c[1] / 2.0 + roi_rect.y;
          for (int k = 0; k < 4; ++k) {
            det->p[k][0] = det->p[k][0] / 2.0 + roi_rect.x;
            det->p[k][1] = det->p[k][1] / 2.0 + roi_rect.y;
          }
        }
      }
    } else {
      detections = tag_detector_->detect(gray_orig);
    }
  } else {
    // 【中低空段 <= 6.0m】：CLAHE 全图直方图均衡化送检 (保证4个大Tag与中心25个小Tag全部完整送检，杜绝裁切)
    cv::Mat gray;
    cv::cvtColor(current_img, gray, cv::COLOR_BGR2GRAY);
    if (enable_c2f_enhancement_ && clahe_) {
      clahe_->apply(gray, gray);
    }
    detections = tag_detector_->detect(gray);
  }

  if (!current_pattern_)
    return;

  // 调用策略类处理特定图案、绘制并返回标准化观测
  TargetObservation obs = current_pattern_->process(
      detections, tag_detector_.get(), camera_inner_matrix_, output.detected);

  if (!obs.is_valid)
    return;

  output.image_width = current_img.cols;
  output.image_height = current_img.rows;
  output.target_pixel_raw = obs.center_pixel;
  if (current_img.cols > 0 && current_img.rows > 0) {
    output.target_pixel_norm.x = obs.center_pixel.x / static_cast<double>(current_img.cols);
    output.target_pixel_norm.y = obs.center_pixel.y / static_cast<double>(current_img.rows);
  }

  double relative_yaw = 0.0;
  double abs_yaw = 0.0;

  // 3. 如果视觉有效，进行姿态歧义消除与坐标系转换
  TargetPose *best_pose = &obs.pose1; // 默认相信 pose1

  bool gimbal_abs = gimbal_abs_;
  std::optional<double> gimbal_roll, gimbal_pitch, gimbal_yaw;
  if (!pose_history_ ||
      !pose_history_->get_gimbal_at(stamp_tracker, gimbal_roll, gimbal_pitch,
                                    gimbal_yaw)) {
    gimbal_roll = gimbal_roll_;
    gimbal_pitch = gimbal_pitch_;
    gimbal_yaw = gimbal_yaw_;
  }

  // --- 歧义消除逻辑 ---
  double time_since_last_valid = stamp_tracker - last_ekf_stamp_;
  bool is_prior_reliable = has_last_valid_pos_ && (time_since_last_valid < 0.5);

  if (obs.pose2.valid && is_prior_reliable) {
    Eigen::Matrix3d R_bc_dynamic = get_dynamic_camera_to_body_rotation(
        hist_R_wb, gimbal_roll, gimbal_pitch, gimbal_yaw, gimbal_abs);

    auto get_yaw_from_pose = [&](const TargetPose &p) {
      Eigen::Matrix3d R_tag_world = hist_R_wb * (R_bc_dynamic * p.R);
      return std::atan2(R_tag_world(1, 0), R_tag_world(0, 0));
    };

    double yaw1 = get_yaw_from_pose(obs.pose1);
    double yaw2 = get_yaw_from_pose(obs.pose2);
    double predicted_yaw = kf_abs_yaw_->get_yaw();

    double diff1 =
        std::abs(KalmanFilterYaw::normalize_angle(yaw1 - predicted_yaw));
    double diff2 =
        std::abs(KalmanFilterYaw::normalize_angle(yaw2 - predicted_yaw));

    if (diff2 < diff1) {
      best_pose = &obs.pose2;
      ROS_INFO("[PlandDetector] Pose Ambiguity Resolved: Chose Pose 2 based on EKF "
               "prior.");
    }
  }

  // --- 纯 PnP 几何解算 (直接输出机身体系 FLU) ---
  PnpResultBody pnp_body = solve_pose_by_pnp(
      best_pose, hist_R_wb, gimbal_roll, gimbal_pitch, gimbal_yaw, gimbal_abs);

  if (std::isnan(pnp_body.pos_body.x()) || std::isnan(pnp_body.pos_body.y()) ||
      std::isnan(pnp_body.pos_body.z())) {
    return;
  }

  // 1. 控制器核心数据：纯净机体系 FLU 下的几何测量值 (零累积误差、零转换时延)
  output.target_pos_body_raw = pnp_body.pos_body;
  output.target_pos_body = pnp_body.pos_body;
  output.target_yaw_body = pnp_body.yaw_body;
  output.is_valid = true;

  // 2. 状态估计数据：按需将机体系投影至大地系 (ENU)，供 CTRV 滤波与全局可视化
  Eigen::Vector3d raw_target_enu = hist_drone_pos + hist_R_wb * pnp_body.pos_body;
  Eigen::Matrix3d R_tag_world = hist_R_wb * pnp_body.R_tag_body;
  abs_yaw = std::atan2(R_tag_world(1, 0), R_tag_world(0, 0));
  relative_yaw = pnp_body.yaw_body;

  // 4. EKF 滤波与运动状态估计
  double dt_ekf = 0.033;
  if (last_ekf_stamp_ > 0) {
    dt_ekf = (stamp_tracker - last_ekf_stamp_);
  }

  if (has_last_valid_pos_) {
    // 物理学常识限制：检查跳变是否合理
    double jump_dist =
        (raw_target_enu.head<2>() - last_valid_enu_pos_.head<2>()).norm();
    double max_physical_jump = 20.0 * dt_ekf + 1.0;

    if (jump_dist > max_physical_jump) {
      ROS_WARN("[PlandDetector] Outlier Rejected! Jump dist %.2fm is physically "
               "impossible.",
               jump_dist);
      output.is_valid = false;
      return;
    }
  }

  if (!has_last_valid_pos_ || dt_ekf <= 1e-4 || dt_ekf > 1.0) {
    reset();
    dt_ekf = 0.033;
    kf_xy_->force_set_state(raw_target_enu.x(), raw_target_enu.y());
    kf_yaw_->force_set_state(relative_yaw);
    kf_abs_yaw_->force_set_state(abs_yaw);
  }
  last_ekf_stamp_ = stamp_tracker;

  // 数据合法，更新记录并进入 EKF
  last_valid_enu_pos_ = raw_target_enu;
  has_last_valid_pos_ = true;

  TargetState target_state =
      target_tracker_->update(raw_target_enu.head<2>());

  double dist_xy = pnp_body.pos_body.head<2>().norm();
  double current_visual_angle_deg =
      std::atan2(dist_xy, current_z) * 180.0 / M_PI;

  // 执行 KF 滤波
  double current_angular_rate = vel_angular_body_.norm();
  double epsilon = 0;
  kf_xy_->update(epsilon, raw_target_enu.x(), raw_target_enu.y(), dt_ekf,
                 current_z, current_angular_rate, current_visual_angle_deg);
  kf_yaw_->update(relative_yaw, dt_ekf);
  kf_abs_yaw_->update(abs_yaw, dt_ekf);

  Eigen::Vector2d v_xy_enu = kf_xy_->get_vel();
  if (v_xy_enu.norm() < velocity_deadzone_) {
    v_xy_enu.setZero();
  }

  // 刷新 OSD 状态字段为本帧最新估计
  output.target_moving = (target_state == TargetState::MOVING);
  output.target_speed = v_xy_enu.norm();

  output.target_yaw_body = kf_yaw_->get_yaw();
  output.target_pos_enu.head<2>() = kf_xy_->get_pos();
  output.target_pos_enu.z() = raw_target_enu.z();
  output.target_yaw_enu = kf_abs_yaw_->get_yaw();

  double yaw_rate = kf_abs_yaw_->get_yaw_rate();

  if (target_state == TargetState::MOVING) {
    output.target_vel_enu << v_xy_enu.x(), v_xy_enu.y(), yaw_rate;
  } else {
    output.target_vel_enu.setZero();
    output.target_vel_enu.z() = yaw_rate;
  }

  // 发布目标位姿与速度话题 (ENU 坐标系)
  publish_target(output);
}

void PlandDetector::update_pose(double t, Eigen::Vector3d pos) {
  pos_enu_ = pos;
  if (pose_history_) {
    pose_history_->push_pos(t, pos);
  }
}

void PlandDetector::update_quat(double t, Eigen::Quaterniond quat) {
  orientation_ = quat;
  Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX
  yaw_enu_ = euler[0];
  pitch_ = euler[1];
  roll_ = euler[2];
  if (pose_history_) {
    pose_history_->push_quat(t, quat);
  }
}

void PlandDetector::update_gimbal(double t, std::optional<double> r,
                                  std::optional<double> p,
                                  std::optional<double> y) {
  if (r.has_value()) {
    gimbal_roll_ = r;
    if (pose_history_)
      pose_history_->push_gimbal_roll(t, r.value());
  }
  if (p.has_value()) {
    gimbal_pitch_ = p;
    if (pose_history_)
      pose_history_->push_gimbal_pitch(t, p.value());
  }
  if (y.has_value()) {
    gimbal_yaw_ = y;
    if (pose_history_)
      pose_history_->push_gimbal_yaw(t, y.value());
  }
}

void PlandDetector::update_image(double t, const cv::Mat &img) {
  image_stamp_ = t;
  pland_image_ = img;
}

void PlandDetector::update_angular_rate(
    const Eigen::Vector3d &vel_angular_body) {
  vel_angular_body_ = vel_angular_body;
}

void PlandDetector::publish_target(const DetectorResult &result) {
  if (!result.is_valid)
    return;

  ros::Time stamp =
      (result.stamp > 0) ? ros::Time(result.stamp) : ros::Time::now();

  // 1. 控制核心话题：发布目标在机体系 (base_link / FLU) 下的相对位姿，供 pland_controller 零误差纯视觉闭环
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = stamp;
  pose_msg.header.frame_id = "base_link";
  pose_msg.pose.position.x = result.target_pos_body.x();
  pose_msg.pose.position.y = result.target_pos_body.y();
  pose_msg.pose.position.z = result.target_pos_body.z();

  Eigen::Quaterniond q(
      Eigen::AngleAxisd(result.target_yaw_body, Eigen::Vector3d::UnitZ()));
  pose_msg.pose.orientation.w = q.w();
  pose_msg.pose.orientation.x = q.x();
  pose_msg.pose.orientation.y = q.y();
  pose_msg.pose.orientation.z = q.z();
  target_pose_pub_.publish(pose_msg);

  // 2. 调试用话题：发布目标在 map (ENU) 坐标系下的绝对位姿，供 RViz / 地面站全局可视化调试展示
  if (debug_target_pose_pub_.getNumSubscribers() > 0) {
    geometry_msgs::PoseStamped pose_enu_msg;
    pose_enu_msg.header.stamp = stamp;
    pose_enu_msg.header.frame_id = "map";
    pose_enu_msg.pose.position.x = result.target_pos_enu.x();
    pose_enu_msg.pose.position.y = result.target_pos_enu.y();
    pose_enu_msg.pose.position.z = result.target_pos_enu.z();

    Eigen::Quaterniond q_enu(
        Eigen::AngleAxisd(result.target_yaw_enu, Eigen::Vector3d::UnitZ()));
    pose_enu_msg.pose.orientation.w = q_enu.w();
    pose_enu_msg.pose.orientation.x = q_enu.x();
    pose_enu_msg.pose.orientation.y = q_enu.y();
    pose_enu_msg.pose.orientation.z = q_enu.z();
    debug_target_pose_pub_.publish(pose_enu_msg);
  }

  // 3. 前馈速度话题：发布目标在 map (ENU) 坐标系下的前馈速度
  geometry_msgs::TwistStamped vel_msg;
  vel_msg.header.stamp = stamp;
  vel_msg.header.frame_id = "map";
  vel_msg.twist.linear.x = result.target_vel_enu.x();
  vel_msg.twist.linear.y = result.target_vel_enu.y();
  vel_msg.twist.linear.z = 0.0;
  vel_msg.twist.angular.z = result.target_vel_enu.z(); // yaw_rate
  target_vel_pub_.publish(vel_msg);

  // 4. 原始图像归一化坐标话题：发布目标在图像平面上的归一化像素坐标 (u/width, v/height)，范围 [0.0, 1.0]
  if (target_pixel_pub_.getNumSubscribers() > 0) {
    geometry_msgs::PointStamped pixel_msg;
    pixel_msg.header.stamp = stamp;
    pixel_msg.header.frame_id = "camera_optical_frame";
    pixel_msg.point.x = result.target_pixel_norm.x;
    pixel_msg.point.y = result.target_pixel_norm.y;
    pixel_msg.point.z = 0.0;
    target_pixel_pub_.publish(pixel_msg);
  }
}