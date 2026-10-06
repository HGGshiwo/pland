#pragma once
#include <Eigen/Dense>
#include <chrono>
#include <fstream>
#include <limits>
#include <memory>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Int32.h>
#include <ros/ros.h>
#include <ros_param_sync/param_sync.hpp>

#include "./gimbal_manager.hpp"
#include "./kalman_filter_ctrv.hpp"
#include "./kalman_filter_yaw.hpp"
#include "./pose_history.hpp"
#include "./tag_detector/multiarray_tags_pattern.hpp"
#include "./tag_detector/safe_aprialtag.hpp"
#include "./target_tracker.hpp"
#include "Eigen/src/Geometry/Quaternion.h"

struct DetectorResult {
  bool is_valid = false;
  double stamp; // 图像拍摄的真实时间

  // 目标位置，原始观测值
  Eigen::Vector3d target_pos_body_raw = Eigen::Vector3d::Zero();

  // ENU 导航系下的绝对目标位置和速度 (EKF 融合后)
  Eigen::Vector3d target_pos_enu = Eigen::Vector3d::Zero();
  Eigen::Vector3d target_vel_enu = Eigen::Vector3d::Zero(); // z is yaw_rate
  double target_yaw_enu = 0.0;

  Eigen::Vector3d target_pos_body = Eigen::Vector3d::Zero();
  double target_yaw_body = 0.0; // yaw_relative

  // 机体误差矢量仅旋转到 ENU 方向轴 (R_wb·pos_body, 不加平移)
  // 自由矢量, 原点无意义; 控制端直接作为 ENU 速度合成的输入
  Eigen::Vector3d target_err_enu = Eigen::Vector3d::Zero();
  cv::Mat detected;             // 标注好结果的图片

  // 图像像素坐标与长宽
  cv::Point2d target_pixel_raw = cv::Point2d(0, 0);     // 原始图像像素坐标 (u, v)
  cv::Point2d target_pixel_norm = cv::Point2d(0, 0);    // 归一化像素坐标 (u/width, v/height)，范围 [0.0, 1.0]
  int image_width = 0;
  int image_height = 0;

  // --- OSD 可视化附加状态 (即使本帧检测失败，也携带最近一次估计值) ---
  double drone_z = 0.0;        // 无人机当前飞行高度 (m, ENU)
  bool target_moving = false;  // 目标是否处于运动状态 (TargetTracker 判定)
  double target_speed = 0.0;   // 目标水平合速度 (m/s, CTRV EKF 估计, 含死区)
};

struct PnpResultBody {
  Eigen::Vector3d pos_body = Eigen::Vector3d::Zero();
  double yaw_body = 0.0;
  Eigen::Matrix3d R_tag_body = Eigen::Matrix3d::Identity();
};

class PlandDetector {
private:
  // --- 动态可调参数 (支持运行时修改并持久化) ---
  double velocity_deadzone_ = 0.0; // 移动物体速度死区阈值 (m/s)
  bool enable_c2f_enhancement_ = true; // 是否开启高空粗检ROI与低空自适应图像增强
  bool disable_all_enhancement_ = false; // 总开关: 置 true 时关闭全部增强, 灰度图直通检测 (评测基线用)
  std::string enhance_mode_ = "adaptive"; // 增强模式: none(直通) / sharpen(全图锐化) / roi(全高度ROI超分) / adaptive(高度自适应C2F)
  cv::Ptr<cv::CLAHE> clahe_;

  // --- 静态配置参数 (在 launch / 构造函数中指定) ---
  Eigen::Matrix3d camera_inner_matrix_ = Eigen::Matrix3d::Identity();
  std::string tag_family_ = "tag16h5";
  std::string tag_config_file_path_;
  std::string target_pose_topic_ = "/pland/target_pose";
  std::string target_err_enu_topic_ = "/pland/target_err_enu";
  std::string target_vel_topic_ = "/pland/target_vel";
  std::string debug_target_pose_topic_ = "/pland/target_pose_enu";
  std::string target_pixel_topic_ = "/pland/target_pixel";

  ros::NodeHandle nh_;
  ros::Publisher target_pose_pub_;
  ros::Publisher target_err_enu_pub_;
  ros::Publisher target_vel_pub_;
  ros::Publisher debug_target_pose_pub_;
  ros::Publisher target_pixel_pub_;
  // 本帧原始检测到的 tag 数量 (供自动化评测脚本统计)
  ros::Publisher tag_count_pub_;

  // --- 状态成员变量 ---
  Eigen::Vector3d pos_enu_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
  cv::Mat pland_image_;
  double image_stamp_ = 0.0;
  Eigen::Vector3d vel_angular_body_ = Eigen::Vector3d::Zero();
  double roll_ = 0.0;
  double pitch_ = 0.0;
  double yaw_enu_ = 0.0;

  // --- 融合高度状态 (与 pland_controller 的 get_current_z 逻辑一致) ---
  bool has_rangefinder_ = false;
  double rangefinder_alt_ = -1.0;
  double rangefinder_stamp_ = 0.0;
  bool has_rel_alt_ = false;
  double rel_alt_ = 0.0;
  double rel_alt_stamp_ = 0.0;
  double frame_height_ = 0.0;  // 机架高度: 起落架底端到测距仪安装面的物理距离

  std::unique_ptr<SafeAprilTagDetector> tag_detector_;
  std::unique_ptr<ITargetPattern> current_pattern_;

  std::shared_ptr<KalmanFilterCTRV> kf_xy_;
  std::shared_ptr<KalmanFilterYaw> kf_yaw_;
  std::shared_ptr<KalmanFilterYaw> kf_abs_yaw_;
  std::shared_ptr<TargetTracker> target_tracker_;
  std::shared_ptr<PoseHistory> pose_history_;
  std::unique_ptr<GimbalManager> gimbal_manager_;

  double last_ekf_stamp_ = 0.0;

  Eigen::Vector3d last_valid_enu_pos_ = Eigen::Vector3d::Zero(); // 记录上一次有效位置
  bool has_last_valid_pos_ = false;

private:
  // 融合高度: 低空 (<1m) 以测距仪 (含机架高度补偿) 为准, 1~3m 线性过渡, 高空用
  // rel_alt/ENU 里程计; 与 pland_controller::get_current_z 保持一致
  double get_current_z() const {
    double now = ros::Time::now().toSec();
    double base_z = std::abs(pos_enu_.z());
    if (has_rel_alt_ && (now - rel_alt_stamp_ < 1.0)) {
      base_z = std::abs(rel_alt_);
    }

    bool rf_valid = has_rangefinder_ && (rangefinder_alt_ > 0.0) &&
                    (now - rangefinder_stamp_ < 0.5);
    if (!rf_valid) {
      return base_z;
    }

    const double min_alt = 1.0;
    const double max_alt = 3.0;
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

  // --- 图像粗检定位 ---
  cv::Rect find_texture_roi(const cv::Mat &gray) const;

  // 纯净 PnP 解算：仅输出机体系 (Body FLU) 下的相对位置与姿态
  PnpResultBody solve_pose_by_pnp(const TargetPose *safe_pose,
                                  const Eigen::Matrix3d &R_bc,
                                  const Eigen::Vector3d &t_bc);

  void publish_target(const DetectorResult &result);

public:
  PlandDetector();
  PlandDetector(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  void init(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  bool load_tag_config(const std::string &path);

  // 融合高度数据源输入
  void update_rangefinder(double stamp, double range, bool is_valid) {
    if (!is_valid) {
      has_rangefinder_ = false;
      return;
    }
    has_rangefinder_ = true;
    rangefinder_alt_ = range;
    rangefinder_stamp_ = stamp;
  }

  void update_rel_alt(double stamp, double rel_alt) {
    has_rel_alt_ = true;
    rel_alt_ = rel_alt;
    rel_alt_stamp_ = stamp;
  }

  void set_frame_height(double frame_height) { frame_height_ = frame_height; }
  void reset();

  void detect(DetectorResult &output);
  void update_pose(double t, Eigen::Vector3d pos_enu);
  void update_quat(double t, Eigen::Quaterniond quat);
  void update_gimbal(double t, std::optional<double> r,
                      std::optional<double> p, std::optional<double> y);
  void update_image(double t, const cv::Mat &img);
  void update_angular_rate(const Eigen::Vector3d &vel_angular_body);

  /**
   * @brief 绑定支持运行时动态修改并回写文件的参数
   */
  void bind_dynamic_params(ros_param_sync::ParamSync &sync);

  /**
   * @brief 注入云台/相机传感头外参管理器
   * (默认构造的检测器自带 "固定云台垂直向下90°" 的默认管理器)
   */
  void set_gimbal_manager(std::unique_ptr<GimbalManager> manager) {
    gimbal_manager_ = std::move(manager);
  }
};
