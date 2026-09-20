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
#include <ros/ros.h>
#include <ros_param_sync/param_sync.hpp>

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
  cv::Mat detected;             // 标注好结果的图片

  // 图像像素坐标与长宽
  cv::Point2d target_pixel_raw = cv::Point2d(0, 0);     // 原始图像像素坐标 (u, v)
  cv::Point2d target_pixel_norm = cv::Point2d(0, 0);    // 归一化像素坐标 (u/width, v/height)，范围 [0.0, 1.0]
  int image_width = 0;
  int image_height = 0;
};

struct PnpResultBody {
  Eigen::Vector3d pos_body = Eigen::Vector3d::Zero();
  double yaw_body = 0.0;
  Eigen::Matrix3d R_tag_body = Eigen::Matrix3d::Identity();
};

class PlandDetector {
private:
  // --- 硬件/静态配置参数 (从 drone_config.yaml 加载) ---
  double offset_x_ = 0.0;   // 相机相对于机身的x轴偏移(FLU)
  double offset_y_ = 0.0;   // 相机相对于机身的y轴偏移(FLU)
  double offset_z_ = 0.0;   // 相机相对于机身的z轴偏移(FLU)

  // --- 动态可调参数 (支持运行时修改并持久化) ---
  double velocity_deadzone_ = 0.0; // 移动物体速度死区阈值 (m/s)
  bool gimbal_abs_ = false; // 云台固定角模式(垂直地面)

  // --- 静态配置参数 (在 launch / 构造函数中指定) ---
  Eigen::Matrix3d camera_inner_matrix_ = Eigen::Matrix3d::Identity();
  std::string tag_family_ = "tag16h5";
  std::string tag_config_file_path_;
  std::string target_pose_topic_ = "/pland/target_pose";
  std::string target_vel_topic_ = "/pland/target_vel";
  std::string debug_target_pose_topic_ = "/pland/target_pose_enu";
  std::string target_pixel_topic_ = "/pland/target_pixel";

  ros::NodeHandle nh_;
  ros::Publisher target_pose_pub_;
  ros::Publisher target_vel_pub_;
  ros::Publisher debug_target_pose_pub_;
  ros::Publisher target_pixel_pub_;

  // --- 状态成员变量 ---
  Eigen::Vector3d pos_enu_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
  cv::Mat pland_image_;
  double image_stamp_ = 0.0;
  std::optional<double> gimbal_roll_;
  std::optional<double> gimbal_pitch_;
  std::optional<double> gimbal_yaw_;
  Eigen::Vector3d vel_angular_body_ = Eigen::Vector3d::Zero();
  double roll_ = 0.0;
  double pitch_ = 0.0;
  double yaw_enu_ = 0.0;

  std::unique_ptr<SafeAprilTagDetector> tag_detector_;
  std::unique_ptr<ITargetPattern> current_pattern_;

  std::shared_ptr<KalmanFilterCTRV> kf_xy_;
  std::shared_ptr<KalmanFilterYaw> kf_yaw_;
  std::shared_ptr<KalmanFilterYaw> kf_abs_yaw_;
  std::shared_ptr<TargetTracker> target_tracker_;
  std::shared_ptr<PoseHistory> pose_history_;

  double last_ekf_stamp_ = 0.0;

  Eigen::Vector3d last_valid_enu_pos_ = Eigen::Vector3d::Zero(); // 记录上一次有效位置
  bool has_last_valid_pos_ = false;

private:
  Eigen::Matrix3d get_cam_to_frd_matrix() const {
    Eigen::Matrix3d R;
    R << 0, 0, 1, 1, 0, 0, 0, 1, 0;
    return R;
  }

  double get_current_z() const {
    return std::abs(pos_enu_.z());
  }

  // 动态获取相机光学系(C) 到 机体系(B) 的旋转矩阵
  Eigen::Matrix3d get_dynamic_camera_to_body_rotation(
      const Eigen::Matrix3d &hist_R_wb, std::optional<double> gimbal_roll,
      std::optional<double> gimbal_pitch, std::optional<double> gimbal_yaw,
      bool is_gimbal_absolute) const;


  // 纯净 PnP 解算：仅输出机体系 (Body FLU) 下的相对位置与姿态
  PnpResultBody solve_pose_by_pnp(
      const TargetPose *safe_pose, const Eigen::Matrix3d &hist_R_wb,
      std::optional<double> gimbal_roll, std::optional<double> gimbal_pitch,
      std::optional<double> gimbal_yaw, bool is_gimbal_absolute);

  void publish_target(const DetectorResult &result);

public:
  PlandDetector();
  PlandDetector(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  void init(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  bool load_tag_config(const std::string &path);
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
   * @brief 设置静态相机外参偏移 (FLU 机体坐标系, 米)
   */
  void set_camera_offset(double x, double y, double z) {
    offset_x_ = x;
    offset_y_ = y;
    offset_z_ = z;
  }
  double get_offset_x() const { return offset_x_; }
  double get_offset_y() const { return offset_y_; }
  double get_offset_z() const { return offset_z_; }
};
