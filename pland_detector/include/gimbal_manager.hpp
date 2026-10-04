#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <optional>
#include <string>

// 云台回读角度的参考系
enum class GimbalAngleFrame {
  kBody = 0,   // 相对机体 (固定挂载/模拟云台)
  kGround = 1, // 相对大地/水平面的绝对角 (DJI 等自稳云台)
};

struct GimbalAxisParams {
  std::string topic; // 回读数据来源话题; 空 = 该轴无回读 (使用固定默认角)
  GimbalAngleFrame frame = GimbalAngleFrame::kBody;
};

struct GimbalParams {
  GimbalAxisParams roll;
  GimbalAxisParams pitch;
  GimbalAxisParams yaw;
  // 相机光心相对 base_link 原点的位移 (机体系 FLU, 米, 云台中位姿态下量取)
  Eigen::Vector3d camera_offset = Eigen::Vector3d::Zero();
};

/**
 * @brief 云台/相机传感头外参管理器 (无状态纯几何计算)
 *
 * 职责: 把 "图像时刻的云台回读角 + 同时刻机体姿态" 合成为
 *       相机光学系(C) -> 机体系(base_link, FLU) 的完整外参 (R_bc, t_bc)。
 * 时间缓存与插值由外部 PoseHistory 完成, 本类不做任何时间对齐。
 *
 * 回读约定: 输入角度为弧度; 某轴参考系由 GimbalAxisParams::frame 描述;
 *          某轴从未收到回读(空 optional)时使用固定默认角 ——
 *          视作垂直向下 90° 的固定云台 (roll 0 / pitch -90° / yaw 0, 相对机体)。
 * 绝对角换算: frame == kGround 的轴按 "云台角 与 同轴机体欧拉角相加/相减"
 *             逐轴补偿 (降落场景机体近水平, 欧拉角逐轴相减的耦合误差可忽略):
 *             pitch: motor = ground + 机体低头角   (低头为正)
 *             roll:  motor = ground - 机体右倾角   (右倾为正)
 *             yaw:   motor = 机体航向 - ground     (电机绕 FRD z 朝下, 方向相反)
 */
class GimbalManager {
public:
  explicit GimbalManager(const GimbalParams &params) : params_(params) {}

  /**
   * @brief 计算图像时刻的相机->机体外参
   * @param R_wb 图像时刻机体姿态 (ENU 世界系 -> FLU 机体系)
   * @param gimbal_roll/pitch/yaw 该时刻的云台回读角 (弧度, 参考系见 params),
   *                              可为空 (无回读, 用固定默认角)
   * @param R_bc 相机光学系 -> 机体系旋转
   * @param t_bc 相机光心在机体系中的位置
   */
  void get_cam_extrinsic(Eigen::Matrix3d &R_bc, Eigen::Vector3d &t_bc,
                         const Eigen::Matrix3d &R_wb,
                         std::optional<double> gimbal_roll,
                         std::optional<double> gimbal_pitch,
                         std::optional<double> gimbal_yaw) const {
    const double roll =
        resolve_axis(Axis::kRoll, gimbal_roll, kDefaultRoll, R_wb);
    const double pitch =
        resolve_axis(Axis::kPitch, gimbal_pitch, kDefaultPitch, R_wb);
    const double yaw = resolve_axis(Axis::kYaw, gimbal_yaw, kDefaultYaw, R_wb);

    // 云台电机旋转矩阵 (FRD 坐标系下 Z-Y-X 旋转)
    const Eigen::Matrix3d R_motor =
        (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();

    // FRD -> FLU 的静态转换
    Eigen::Matrix3d R_frd_to_flu;
    R_frd_to_flu << 1, 0, 0, 0, -1, 0, 0, 0, -1;

    // 相机光学系(C) -> FRD系 -> FLU机身系
    R_bc = R_frd_to_flu * R_motor * get_cam_to_frd_matrix();
    t_bc = params_.camera_offset;
  }

  const GimbalParams &params() const { return params_; }

private:
  enum class Axis { kRoll, kPitch, kYaw };

  // 无回读兜底: 固定云台, 垂直向下 90° (机体系 motor 角)
  static constexpr double kDefaultRoll = 0.0;
  static constexpr double kDefaultPitch = -M_PI_2;
  static constexpr double kDefaultYaw = 0.0;

  // 相机光学系 (z 朝前, x 朝右, y 朝下) -> FRD (x 前, y 右, z 下)
  static Eigen::Matrix3d get_cam_to_frd_matrix() {
    Eigen::Matrix3d R;
    R << 0, 0, 1, 1, 0, 0, 0, 1, 0;
    return R;
  }

  const GimbalAxisParams &axis_params(const Axis &a) const {
    return a == Axis::kRoll    ? params_.roll
           : a == Axis::kPitch ? params_.pitch
                               : params_.yaw;
  }

  // 单轴角度解析: 空 -> 固定默认角; body -> 直用; ground -> 减同轴机体欧拉角
  double resolve_axis(const Axis &a, const std::optional<double> &angle,
                      double default_rad, const Eigen::Matrix3d &R_wb) const {
    if (!angle.has_value()) {
      return default_rad;
    }
    if (axis_params(a).frame == GimbalAngleFrame::kBody) {
      return angle.value();
    }
    return ground_to_body(a, angle.value(), R_wb);
  }

  // 绝对角 -> 机体系 motor 角 (R_wb 为同时刻机体姿态)
  double ground_to_body(const Axis &a, double ground_rad,
                        const Eigen::Matrix3d &R_wb) const {
    switch (a) {
    case Axis::kPitch:
      // 机头低头角 (低头为正), 与绝对下视角同号约定 (-90° = 垂直向下)
      return ground_rad + body_pitch_down(R_wb);
    case Axis::kRoll:
      // 机体右倾角 (右倾为正)
      return ground_rad - std::atan2(R_wb(2, 1), R_wb(2, 2));
    case Axis::kYaw:
      // 电机 Rz 绕 FRD z(朝下), 正转 = 航向减小, 故相机世界航向 = ψ - m;
      // 令其等于绝对回读 G: m = ψ - G (与 pitch/roll 的加性方向相反)
      return std::atan2(R_wb(1, 0), R_wb(0, 0)) - ground_rad;
    }
    return ground_rad;
  }

  // 机体前向轴 (FLU x) 在 ENU 中的低头角, 低头为正
  static double body_pitch_down(const Eigen::Matrix3d &R_wb) {
    const Eigen::Vector3d forward_enu = R_wb.col(0);
    return std::atan2(-forward_enu.z(),
                      std::sqrt(forward_enu.x() * forward_enu.x() +
                                forward_enu.y() * forward_enu.y()));
  }

  GimbalParams params_;
};
