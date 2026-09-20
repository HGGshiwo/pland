#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

/**
 * @brief 机体系速度平滑器 (严格参考 dankong/include/features/tracker/tracker.hpp apply_kinematic_constraints 实现)
 * 1. 坐标系旋转处理：当机体自转时，将上一拍机体系的速度与加速度重新投影到当前机体系
 * 2. 考虑减速特化的加速度限制与加加速度 (Jerk) 限制
 * 3. 垂直与偏航角速度速率限制
 */
class VelocitySmoother {
public:
  VelocitySmoother() = default;

  void reset(const Eigen::Vector4d &initial_vel = Eigen::Vector4d::Zero(),
             double initial_yaw = 0.0) {
    last_cmd_vel_ = initial_vel;
    last_cmd_acc_ = Eigen::Vector3d::Zero();
    last_yaw_ = initial_yaw;
    initialized_ = true;
  }

  bool is_initialized() const { return initialized_; }

  /**
   * @brief 施加运动学约束和平滑
   * @param raw_cmd 原始机体系速度指令 [vx, vy, vz, yaw_rate]
   * @param current_yaw 当前无人机机体偏航角 (rad)
   * @param dt 控制周期 (s)
   * @param limit_v_xy 水平最大速度 (m/s)
   * @param limit_v_z 垂直最大速度 (m/s)
   * @param limit_v_yaw 最大偏航角速度 (rad/s)
   * @param max_acc_xy 水平最大加速度 (m/s^2)
   * @param max_decel_xy 水平最大减速度 (m/s^2)
   * @param max_jerk_xy 水平最大加加速度 (m/s^3)
   * @param max_acc_z 垂直最大加速度 (m/s^2)
   * @param max_jerk_z 垂直最大加加速度 (m/s^3)
   * @param max_acc_yaw 偏航最大角加速度 (rad/s^2)
   * @return 经过机体坐标系重投影与加速度/Jerk平滑后的速度指令
   */
  Eigen::Vector4d apply_constraints(Eigen::Vector4d raw_cmd,
                                    double current_yaw,
                                    double dt,
                                    double limit_v_xy,
                                    double limit_v_z,
                                    double limit_v_yaw,
                                    double max_acc_xy,
                                    double max_decel_xy,
                                    double max_jerk_xy = 1000.0,
                                    double max_acc_z = 2.0,
                                    double max_jerk_z = 1000.0,
                                    double max_acc_yaw = 3.0) {
    if (!initialized_) {
      reset(raw_cmd, current_yaw);
      return raw_cmd;
    }
    if (dt < 1e-4) {
      return last_cmd_vel_;
    }

    // 1. 速度硬约束 (限速)
    double speed_xy = std::hypot(raw_cmd.x(), raw_cmd.y());
    if (speed_xy > limit_v_xy && speed_xy > 1e-4) {
      raw_cmd.x() = (raw_cmd.x() / speed_xy) * limit_v_xy;
      raw_cmd.y() = (raw_cmd.y() / speed_xy) * limit_v_xy;
    }
    raw_cmd.z() = std::clamp(raw_cmd.z(), -limit_v_z, limit_v_z);
    raw_cmd.w() = std::clamp(raw_cmd.w(), -limit_v_yaw, limit_v_yaw);

    // 2. 坐标系旋转处理：当机体发生偏航时，上一拍的速度和加速度必须旋转到新机体系下
    double delta_yaw = current_yaw - last_yaw_;
    double cy = std::cos(delta_yaw);
    double sy = std::sin(delta_yaw);

    double rotated_last_vx =  last_cmd_vel_.x() * cy + last_cmd_vel_.y() * sy;
    double rotated_last_vy = -last_cmd_vel_.x() * sy + last_cmd_vel_.y() * cy;

    double rotated_last_ax =  last_cmd_acc_.x() * cy + last_cmd_acc_.y() * sy;
    double rotated_last_ay = -last_cmd_acc_.x() * sy + last_cmd_acc_.y() * cy;

    last_cmd_vel_.x() = rotated_last_vx;
    last_cmd_vel_.y() = rotated_last_vy;
    last_cmd_acc_.x() = rotated_last_ax;
    last_cmd_acc_.y() = rotated_last_ay;
    last_yaw_ = current_yaw;

    // ---------------------------------------------------------------------
    // 3. XY轴的加速度与加加速度(Jerk)限制
    // ---------------------------------------------------------------------
    double desired_ax = (raw_cmd.x() - last_cmd_vel_.x()) / dt;
    double desired_ay = (raw_cmd.y() - last_cmd_vel_.y()) / dt;

    // 3.1 限制加速度大小 (判断是否为减速状态：速度与期望加速度点乘小于0且当前速度较大)
    double limit_accel = max_acc_xy;
    if (last_cmd_vel_.head<2>().norm() > 0.01 &&
        (last_cmd_vel_.x() * desired_ax + last_cmd_vel_.y() * desired_ay) < 0.0) {
      limit_accel = max_decel_xy;
    }

    double desired_a_mag = std::hypot(desired_ax, desired_ay);
    if (desired_a_mag > limit_accel && desired_a_mag > 1e-6) {
      desired_ax = (desired_ax / desired_a_mag) * limit_accel;
      desired_ay = (desired_ay / desired_a_mag) * limit_accel;
    }

    // 3.2 限制加加速度大小 (Jerk Limit)
    double dax = desired_ax - last_cmd_acc_.x();
    double day = desired_ay - last_cmd_acc_.y();
    double da_mag = std::hypot(dax, day);
    double max_da = max_jerk_xy * dt;

    if (da_mag > max_da && da_mag > 1e-6) {
      desired_ax = last_cmd_acc_.x() + (dax / da_mag) * max_da;
      desired_ay = last_cmd_acc_.y() + (day / da_mag) * max_da;
    }

    last_cmd_acc_.x() = desired_ax;
    last_cmd_acc_.y() = desired_ay;
    raw_cmd.x() = last_cmd_vel_.x() + desired_ax * dt;
    raw_cmd.y() = last_cmd_vel_.y() + desired_ay * dt;

    // ---------------------------------------------------------------------
    // 4. Z轴的加速度与加加速度(Jerk)限制
    // ---------------------------------------------------------------------
    double desired_az = (raw_cmd.z() - last_cmd_vel_.z()) / dt;
    desired_az = std::clamp(desired_az, -max_acc_z, max_acc_z);

    double max_daz = max_jerk_z * dt;
    desired_az = std::clamp(desired_az, last_cmd_acc_.z() - max_daz,
                            last_cmd_acc_.z() + max_daz);

    last_cmd_acc_.z() = desired_az;
    raw_cmd.z() = last_cmd_vel_.z() + desired_az * dt;

    // ---------------------------------------------------------------------
    // 5. Yaw轴 角加速度限制
    // ---------------------------------------------------------------------
    double max_delta_w = max_acc_yaw * dt;
    raw_cmd.w() = std::clamp(raw_cmd.w(), last_cmd_vel_.w() - max_delta_w,
                             last_cmd_vel_.w() + max_delta_w);

    last_cmd_vel_ = raw_cmd;
    return raw_cmd;
  }

  const Eigen::Vector4d &get_last_cmd_vel() const { return last_cmd_vel_; }

private:
  Eigen::Vector4d last_cmd_vel_ = Eigen::Vector4d::Zero();
  Eigen::Vector3d last_cmd_acc_ = Eigen::Vector3d::Zero();
  double last_yaw_ = 0.0;
  bool initialized_ = false;
};
