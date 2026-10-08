#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

class KalmanFilterCTRV {
   public:
    // 基础观测噪声方差 (恢复至早期稳定版本 0.3 m^2)
    double base_r_noise_ = 0.3;
    // 航向角观测噪声方差 (rad^2, 约 5 度标准差，高置信度)
    double base_r_yaw_noise_ = 0.02;
    // 卡方检验正常机动阈值 (自由度 3, 95% 置信度)
    double chi2_nominal_thresh_ = 7.81;
    // 卡方检验野值抑制阈值 (自由度 3, 99.9% 置信度, 超过此值不放大Q且膨胀R)
    double chi2_outlier_thresh_ = 16.0;
    // AKF 过程噪声 Q 自适应放大上限 (收敛至温和区间，防过度微分)
    double max_q_scale_ = 3.0;
    // 目标名义物理加速度与角加速度限制 (用于生成名义 Q 矩阵，压平常规静止与匀速工况)
    double nominal_max_acc_ = 0.08;
    double nominal_max_yaw_acc_ = 0.5;
    // 状态更新后速度单步最大允许加速度 (用于防止单帧突变: max_dv = a * dt)
    double max_dv_acc_ = 0.8;
    // 目标物理最大极限线速度硬限幅 (杜绝瞬态数百 m/s 爆表)
    double max_speed_ = 3.0;

    int update_count_ = 0;
    double last_v_ = 0.0;

    KalmanFilterCTRV() {
        // 状态向量 x: [pos_x, pos_y, v, yaw, omega]^T
        x_.setZero();
        P_.setIdentity();
        P_ *= 10.0;

        // 观测矩阵 H: 测量位置与航向 [pos_x, pos_y, yaw]
        H_.setZero();
        H_(0, 0) = 1.0;
        H_(1, 1) = 1.0;
        H_(2, 3) = 1.0;
    }

    void reset() {
        x_.setZero();
        P_.setIdentity();
        P_ *= 10.0;
        update_count_ = 0;
        last_v_ = 0.0;
    }

    void set_params(double chi2_nominal, double chi2_outlier, double max_q_scale,
                    double nominal_max_acc, double nominal_max_yaw_acc,
                    double max_dv_acc, double max_speed) {
        chi2_nominal_thresh_ = chi2_nominal;
        chi2_outlier_thresh_ = chi2_outlier;
        max_q_scale_ = max_q_scale;
        nominal_max_acc_ = nominal_max_acc;
        nominal_max_yaw_acc_ = nominal_max_yaw_acc;
        max_dv_acc_ = max_dv_acc;
        max_speed_ = max_speed;
    }

    // 辅助函数：将角度归一化到 [-pi, pi]
    static double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void force_set_state(double x, double y, double yaw = 0.0) {
        x_ << x, y, 0.0, normalize_angle(yaw), 0.0;
        P_.setIdentity();
        P_ *= 0.1;
        last_v_ = 0.0;
    }

    // 融合绝对航向角 (meas_yaw)，引入卡方检验门控与动力学一致性限幅
    Eigen::Vector2d update(double& epsilon, double meas_x, double meas_y,
                           double meas_yaw, double dt, double current_z,
                           double angular_rate, double visual_angle_deg,
                           double max_acc = -1.0, double max_yaw_acc = -1.0) {
        update_count_++;
        if (dt <= 1e-4) return get_vel();

        // 提取当前状态
        double px = x_(0);
        double py = x_(1);
        double v = x_(2);
        double yaw = x_(3);
        double omega = x_(4);

        // --- 1. 非线性状态预测 (Predict State) ---
        Vector5d x_pred = x_;

        if (std::abs(omega) > 1e-4) {
            x_pred(0) =
                px + (v / omega) * (std::sin(yaw + omega * dt) - std::sin(yaw));
            x_pred(1) = py + (v / omega) *
                                 (-std::cos(yaw + omega * dt) + std::cos(yaw));
        } else {
            x_pred(0) = px + v * dt * std::cos(yaw);
            x_pred(1) = py + v * dt * std::sin(yaw);
        }
        x_pred(3) = normalize_angle(yaw + omega * dt);

        // --- 2. 计算雅可比矩阵 F_j ---
        Matrix5d F_j;
        F_j.setIdentity();

        if (std::abs(omega) > 1e-4) {
            F_j(0, 2) = (std::sin(yaw + omega * dt) - std::sin(yaw)) / omega;
            F_j(0, 3) =
                (v / omega) * (std::cos(yaw + omega * dt) - std::cos(yaw));
            F_j(0, 4) = (v / (omega * omega)) *
                            (std::sin(yaw) - std::sin(yaw + omega * dt)) +
                        (v * dt / omega) * std::cos(yaw + omega * dt);

            F_j(1, 2) = (-std::cos(yaw + omega * dt) + std::cos(yaw)) / omega;
            F_j(1, 3) =
                (v / omega) * (std::sin(yaw + omega * dt) - std::sin(yaw));
            F_j(1, 4) = (v / (omega * omega)) *
                            (std::cos(yaw + omega * dt) - std::cos(yaw)) +
                        (v * dt / omega) * std::sin(yaw + omega * dt);
        } else {
            F_j(0, 2) = std::cos(yaw) * dt;
            F_j(0, 3) = -v * std::sin(yaw) * dt;
            F_j(0, 4) = -0.5 * v * std::sin(yaw) * dt * dt;

            F_j(1, 2) = std::sin(yaw) * dt;
            F_j(1, 3) = v * std::cos(yaw) * dt;
            F_j(1, 4) = 0.5 * v * std::cos(yaw) * dt * dt;
        }
        F_j(3, 4) = dt;

        // --- 3. 动态生成过程噪声 Q (基于标准连续时间向离散时间积分公式: Bar-Shalom 规范) ---
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double eff_max_acc = (max_acc > 0.0) ? max_acc : nominal_max_acc_;
        double eff_max_yaw_acc =
            (max_yaw_acc > 0.0) ? max_yaw_acc : nominal_max_yaw_acc_;
        double var_a = eff_max_acc * eff_max_acc;
        double var_yaw_a = eff_max_yaw_acc * eff_max_yaw_acc;

        Matrix5d Q;
        Q.setZero();
        // 水平位置与线速度子块 [px, py, v] (积分得 dt^3/3, dt^2/2, dt)
        Q(0, 0) = dt3 / 3.0 * var_a;
        Q(0, 2) = dt2 / 2.0 * var_a;
        Q(1, 1) = dt3 / 3.0 * var_a;
        Q(1, 2) = dt2 / 2.0 * var_a;
        Q(2, 0) = dt2 / 2.0 * var_a;
        Q(2, 1) = dt2 / 2.0 * var_a;
        Q(2, 2) = dt * var_a;

        // 航向角与角速度子块 [yaw, omega] (积分得 dt^3/3, dt^2/2, dt)
        Q(3, 3) = dt3 / 3.0 * var_yaw_a;
        Q(3, 4) = dt2 / 2.0 * var_yaw_a;
        Q(4, 3) = dt2 / 2.0 * var_yaw_a;
        Q(4, 4) = dt * var_yaw_a;

        Matrix5d P_pred = F_j * P_ * F_j.transpose() + Q;

        // --- 4. 动态观测噪声 R (去除硬上限截断，按单目几何方差自然缩放，保留数值安全下限) ---
        double height_factor = std::max(0.8, current_z / 2.0);
        double rotation_factor = std::max(1.0, 1.0 + angular_rate * 2.0);
        double angle_factor = std::max(1.0, 1.0 + std::pow(visual_angle_deg / 15.0, 2.0));

        double dynamic_r_pos = std::max(
            0.1,
            base_r_noise_ * height_factor * height_factor * rotation_factor * angle_factor);
        double dynamic_r_yaw = std::max(
            0.02,
            base_r_yaw_noise_ * rotation_factor * angle_factor);

        Eigen::Matrix3d R;
        R.setZero();
        R(0, 0) = dynamic_r_pos;
        R(1, 1) = dynamic_r_pos;
        R(2, 2) = dynamic_r_yaw;

        // --- 5. 计算 3D 观测残差与自适应 AKF (卡方检验新息门控) ---
        Eigen::Vector3d z(meas_x, meas_y, normalize_angle(meas_yaw));
        Eigen::Vector3d y;
        y(0) = z(0) - x_pred(0);
        y(1) = z(1) - x_pred(1);
        y(2) = normalize_angle(z(2) - x_pred(3)); // 航向残差环绕保护

        // 如果航向角残差超过 60 度 (异常跳变门限)，动态提高该帧航向噪声以防野值冲击
        if (std::abs(y(2)) > 1.0) {
            R(2, 2) *= 10.0;
        }

        Eigen::Matrix3d S = H_ * P_pred * H_.transpose() + R;
        epsilon = y.transpose() * S.inverse() * y;

        // 高空视野开阔度补偿平滑展宽
        double nominal_thresh = chi2_nominal_thresh_;
        double outlier_thresh = chi2_outlier_thresh_;
        if (current_z > 3.0) {
            double alt_relax = std::min(4.0, (current_z - 3.0) * 0.8);
            nominal_thresh += alt_relax;
            outlier_thresh += alt_relax * 1.5;
        }

        if (epsilon > outlier_thresh) {
            // 核心 1：【测量野值/剧烈突变】(>99.9% 置信度)
            // 坚决不放大 Q！对该帧观测 R 进行惩罚性膨胀降权，极度不信任该帧观测
            double r_penalty = std::min(100.0, epsilon / outlier_thresh);
            R *= r_penalty;
            S = H_ * P_pred * H_.transpose() + R;
        } else if (epsilon > nominal_thresh) {
            // 核心 1：【目标真实机动变道】(95% ~ 99.9%)
            // 温和放大 Q，上限严格受控于 max_q_scale_ (默认 3.0 倍)
            double scale_factor =
                std::min(max_q_scale_, epsilon / nominal_thresh);
            Matrix5d adaptive_Q = Q * scale_factor;
            P_pred = F_j * P_ * F_j.transpose() + adaptive_Q;
            S = H_ * P_pred * H_.transpose() + R;
        }

        // --- 6. 更新阶段 (Update) ---
        Eigen::Matrix<double, 5, 3> K = P_pred * H_.transpose() * S.inverse();

        x_ = x_pred + K * y;
        x_(3) = normalize_angle(x_(3));

        Matrix5d I = Matrix5d::Identity();
        P_ = (I - K * H_) * P_pred;

        // 核心 3：速度变化率动力学一致性限幅与物理极速硬截断
        if (update_count_ > 1) {
            double max_dv = max_dv_acc_ * dt;
            x_(2) = std::clamp(x_(2), last_v_ - max_dv, last_v_ + max_dv);
        }
        x_(2) = std::clamp(x_(2), -max_speed_, max_speed_);
        last_v_ = x_(2);

        return get_vel();
    }

    Eigen::Vector2d get_pos() const { return Eigen::Vector2d(x_(0), x_(1)); }

    // CTRV 模型的速度分解
    Eigen::Vector2d get_vel() const {
        if (update_count_ < 3) {
            return Eigen::Vector2d(0.0, 0.0);
        }
        double v = x_(2);
        double yaw = x_(3);
        return Eigen::Vector2d(v * std::cos(yaw), v * std::sin(yaw));
    }

    double get_yaw() const { return x_(3); }
    double get_yaw_rate() const { return x_(4); }

   private:
    using Vector5d = Eigen::Matrix<double, 5, 1>;
    using Matrix5d = Eigen::Matrix<double, 5, 5>;

    Vector5d x_;
    Matrix5d P_;
    Eigen::Matrix<double, 3, 5> H_;
};