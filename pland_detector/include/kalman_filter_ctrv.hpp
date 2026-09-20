#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

class KalmanFilterCTRV {
   public:
    // 基础观测噪声方差 (理想状态下的像素投影误差)
    double base_r_noise_ = 0.3;
    // AKF 马氏距离阈值 (超过此值判定为目标机动)
    double adaptive_threshold_ = 5.0;
    double max_q_scale_ = 10.0;
    int update_count_ = 0;  // 新增计数器

    KalmanFilterCTRV() {
        // 状态向量 x: [pos_x, pos_y, v, yaw, omega]^T
        x_.setZero();
        P_.setIdentity();
        P_ *= 10.0;

        // 观测矩阵 H: 我们只测量位置 [pos_x, pos_y]
        H_.setZero();
        H_(0, 0) = 1.0;
        H_(1, 1) = 1.0;
    }

    void reset() {
        x_.setZero();
        P_.setIdentity();
        P_ *= 10.0;
        update_count_ = 0;  // 重置时清零
    }

    // 辅助函数：将角度归一化到 [-pi, pi]
    static double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void force_set_state(double x, double y) {
        // 将状态向量直接重置
        x_ << x, y, 0.0, 0.0, 0.0;

        // 【重要】重置协方差矩阵 P
        // 因为状态已经被强制设定为极高置信度了，
        // 我们需要把协方差调小，告诉滤波器：“我现在对这个新位置非常确信”
        P_.setIdentity();
        P_ *= 0.01;  // 赋予一个很小的初始不确定度
    }

    // [重磅升级] 外部传入最大线加速度(max_acc)和最大角加速度(max_yaw_acc)
    Eigen::Vector2d update(double& epsilon, double meas_x, double meas_y,
                           double dt, double current_z, double angular_rate,
                           double visual_angle_deg, double max_acc = 1.0,
                           double max_yaw_acc = 0.5) {
        update_count_++;
        if (dt <= 1e-4) return get_vel();

        // 速度阻尼 (Velocity Damping / Leaky Integrator)
        // 就像给物理世界加入了空气阻力，如果没有强烈的连续观测支撑，速度会自动衰减
        // double velocity_damping = 0.95;  // 调参：0.95 代表每帧衰减 5% 的速度
        // x_(2) *= velocity_damping;

        // 提取当前状态
        double px = x_(0);
        double py = x_(1);
        double v = x_(2);
        double yaw = x_(3);
        double omega = x_(4);

        // --- 1. 非线性状态预测 (Predict State) ---
        Vector5d x_pred = x_;

        // 核心：分母极小值保护 (直线运动 vs 圆周运动)
        if (std::abs(omega) > 1e-4) {
            x_pred(0) =
                px + (v / omega) * (std::sin(yaw + omega * dt) - std::sin(yaw));
            x_pred(1) = py + (v / omega) *
                                 (-std::cos(yaw + omega * dt) + std::cos(yaw));
        } else {
            x_pred(0) = px + v * dt * std::cos(yaw);
            x_pred(1) = py + v * dt * std::sin(yaw);
        }
        x_pred(3) = normalize_angle(yaw + omega * dt);  // 航向角预测

        // --- 2. 计算雅可比矩阵 F_j (状态转移对状态量的偏导数) ---
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
            // omega 趋于0时的洛必达极限近似
            F_j(0, 4) = -0.5 * v * std::sin(yaw) * dt * dt;

            F_j(1, 2) = std::sin(yaw) * dt;
            F_j(1, 3) = v * std::cos(yaw) * dt;
            F_j(1, 4) = 0.5 * v * std::cos(yaw) * dt * dt;
        }
        F_j(3, 4) = dt;

        // --- 3. 动态生成过程噪声 Q (依赖外部传入的 max_acc 和 max_yaw_acc) ---
        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double dt4 = dt3 * dt;
        double var_a = max_acc * max_acc;
        double var_yaw_a = max_yaw_acc * max_yaw_acc;

        Matrix5d Q;
        Q.setZero();
        Q(0, 0) = dt4 / 4.0 * var_a;
        Q(0, 2) = dt3 / 2.0 * var_a;
        Q(1, 1) = dt4 / 4.0 * var_a;
        Q(1, 2) = dt3 / 2.0 * var_a;
        Q(2, 0) = dt3 / 2.0 * var_a;
        Q(2, 1) = dt3 / 2.0 * var_a;
        Q(2, 2) = dt2 * var_a;
        Q(3, 3) = dt4 / 4.0 * var_yaw_a;
        Q(3, 4) = dt3 / 2.0 * var_yaw_a;
        Q(4, 3) = dt3 / 2.0 * var_yaw_a;
        Q(4, 4) = dt2 * var_yaw_a;

        Matrix5d P_pred = F_j * P_ * F_j.transpose() + Q;

        // --- 4. 动态观测噪声 R ---
        double height_factor = std::clamp(current_z / 2.0, 0.8, 3.0);
        double rotation_factor = 1.0 + angular_rate * 5.0;
        double angle_factor = 1.0 + std::pow(visual_angle_deg / 10.0, 2.0);

        double dynamic_r = base_r_noise_ * height_factor * height_factor *
                           rotation_factor * angle_factor;

        Eigen::Matrix2d R;
        R.setIdentity();
        R *= dynamic_r;

        // --- 5. 计算残差与自适应 AKF ---
        Eigen::Vector2d z(meas_x, meas_y);
        Eigen::Vector2d y = z - H_ * x_pred;
        Eigen::Matrix2d S = H_ * P_pred * H_.transpose() + R;

        epsilon = y.transpose() * S.inverse() * y;

        double dynamic_threshold = adaptive_threshold_;

        if (current_z > 3.0) {
            // 超过 3 米的高空，每高 1 米，阈值增加 2.0
            dynamic_threshold += std::min(10.0, (current_z - 3.0) * 2.0);
        }

        if (epsilon > dynamic_threshold) {
            double scale_factor =
                std::min(max_q_scale_, epsilon / dynamic_threshold);
            Matrix5d adaptive_Q = Q * scale_factor;
            P_pred = F_j * P_ * F_j.transpose() + adaptive_Q;
            S = H_ * P_pred * H_.transpose() + R;
        }

        // --- 6. 更新阶段 (Update) ---
        Eigen::Matrix<double, 5, 2> K = P_pred * H_.transpose() * S.inverse();

        x_ = x_pred + K * y;
        x_(3) = normalize_angle(x_(3));  // 确保更新后的航向角依然在合法区间

        Matrix5d I = Matrix5d::Identity();
        P_ = (I - K * H_) * P_pred;

        return get_vel();
    }

    Eigen::Vector2d get_pos() const { return Eigen::Vector2d(x_(0), x_(1)); }

    // CTRV 模型的速度分解
    Eigen::Vector2d get_vel() const {
        if (update_count_ < 5) {
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
    Eigen::Matrix<double, 2, 5> H_;
};