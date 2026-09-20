#pragma once
#include <Eigen/Dense>
#include <cmath>
class KalmanFilterYaw {
   public:
    KalmanFilterYaw() {
        x_.setZero();  // 状态: [yaw, yaw_rate]^T
        P_.setIdentity();
        F_.setIdentity();
        H_ << 1.0, 0.0;  // 只测量角度 yaw

        Q_.setIdentity();
        Q_ *= 0.05;  // 过程噪声 (目标实际旋转的灵敏度)

        R_ = 0.5;  // 观测噪声 (视觉角度抖动，设大一点让其平滑)
    }
    void reset(double initial_yaw = 0.0) {
        x_ << initial_yaw, 0.0;
        P_.setIdentity();
    }
    // 将角度限制在 -pi 到 pi 之间 (核心除错魔法)
    static double normalize_angle(double angle) {
        // 利用 atan2(sin, cos) 是最稳妥的规范化方法
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    void force_set_state(double yaw) {
        // 将状态向量直接重置
        x_ << yaw, 0.0;

        // 【重要】重置协方差矩阵 P
        // 因为状态已经被强制设定为极高置信度了，
        // 我们需要把协方差调小，告诉滤波器：“我现在对这个新位置非常确信”
        P_.setIdentity();
        P_ *= 0.01;  // 赋予一个很小的初始不确定度
    }

    // 输入测量到的 Yaw (弧度制) 和 dt，返回滤波后的 [yaw, yaw_rate]
    Eigen::Vector2d update(double meas_yaw, double dt) {
        if (dt <= 0) return x_;
        // --- 1. 预测 (Predict) ---
        F_(0, 1) = dt;
        x_ = F_ * x_;
        // 预测完也要规范化一下
        x_(0) = normalize_angle(x_(0));

        P_ = F_ * P_ * F_.transpose() + Q_;
        // --- 2. 更新 (Update) ---
        // 【绝对关键步】：计算残差时，必须求最短路径！
        double innovation = meas_yaw - x_(0);
        innovation =
            normalize_angle(innovation);  // 比如 -358度 会被修正为 +2度
        double S = H_ * P_ * H_.transpose() + R_;
        Eigen::Vector2d K = P_ * H_.transpose() / S;
        // 修正状态
        x_ = x_ + K * innovation;
        x_(0) = normalize_angle(x_(0));  // 再次规范化
        Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
        P_ = (I - K * H_) * P_;
        return x_;
    }

    double get_yaw() const { return x_(0); }
    double get_yaw_rate() const { return x_(1); }

   private:
    Eigen::Vector2d x_;  // [yaw, yaw_rate]
    Eigen::Matrix2d P_;
    Eigen::Matrix2d F_;
    Eigen::RowVector2d H_;
    Eigen::Matrix2d Q_;
    double R_;
};