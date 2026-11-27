#pragma once
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

#include "safe_aprialtag.hpp"

// ==========================================
// 统一的视觉观测结果
// ==========================================
struct TargetObservation {
    bool is_valid = false;
    cv::Point2d center_pixel;  // 用于 LOS 射线法的中心像素
    TargetPose pose1;          // 主要位姿解
    TargetPose pose2;  // 备选位姿解（用于解决歧义，如果没有歧义则不合法）
};

// ==========================================
// 图案识别与绘制策略接口
// ==========================================
class ITargetPattern {
   public:
    virtual ~ITargetPattern() = default;

    /**
     * @brief 处理检测结果，提取统一的位姿观测，并在图像上绘制
     * @param detections AprilTag 原始检测结果
     * @param detector 用于调用位姿估计的安全指针
     * @param cam_K 相机内参矩阵
     * @param draw_img 用于绘制可视化结果的图像（In/Out）
     * @return 统一的观测结果
     */
    virtual TargetObservation process(const SafeDetections& detections,
                                      SafeAprilTagDetector* detector,
                                      const Eigen::Matrix3d& cam_K,
                                      cv::Mat& draw_img) = 0;

    /**
     * @brief 设置 PnP 相关参数 (可选, 默认实现忽略)
     * @param ransac_thresh_px RANSAC 重投影门限 (像素)
     * @param min_tags 最少检出 tag 数, 少于此数不出解
     * @param dist_coeffs 相机畸变系数 (k1,k2,p1,p2[,k3]); 空表示无畸变
     */
    virtual void set_pnp_params(double ransac_thresh_px, int min_tags,
                                const std::vector<double>& dist_coeffs) {
        (void)ransac_thresh_px;
        (void)min_tags;
        (void)dist_coeffs;
    }
};