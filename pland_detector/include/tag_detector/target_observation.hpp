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
};