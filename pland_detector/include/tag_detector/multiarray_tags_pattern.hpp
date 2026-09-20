#pragma once
#include <map>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "./safe_aprialtag.hpp"
#include "target_observation.hpp"

using LayoutMap = std::map<std::string, std::vector<Eigen::Vector3d>>;

class MultiArrayTagsPattern : public ITargetPattern {
   private:
    // 存储阵列中每个 Tag 的 3D 物理坐标 (从 Python 脚本的输出中获取)
    // 键为 Tag ID，值为 4 个角点的 3D 坐标 (顺序: 左下, 右下, 右上, 左上)
    LayoutMap tag_3d_layout_;

   public:
    MultiArrayTagsPattern(const LayoutMap& tag_3d_layout)
        : tag_3d_layout_(tag_3d_layout) {
        if (tag_3d_layout_.empty()) {
            throw std::runtime_error("[Pland] no config found!");
        }
    }

    TargetObservation process(const SafeDetections& detections,
                              SafeAprilTagDetector* detector,
                              const Eigen::Matrix3d& cam_K,
                              cv::Mat& draw_img) override {
        TargetObservation obs;
        obs.is_valid = false;

        std::vector<cv::Point3d> object_points;  // 3D 物理点
        std::vector<cv::Point2d> image_points;   // 2D 像素点

        // 1. 遍历检测结果，收集所有属于本阵列的 Tag 角点
        for (int i = 0; i < detections.size(); i++) {
            apriltag_detection_t* det = detections[i];

            // 检查这个 ID 是否在我们的阵列字典中
            if (tag_3d_layout_.find(std::to_string(det->id)) !=
                tag_3d_layout_.end()) {
                // 绘制识别框用于可视化
                for (int j = 0; j < 4; j++) {
                    cv::Point pt1(det->p[j][0], det->p[j][1]);
                    cv::Point pt2(det->p[(j + 1) % 4][0],
                                  det->p[(j + 1) % 4][1]);
                    cv::line(draw_img, pt1, pt2, cv::Scalar(255, 0, 0), 2);
                }
                std::string tag_label = "ID:" + std::to_string(det->id);
                cv::putText(
                    draw_img, tag_label, cv::Point(det->c[0], det->c[1]),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);

                // 将该 Tag 的 4 个 2D 角点压入集合
                for (int j = 0; j < 4; j++) {
                    image_points.push_back(
                        cv::Point2d(det->p[j][0], det->p[j][1]));
                }

                // 将该 Tag 对应的 4 个 3D 物理角点压入集合
                const auto& pts_3d = tag_3d_layout_[std::to_string(det->id)];
                for (int j = 0; j < 4; j++) {
                    object_points.push_back(
                        {pts_3d[j].x(), pts_3d[j].y(), pts_3d[j].z()});
                }
            }
        }

        // 2. 联合解算 (核心聚类步骤)
        // 至少需要 4 个点 (1 个 Tag) 才能解算，但点越多越精确
        if (image_points.size() >= 4) {
            cv::Mat rvec, tvec;
            cv::Mat camera_matrix =
                (cv::Mat_<double>(3, 3) << cam_K(0, 0), cam_K(0, 1),
                 cam_K(0, 2), cam_K(1, 0), cam_K(1, 1), cam_K(1, 2),
                 cam_K(2, 0), cam_K(2, 1), cam_K(2, 2));
            cv::Mat dist_coeffs =
                cv::Mat::zeros(4, 1, CV_64F);  // 假设图像已去畸变

            // 使用 RANSAC
            // 进行解算，这能有效剔除误识别的单帧噪点，实现你想要的"提取精确解"
            bool success =
                cv::solvePnPRansac(object_points, image_points, camera_matrix,
                                   dist_coeffs, rvec, tvec, false, 100, 8.0,
                                   0.99, cv::noArray(), cv::SOLVEPNP_ITERATIVE);

            if (success) {
                obs.is_valid = true;

                // 1. 将 OpenCV 的旋转向量(3x1)转换为旋转矩阵(3x3)
                cv::Mat R;
                cv::Rodrigues(rvec, R);

                obs.pose1.R << R.at<double>(0, 0), R.at<double>(0, 1),
                    R.at<double>(0, 2), R.at<double>(1, 0), R.at<double>(1, 1),
                    R.at<double>(1, 2), R.at<double>(2, 0), R.at<double>(2, 1),
                    R.at<double>(2, 2);

                obs.pose1.t << tvec.at<double>(0), tvec.at<double>(1),
                    tvec.at<double>(2);
                obs.pose1.error = 0.0;
                obs.pose1.valid = true;

                obs.pose2.valid = false;

                // 可视化中心点 (将 3D 原点投影到 2D 图像上)
                std::vector<cv::Point3d> center_3d = {cv::Point3d(0, 0, 0)};
                std::vector<cv::Point2d> center_2d;
                cv::projectPoints(center_3d, rvec, tvec, camera_matrix,
                                  dist_coeffs, center_2d);

                obs.center_pixel = center_2d[0];
                cv::circle(draw_img, center_2d[0], 6, cv::Scalar(0, 0, 255),
                           -1);
                cv::putText(
                    draw_img, "Center", center_2d[0] + cv::Point2d(10, -10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
            }
        }

        return obs;
    }
};