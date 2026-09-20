#pragma once

extern "C" {
#include "apriltag.h"
#include "apriltag_pose.h"
#include "tag16h5.h"
#include "tag25h9.h"
#include "tag36h11.h"
#include "tagCustom48h12.h"
}
#include <Eigen/Dense>
#include <memory>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <vector>

struct TargetPose {
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    bool valid = false;
    double error = 0;
};

// ==========================================
// 1. 姿态资源的 RAII 包装器 (解决野指针和内存泄漏)
// ==========================================
struct SafeTagPose {
    apriltag_pose_t pose;
    double error;

    // 默认构造：强制零初始化，彻底干掉野指针！
    SafeTagPose() {
        pose = {nullptr, nullptr};
        error = 0.0;
    }

    ~SafeTagPose() {
        if (pose.R != nullptr) matd_destroy(pose.R);
        if (pose.t != nullptr) matd_destroy(pose.t);
    }

    // 禁用拷贝，防止重复释放
    SafeTagPose(const SafeTagPose&) = delete;
    SafeTagPose& operator=(const SafeTagPose&) = delete;

    // 允许移动
    SafeTagPose(SafeTagPose&& other) noexcept {
        pose = other.pose;
        error = other.error;
        other.pose = {nullptr, nullptr};
    }

    Eigen::Matrix3d getRotation() const {
        Eigen::Matrix3d R;
        if (pose.R) {
            R << pose.R->data[0], pose.R->data[1], pose.R->data[2],
                pose.R->data[3], pose.R->data[4], pose.R->data[5],
                pose.R->data[6], pose.R->data[7], pose.R->data[8];
        } else {
            R.setIdentity();
        }
        return R;
    }

    bool isValid() const { return pose.R != nullptr; }
};

// ==========================================
// 2. 检测结果数组的 RAII 包装器
// ==========================================
class SafeDetections {
   private:
    zarray_t* detections_ = nullptr;

   public:
    explicit SafeDetections(zarray_t* d) : detections_(d) {}

    ~SafeDetections() {
        if (detections_ != nullptr) {
            apriltag_detections_destroy(detections_);
        }
    }

    SafeDetections(const SafeDetections&) = delete;
    SafeDetections& operator=(const SafeDetections&) = delete;

    SafeDetections(SafeDetections&& other) noexcept
        : detections_(other.detections_) {
        other.detections_ = nullptr;
    }

    int size() const { return detections_ ? zarray_size(detections_) : 0; }

    // 重载 [] 操作符，像用 std::vector 一样用它
    apriltag_detection_t* operator[](int i) const {
        apriltag_detection_t* det = nullptr;
        zarray_get(detections_, i, &det);
        return det;
    }
};

// ==========================================
// 3. 安全检测器核心类
// ==========================================
class SafeAprilTagDetector {
   private:
    apriltag_family_t* tf_ = nullptr;
    apriltag_detector_t* td_ = nullptr;
    std::string tag_name_;

   public:
    SafeAprilTagDetector(std::string tag_name, int threads = 4)
        : tag_name_(tag_name) {
        if (tag_name == "tagCustom48h12") {
            tf_ = tagCustom48h12_create();
        } else if (tag_name == "tag16h5") {
            tf_ = tag16h5_create();
        } else if (tag_name == "tag25h9") {
            tf_ = tag25h9_create();
        } else if (tag_name == "tag36h11") {
            tf_ = tag36h11_create();
        } else {
            throw std::runtime_error(tag_name);
        }
        td_ = apriltag_detector_create();
        apriltag_detector_add_family(td_, tf_);
        td_->nthreads = threads;
    }

    ~SafeAprilTagDetector() {
        if (td_ != nullptr) apriltag_detector_destroy(td_);
        if (tf_ != nullptr) {
            if (tag_name_ == "tagCustom48h12") {
                tagCustom48h12_destroy(tf_);
            } else if (tag_name_ == "tag16h5") {
                tag16h5_destroy(tf_);
            } else if (tag_name_ == "tag25h9") {
                tag25h9_destroy(tf_);
            } else if (tag_name_ == "tag36h11") {
                tag36h11_destroy(tf_);
            } else {
                throw std::runtime_error(tag_name_);
            }
        }
    }

    // 第一步：只管检测，不管姿态。返回的 RAII 对象会在离开作用域时自动销毁
    SafeDetections detect(const cv::Mat& gray_img) {
        if (gray_img.empty() || gray_img.type() != CV_8UC1) {
            return SafeDetections(nullptr);
        }
        image_u8_t im = {.width = gray_img.cols,
                         .height = gray_img.rows,
                         .stride = static_cast<int32_t>(gray_img.step[0]),
                         .buf = gray_img.data};
        return SafeDetections(apriltag_detector_detect(td_, &im));
    }

    // 第二步：只针对你挑出来的那一个 Tag 算姿态！
    void estimatePose(apriltag_detection_t* det, double tag_size, double fx,
                      double fy, double cx, double cy, TargetPose& out_pose1,
                      TargetPose& out_pose2) {
        apriltag_detection_info_t info = {.det = det,
                                          .tagsize = tag_size,
                                          .fx = fx,
                                          .fy = fy,
                                          .cx = cx,
                                          .cy = cy};

        // 1. 声明原生的 C 指针用于接收 AprilTag 底层分配的数据
        apriltag_pose_t* raw_pose1 = nullptr;
        apriltag_pose_t* raw_pose2 = nullptr;

        // 2. 调用底层解算函数 (注意这里传入的是指针的地址)
        estimate_tag_pose_orthogonal_iteration(&info, &out_pose1.error,
                                               raw_pose1, &out_pose2.error,
                                               raw_pose2, 50);

        // 3. 将解算结果转换为 Eigen 格式并释放底层 C 内存
        if (raw_pose1 != nullptr) {
            out_pose1.valid = true;
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    // 使用 MATD_EL 宏安全读取 matd_t 元素
                    out_pose1.R(i, j) = MATD_EL(raw_pose1->R, i, j);
                }
                out_pose1.t(i) = MATD_EL(raw_pose1->t, i, 0);
            }
            // ⚠️
            // 极其关键：必须手动释放底层的矩阵和结构体内存，否则会严重泄漏
            matd_destroy(raw_pose1->R);
            matd_destroy(raw_pose1->t);
            free(raw_pose1);
        } else {
            out_pose1.valid = false;
        }

        // 4. 同理处理 pose2
        if (raw_pose2 != nullptr) {
            out_pose2.valid = true;
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    out_pose2.R(i, j) = MATD_EL(raw_pose2->R, i, j);
                }
                out_pose2.t(i) = MATD_EL(raw_pose2->t, i, 0);
            }
            matd_destroy(raw_pose2->R);
            matd_destroy(raw_pose2->t);
            free(raw_pose2);
        } else {
            out_pose2.valid = false;
        }
    }
};
