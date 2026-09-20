#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>

// 定义目标运动状态枚举
enum class TargetState { STATIONARY, MOVING };

class TargetTracker {
   public:
    /**
     * @brief 构造函数
     * @param window_size 历史记录的最大帧数
     * @param base_threshold 判定运动的基础距离阈值，例如 0.2 米
     */
    TargetTracker(int window_size = 30, double base_threshold = 0.2)
        : window_size_(window_size),
          threshold_(base_threshold),
          state_(TargetState::STATIONARY),
          move_counter_(0) {}

    void reset() {
        history_.clear();
        state_ = TargetState::STATIONARY;
        move_counter_ = 0;
    }

    TargetState update(const Eigen::Vector2d& pos) {
        history_.push_back(pos);
        if (history_.size() > window_size_) {
            history_.pop_front();
        }

        // 基础数据量保护：至少攒够 5 帧（约0.16秒）再做任何高级判定
        if (history_.size() < 5) {
            return state_;
        }

        if (state_ == TargetState::STATIONARY) {
            // ==========================================
            // [抗噪起步逻辑]：质心偏离 + 连续帧防抖
            // ==========================================
            Eigen::Vector2d centroid(0, 0);
            int count = history_.size() - 1;

            // 计算前 N-1
            // 帧的历史质心（故意不把最新这一帧算进去，防止它就是个巨大噪点）
            for (int i = 0; i < count; ++i) {
                centroid += history_[i];
            }
            centroid /= count;

            // 检查当前最新点偏离历史“安乐窝”的距离
            double dist_to_centroid = (pos - centroid).norm();

            if (dist_to_centroid > threshold_ * 1.2) {
                // 确实偏离了！防抖计数器 +1
                move_counter_++;

                // 设定防抖门限：必须连续 3 帧（约 0.1 秒）都偏离，才确认起步
                if (move_counter_ >= 3) {
                    state_ = TargetState::MOVING;
                    move_counter_ = 0;  // 重置状态
                }
            } else {
                // 只要有一帧乖乖缩回质心附近，证明刚才那个偏离点只是噪点，立刻清零！
                move_counter_ = 0;
            }

        } else {  // state_ == TargetState::MOVING
            // ==========================================
            // [严谨刹车逻辑]：近期活动包围盒
            // ==========================================
            // 刹车不能急，看最近的 15 帧（约 0.5 秒）是否真的安静下来了
            int check_len = std::min(15, (int)history_.size());

            double min_x = history_.back().x(), max_x = history_.back().x();
            double min_y = history_.back().y(), max_y = history_.back().y();

            for (int i = history_.size() - check_len; i < history_.size();
                 ++i) {
                min_x = std::min(min_x, history_[i].x());
                max_x = std::max(max_x, history_[i].x());
                min_y = std::min(min_y, history_[i].y());
                max_y = std::max(max_y, history_[i].y());
            }

            double spread_dist = std::hypot(max_x - min_x, max_y - min_y);

            // 如果近 0.5 秒内的最大活动范围已经小于阈值的 0.8，确认停稳
            if (spread_dist < threshold_ * 0.8) {
                state_ = TargetState::STATIONARY;
                move_counter_ = 0;
            }
        }

        return state_;
    }

    bool isMoving() const { return state_ == TargetState::MOVING; }

   private:
    int window_size_;
    double threshold_;
    TargetState state_;
    std::deque<Eigen::Vector2d> history_;
    int move_counter_;  // [新增] 防抖计数器
};