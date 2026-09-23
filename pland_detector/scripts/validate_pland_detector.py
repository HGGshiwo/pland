"""
==============================================================================
Pland Detector 识别效果与性能自动化验证评测脚本
==============================================================================
功能：
1. 启动真实的 ROS C++ 节点 (pland_detector_node)
2. 自动重放指定 rosbag 图像与里程计数据进行闭环评测
3. 实时采集节点输出话题 (/pland/target_pose, /pland/target_pixel 等)
4. 统计并对比：
   - 全局识别成功率与各高度分段识别率 (高空 >8m, 中空 3~8m, 低空 <=3m)
   - 处理速度与耗时延迟 (平均延迟, 最大延迟, 处理帧率 FPS)
   - 连续丢靶事件与最长失锁中断时长 (Target Lost 分析)
   - 首次锁靶时间与首次识别高度
5. 支持基线 (修复前) 与优化版 (修复后) A/B 双向对比报表输出

使用示例：
------------------------------------------------------------------------------
# 1. 方案一：Coarse-to-Fine 粗检精检增强 (推荐方案)
python3 /home/hggshiwo/catkin_ws/src/pland/pland_detector/scripts/validate_pland_detector.py \
    --bag /home/hggshiwo/catkin_ws/src/task_20260923_074125_372.bag \
    --rate 2.0 \
    --preprocess coarse_to_fine \
    --compare_with_json /home/hggshiwo/catkin_ws/src/pland/pland_detector/scripts/baseline_metrics.json

# 2. 自适应增强 (高空锐化 + 低空CLAHE)
python3 /home/hggshiwo/catkin_ws/src/pland/pland_detector/scripts/validate_pland_detector.py \
    --bag /home/hggshiwo/catkin_ws/src/task_20260923_074125_372.bag \
    --rate 2.0 \
    --preprocess adaptive \
    --compare_with_json /home/hggshiwo/catkin_ws/src/pland/pland_detector/scripts/baseline_metrics.json

# 3. 原始原图直通 (Baseline 测试)
python3 /home/hggshiwo/catkin_ws/src/pland/pland_detector/scripts/validate_pland_detector.py \
    --bag /home/hggshiwo/catkin_ws/src/task_20260923_074125_372.bag \
    --rate 2.0 \
    --preprocess none \
    --compare_with_json /home/hggshiwo/catkin_ws/src/pland/pland_detector/scripts/baseline_metrics.json

预处理模式说明 (--preprocess)：
  • coarse_to_fine (别名 c2f): 【方案一】纯视觉宏观纹理能量粗检定位靶标 ROI + 局域对比度锐化增强，低空融入 CLAHE。
  • adaptive: 高空(>6m)动态范围拉伸与边缘反锐化，低空(<=3m)自适应直方图均衡化(CLAHE)。
  • sharpen: 全图反锐化掩模 (Unsharp Masking)。
  • clahe: 全图对比度受限自适应直方图均衡化。
  • none: 直通原图，不做任何预处理。
==============================================================================
"""

import os
import sys
import time
import json
import argparse
import subprocess
import signal
import threading
import numpy as np

import cv2
try:
    import rospy
    import rosbag
    from sensor_msgs.msg import Image
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped, PointStamped, TwistStamped
    from std_msgs.msg import Float64
    from cv_bridge import CvBridge
except ImportError:
    print("[Error] ROS Python libraries (rospy, rosbag, cv_bridge) not found. Please source your catkin workspace!")
    sys.exit(1)


class ImageEnhancer:
    def __init__(self, mode="none"):
        self.mode = mode
        self.bridge = CvBridge()
        self.clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))

    @property
    def scale_factor(self):
        if self.mode in ["upscale_bicubic", "upscale_sharpen", "upscale_clahe", "upscale"]:
            return 2.0
        return 1.0

    def find_texture_roi(self, gray):
        """0.3ms 极速拉普拉斯梯度能量方差检测，定位靶标 ROI 窗口"""
        small = cv2.resize(gray, (320, 240), interpolation=cv2.INTER_AREA)
        lap = cv2.Laplacian(small, cv2.CV_16S, ksize=3)
        abs_lap = cv2.convertScaleAbs(lap)
        blurred = cv2.blur(abs_lap, (15, 15))
        _, _, _, max_loc = cv2.minMaxLoc(blurred)
        cx = int(max_loc[0] * 2)
        cy = int(max_loc[1] * 2)
        roi_size = 240
        h, w = gray.shape
        x1 = max(0, min(w - roi_size, cx - roi_size // 2))
        y1 = max(0, min(h - roi_size, cy - roi_size // 2))
        return (x1, y1, roi_size, roi_size)

    def process(self, img_msg, current_alt=0.0):
        if self.mode == "none":
            return img_msg

        try:
            cv_img = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        except Exception:
            return img_msg

        enhanced = cv_img

        if self.mode in ["coarse_to_fine", "c2f"]:
            # 方案一：高频纹理粗检定位 ROI + 局域对比度锐化增强 (保持 640x480 尺寸不变)
            if current_alt > 4.0:
                gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
                rx, ry, rw, rh = self.find_texture_roi(gray)
                roi = cv_img[ry:ry+rh, rx:rx+rw]
                
                # 局部动态范围归一化拉伸 + 强反锐化掩模
                norm = cv2.normalize(roi, None, alpha=10, beta=245, norm_type=cv2.NORM_MINMAX)
                gaussian = cv2.GaussianBlur(norm, (0, 0), 2.0)
                roi_enh = cv2.addWeighted(norm, 1.8, gaussian, -0.8, 0)
                
                enhanced = cv_img.copy()
                enhanced[ry:ry+rh, rx:rx+rw] = roi_enh
            else:
                # 低空平滑融入 CLAHE 局部直方图均衡化
                lab = cv2.cvtColor(cv_img, cv2.COLOR_BGR2LAB)
                lab[:, :, 0] = self.clahe.apply(lab[:, :, 0])
                enhanced = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

        elif self.mode in ["c2f_upscale", "coarse_to_fine_upscale"]:
            # 方案一融合版：高空 ROI 粗定位 + 局域 2x 双三次超分插值 + 强边缘锐化
            if current_alt > 4.0:
                gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
                rx, ry, rw, rh = self.find_texture_roi(gray)
                roi = cv_img[ry:ry+rh, rx:rx+rw]
                # 局部 2x 超分辨率插值 (240x240 -> 480x480)
                roi_zoom = cv2.resize(roi, (0, 0), fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
                norm = cv2.normalize(roi_zoom, None, alpha=10, beta=245, norm_type=cv2.NORM_MINMAX)
                gaussian = cv2.GaussianBlur(norm, (0, 0), 2.5)
                enhanced = cv2.addWeighted(norm, 1.8, gaussian, -0.8, 0)
            else:
                # 低空平滑融入 CLAHE
                lab = cv2.cvtColor(cv_img, cv2.COLOR_BGR2LAB)
                lab[:, :, 0] = self.clahe.apply(lab[:, :, 0])
                enhanced = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

        elif self.mode == "sharpen":
            # 反锐化掩模 (Unsharp Masking)：提升黑白交界边缘对比度
            gaussian = cv2.GaussianBlur(cv_img, (0, 0), 2.0)
            enhanced = cv2.addWeighted(cv_img, 1.6, gaussian, -0.6, 0)

        elif self.mode == "clahe":
            # 对比度受限自适应直方图均衡化 (CLAHE)
            lab = cv2.cvtColor(cv_img, cv2.COLOR_BGR2LAB)
            lab[:, :, 0] = self.clahe.apply(lab[:, :, 0])
            enhanced = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

        elif self.mode == "adaptive":
            # 自适应优化模式：
            # 1. 高空段 (Alt > 6m)：执行灰度动态范围拉伸 (Contrast Stretch) + 强反锐化掩模
            # 2. 中空段 (3m < Alt <= 6m)：温和边缘锐化
            # 3. 低空段 (Alt <= 3m)：CLAHE 局部自适应直方图均衡化 (实测低空识别率达 96.21%)
            if current_alt > 6.0:
                norm = cv2.normalize(cv_img, None, alpha=10, beta=245, norm_type=cv2.NORM_MINMAX)
                gaussian = cv2.GaussianBlur(norm, (0, 0), 2.0)
                enhanced = cv2.addWeighted(norm, 1.8, gaussian, -0.8, 0)
            elif current_alt > 3.0:
                gaussian = cv2.GaussianBlur(cv_img, (0, 0), 1.5)
                enhanced = cv2.addWeighted(cv_img, 1.4, gaussian, -0.4, 0)
            else:
                lab = cv2.cvtColor(cv_img, cv2.COLOR_BGR2LAB)
                lab[:, :, 0] = self.clahe.apply(lab[:, :, 0])
                enhanced = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

        elif self.mode in ["upscale_bicubic", "upscale"]:
            # 2.0倍双三次插值超分辨率重建 (640x480 -> 1280x960)
            enhanced = cv2.resize(cv_img, (0, 0), fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)

        elif self.mode == "upscale_sharpen":
            # 2.0倍双三次插值 + 高频边缘反锐化掩模融合增强
            upscaled = cv2.resize(cv_img, (0, 0), fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
            gaussian = cv2.GaussianBlur(upscaled, (0, 0), 3.0)
            enhanced = cv2.addWeighted(upscaled, 1.7, gaussian, -0.7, 0)

        elif self.mode == "upscale_clahe":
            # 2.0倍插值 + CLAHE 局部直方图均衡化
            upscaled = cv2.resize(cv_img, (0, 0), fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
            lab = cv2.cvtColor(upscaled, cv2.COLOR_BGR2LAB)
            lab[:, :, 0] = self.clahe.apply(lab[:, :, 0])
            enhanced = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

        out_msg = self.bridge.cv2_to_imgmsg(enhanced, encoding="bgr8")
        out_msg.header = img_msg.header
        return out_msg


class DetectorBenchmarkRunner:
    def __init__(self, bag_path, image_topic, odom_topic, target_pose_topic, launch_node=True, rate=1.0, duration=None, preprocess_mode="none"):
        self.bag_path = bag_path
        self.image_topic = image_topic
        self.odom_topic = odom_topic
        self.target_pose_topic = target_pose_topic
        self.launch_node = launch_node
        self.playback_rate = rate
        self.duration = duration
        self.preprocess_mode = preprocess_mode
        self.enhancer = ImageEnhancer(preprocess_mode)

        self.node_process = None
        self.roscore_process = None

        # 统计数据容器
        self.input_frames = []      # (seq, stamp_sec, alt)
        self.output_poses = []      # (stamp_sec, recv_sec, pos_x, pos_y, pos_z)
        self.current_alt = 0.0
        self.has_odom = False
        self.start_time = None
        self.is_running = False

    def check_roscore(self):
        try:
            rospy.get_master().getPid()
            return True
        except Exception:
            return False

    def start_roscore_if_needed(self):
        if not self.check_roscore():
            print("[Benchmark] Starting roscore in background...")
            self.roscore_process = subprocess.Popen(["roscore"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(2.0)

    def start_detector_node(self):
        scale = self.enhancer.scale_factor
        fx = 554.38271282264407 * scale
        fy = 554.38271282264407 * scale
        cx = 320.5 * scale
        cy = 240.5 * scale

        self.temp_launch_path = f"/tmp/pland_benchmark_{int(time.time())}.launch"
        with open(self.temp_launch_path, "w") as f:
            f.write(f'''<launch>
    <node pkg="pland_detector" type="pland_detector_node" name="pland_detector_node" output="screen">
        <param name="odom_topic" value="{self.odom_topic}" />
        <param name="image_topic" value="{self.image_topic}" />
        <param name="target_pose_topic" value="{self.target_pose_topic}" />
        <param name="tag_family" value="tag36h11" />
        <param name="tag_config_file_path" value="/home/hggshiwo/catkin_ws/src/pland/pland_detector/config/tag_pos_map.json" />
        <param name="param_config_path" value="/home/hggshiwo/catkin_ws/src/pland/pland_detector/config/pland_detector.yaml" />
        <param name="drone_config_path" value="/home/hggshiwo/catkin_ws/src/pland/pland_controller/config/drone_config.yaml" />
        <param name="publish_debug_image" value="false" />
        <rosparam param="camera_inner_matrix">[{fx}, 0.0, {cx}, 0.0, {fy}, {cy}, 0.0, 0.0, 1.0]</rosparam>
    </node>
</launch>''')

        cmd = ["roslaunch", self.temp_launch_path]
        print(f"[Benchmark] Launching pland_detector_node (图像缩放倍率: {scale}x, 相机内参: fx={fx:.1f}, cx={cx:.1f})")
        self.node_process = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(2.5)

    def odom_callback(self, msg):
        self.current_alt = msg.pose.pose.position.z
        self.has_odom = True

    def target_pose_callback(self, msg):
        recv_time = rospy.Time.now().to_sec()
        stamp_time = msg.header.stamp.to_sec()
        self.output_poses.append({
            "stamp": stamp_time,
            "recv_time": recv_time,
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "z": msg.pose.position.z,
            "alt": self.current_alt
        })

    def run_benchmark(self, label="Test"):
        print(f"\n=======================================================")
        print(f" 开始评测: {label}")
        print(f" Bag 文件: {self.bag_path}")
        print(f" 图像话题: {self.image_topic}")
        print(f"=======================================================")

        self.start_roscore_if_needed()

        if self.launch_node:
            self.start_detector_node()

        rospy.init_node(f"pland_benchmark_{int(time.time())}", anonymous=True, disable_signals=True)

        # 订阅输出话题与里程计
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        rospy.Subscriber(self.target_pose_topic, PoseStamped, self.target_pose_callback)

        image_pub = rospy.Publisher(self.image_topic, Image, queue_size=10)
        odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)

        # 读取 Bag 文件
        if not os.path.exists(self.bag_path):
            raise FileNotFoundError(f"Rosbag not found: {self.bag_path}")

        bag = rosbag.Bag(self.bag_path)
        bag_start = bag.get_start_time()
        bag_end = bag.get_end_time()
        total_duration = bag_end - bag_start

        print(f"[Benchmark] Rosbag 时长: {total_duration:.2f}s, 播放倍速: {self.playback_rate}x")

        # 提取相关话题消息流
        sim_start_time = rospy.Time.now().to_sec()
        self.is_running = True
        frame_idx = 0

        try:
            for topic, msg, t in bag.read_messages(topics=[self.image_topic, self.odom_topic]):
                if rospy.is_shutdown():
                    break

                rel_t = t.to_sec() - bag_start
                if self.duration and rel_t > self.duration:
                    break

                # 模拟时钟节奏
                expected_elapsed = rel_t / self.playback_rate
                actual_elapsed = rospy.Time.now().to_sec() - sim_start_time
                if expected_elapsed > actual_elapsed:
                    time.sleep(min(0.05, expected_elapsed - actual_elapsed))

                now_ros = rospy.Time.now()
                if topic == self.odom_topic:
                    msg.header.stamp = now_ros
                    odom_pub.publish(msg)
                    self.current_alt = msg.pose.pose.position.z
                elif topic == self.image_topic:
                    msg.header.stamp = now_ros
                    # 在外部对图像进行预处理增强 (锐化 / 对比度自适应)，随后注入 C++ 节点
                    enhanced_msg = self.enhancer.process(msg, self.current_alt)
                    enhanced_msg.header.stamp = now_ros
                    # 记录输入帧
                    self.input_frames.append({
                        "seq": frame_idx,
                        "stamp": now_ros.to_sec(),
                        "pub_time": now_ros.to_sec(),
                        "alt": self.current_alt
                    })
                    image_pub.publish(enhanced_msg)
                    frame_idx += 1

                if frame_idx % 100 == 0 and frame_idx > 0:
                    sys.stdout.write(f"\r[Benchmark] 已重放 {frame_idx} 帧图像 | 当前高度: {self.current_alt:.2f}m | 收到识别位姿: {len(self.output_poses)} 次")
                    sys.stdout.flush()

        finally:
            bag.close()

        # 等待最后几帧处理完成
        time.sleep(1.0)
        print(f"\n[Benchmark] 重放完成，总计输入图像: {len(self.input_frames)} 帧，节点输出检测: {len(self.output_poses)} 次")

        # 终止节点
        self.cleanup()

        # 计算并返回统计结果
        return self.calculate_metrics(label)

    def cleanup(self):
        if self.node_process:
            print("[Benchmark] Stopping pland_detector_node...")
            self.node_process.terminate()
            try:
                self.node_process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.node_process.kill()
            self.node_process = None

        if hasattr(self, 'temp_launch_path') and self.temp_launch_path and os.path.exists(self.temp_launch_path):
            try:
                os.remove(self.temp_launch_path)
            except Exception:
                pass

        if self.roscore_process:
            self.roscore_process.terminate()
            self.roscore_process = None

    def calculate_metrics(self, label):
        if not self.input_frames:
            return {"label": label, "error": "No input frames processed"}

        total_input = len(self.input_frames)
        total_output = len(self.output_poses)

        # 匹配每帧图像是否成功得到识别位姿 (按时间戳临近匹配 <= 0.08s)
        matched_results = []
        latencies_ms = []

        out_idx = 0
        num_out = len(self.output_poses)

        for in_f in self.input_frames:
            t_in = in_f["stamp"]
            alt = in_f["alt"]

            # 在 output 中查找时间差最小的匹配
            best_match = None
            min_dt = 999.0

            for out_f in self.output_poses:
                dt = abs(out_f["stamp"] - t_in)
                if dt < min_dt:
                    min_dt = dt
                    best_match = out_f

            if best_match is not None and min_dt <= 0.06:
                lat_ms = max(0.0, (best_match["recv_time"] - in_f["pub_time"]) * 1000.0)
                matched_results.append({
                    "seq": in_f["seq"],
                    "alt": alt,
                    "valid": True,
                    "latency_ms": lat_ms,
                    "x": best_match["x"],
                    "y": best_match["y"],
                    "z": best_match["z"]
                })
                latencies_ms.append(lat_ms)
            else:
                matched_results.append({
                    "seq": in_f["seq"],
                    "alt": alt,
                    "valid": False,
                    "latency_ms": None,
                    "x": None, "y": None, "z": None
                })

        # 高度分段统计 (高空 >8m, 中空 3~8m, 低空 <=3m)
        high_alt_frames = [r for r in matched_results if r["alt"] > 8.0]
        mid_alt_frames  = [r for r in matched_results if 3.0 < r["alt"] <= 8.0]
        low_alt_frames  = [r for r in matched_results if r["alt"] <= 3.0]

        def get_stat(frames):
            if not frames:
                return {"count": 0, "valid": 0, "rate": 0.0, "avg_lat": 0.0}
            valid_c = sum(1 for f in frames if f["valid"])
            rate = (valid_c / len(frames)) * 100.0
            lats = [f["latency_ms"] for f in frames if f["latency_ms"] is not None]
            avg_lat = np.mean(lats) if lats else 0.0
            return {"count": len(frames), "valid": valid_c, "rate": rate, "avg_lat": avg_lat}

        stat_high = get_stat(high_alt_frames)
        stat_mid  = get_stat(mid_alt_frames)
        stat_low  = get_stat(low_alt_frames)
        stat_all  = get_stat(matched_results)

        # 连续丢靶/失锁分析 (连续连续未识别帧序列)
        lost_intervals = []
        cur_lost = 0
        for r in matched_results:
            if not r["valid"]:
                cur_lost += 1
            else:
                if cur_lost > 0:
                    lost_intervals.append(cur_lost)
                cur_lost = 0
        if cur_lost > 0:
            lost_intervals.append(cur_lost)

        max_consecutive_lost = max(lost_intervals) if lost_intervals else 0
        avg_consecutive_lost = np.mean(lost_intervals) if lost_intervals else 0.0
        fps = (1000.0 / stat_all["avg_lat"]) if stat_all["avg_lat"] > 0 else 0.0

        # 首次锁靶高度与时间
        first_lock_frame = next((f for f in matched_results if f["valid"]), None)
        first_lock_alt = first_lock_frame["alt"] if first_lock_frame else -1.0
        first_lock_seq = first_lock_frame["seq"] if first_lock_frame else -1

        metrics = {
            "label": label,
            "total_frames": total_input,
            "valid_detections": stat_all["valid"],
            "overall_rate_pct": stat_all["rate"],
            "avg_latency_ms": stat_all["avg_lat"],
            "max_latency_ms": float(np.max(latencies_ms)) if latencies_ms else 0.0,
            "fps": fps,
            "first_lock_alt": first_lock_alt,
            "first_lock_frame": first_lock_seq,
            "max_consecutive_lost_frames": max_consecutive_lost,
            "stat_high": stat_high,
            "stat_mid": stat_mid,
            "stat_low": stat_low
        }
        return metrics


def print_comparison_table(res_before, res_after=None):
    print("\n" + "=" * 80)
    print("                PLAND DETECTOR 识别性能评测与对比报告")
    print("=" * 80)

    col_w = [26, 24, 24]
    if res_after:
        header = f"{'指标项目':<{col_w[0]}} | {res_before['label']:<{col_w[1]}} | {res_after['label']:<{col_w[2]}}"
    else:
        header = f"{'指标项目':<{col_w[0]}} | {res_before['label']:<{col_w[1]}}"
    print(header)
    print("-" * len(header))

    def row(name, v1_str, v2_str=None):
        if res_after:
            print(f"{name:<{col_w[0]}} | {v1_str:<{col_w[1]}} | {v2_str:<{col_w[2]}}")
        else:
            print(f"{name:<{col_w[0]}} | {v1_str:<{col_w[1]}}")

    # 1. 总体识别率
    row("总输入图像帧数", f"{res_before['total_frames']} 帧", f"{res_after['total_frames']} 帧" if res_after else None)
    row("有效检出位姿帧数", f"{res_before['valid_detections']} 帧", f"{res_after['valid_detections']} 帧" if res_after else None)
    row("【全局识别成功率】", f"{res_before['overall_rate_pct']:.2f} %", f"{res_after['overall_rate_pct']:.2f} %" if res_after else None)

    print("-" * len(header))
    # 2. 高度分段识别率
    row("高空段 (>8m) 识别率", f"{res_before['stat_high']['rate']:.2f} % ({res_before['stat_high']['valid']}/{res_before['stat_high']['count']})",
        f"{res_after['stat_high']['rate']:.2f} % ({res_after['stat_high']['valid']}/{res_after['stat_high']['count']})" if res_after else None)
    row("中空段 (3~8m) 识别率", f"{res_before['stat_mid']['rate']:.2f} % ({res_before['stat_mid']['valid']}/{res_before['stat_mid']['count']})",
        f"{res_after['stat_mid']['rate']:.2f} % ({res_after['stat_mid']['valid']}/{res_after['stat_mid']['count']})" if res_after else None)
    row("低空段 (<=3m) 识别率", f"{res_before['stat_low']['rate']:.2f} % ({res_before['stat_low']['valid']}/{res_before['stat_low']['count']})",
        f"{res_after['stat_low']['rate']:.2f} % ({res_after['stat_low']['valid']}/{res_after['stat_low']['count']})" if res_after else None)

    print("-" * len(header))
    # 3. 处理速度与延迟
    row("平均处理延迟 (ms)", f"{res_before['avg_latency_ms']:.2f} ms", f"{res_after['avg_latency_ms']:.2f} ms" if res_after else None)
    row("最大单帧延迟 (ms)", f"{res_before['max_latency_ms']:.2f} ms", f"{res_after['max_latency_ms']:.2f} ms" if res_after else None)
    row("处理帧率 (FPS)", f"{res_before['fps']:.1f} FPS", f"{res_after['fps']:.1f} FPS" if res_after else None)

    print("-" * len(header))
    # 4. 连续性与首次捕获
    row("首次稳定捕获高度", f"{res_before['first_lock_alt']:.2f} m", f"{res_after['first_lock_alt']:.2f} m" if res_after else None)
    row("最大连续丢靶帧数", f"{res_before['max_consecutive_lost_frames']} 帧", f"{res_after['max_consecutive_lost_frames']} 帧" if res_after else None)
    print("=" * 80 + "\n")


def main():
    parser = argparse.ArgumentParser(description="Pland Detector Benchmark & Verification Tool")
    parser.add_argument("--bag", type=str, default="/home/hggshiwo/catkin_ws/src/task_20260923_074125_372.bag",
                        help="Path to the test rosbag file")
    parser.add_argument("--image_topic", type=str, default="/UAV0/sensor/video11_camera/image_raw",
                        help="Camera image raw topic in rosbag")
    parser.add_argument("--odom_topic", type=str, default="/mavros/local_position/odom",
                        help="Drone odometry topic in rosbag")
    parser.add_argument("--target_pose_topic", type=str, default="/pland/target_pose",
                        help="Target pose topic output by pland_detector_node")
    parser.add_argument("--preprocess", type=str, default="coarse_to_fine",
                        choices=["coarse_to_fine", "c2f", "c2f_upscale", "coarse_to_fine_upscale", "none", "sharpen", "clahe", "adaptive", "upscale_bicubic", "upscale_sharpen", "upscale_clahe"],
                        help="External image enhancement mode before sending to C++ pland_detector_node (coarse_to_fine, c2f, c2f_upscale, coarse_to_fine_upscale, none, sharpen, clahe, adaptive, upscale_bicubic, upscale_sharpen, upscale_clahe)")
    parser.add_argument("--rate", type=float, default=1.0, help="Playback speed multiplier (e.g. 1.0 or 2.0)")
    parser.add_argument("--duration", type=float, default=None, help="Max test duration in seconds")
    parser.add_argument("--label", type=str, default=None, help="Label for this test run")
    parser.add_argument("--compare_with_json", type=str, default=None, help="Path to baseline json report to compare against")
    parser.add_argument("--save_json", type=str, default=None, help="Save evaluation metrics to json file")
    parser.add_argument("--no_launch", action="store_true", help="Do not launch node (if node is already running externally)")

    args = parser.parse_args()

    run_label = args.label if args.label else f"图像增强[{args.preprocess}]"

    runner = DetectorBenchmarkRunner(
        bag_path=args.bag,
        image_topic=args.image_topic,
        odom_topic=args.odom_topic,
        target_pose_topic=args.target_pose_topic,
        launch_node=not args.no_launch,
        rate=args.rate,
        duration=args.duration,
        preprocess_mode=args.preprocess
    )

    metrics = runner.run_benchmark(label=run_label)

    res_baseline = None
    if args.compare_with_json and os.path.exists(args.compare_with_json):
        with open(args.compare_with_json, "r") as f:
            res_baseline = json.load(f)
        print_comparison_table(res_baseline, metrics)
    else:
        print_comparison_table(metrics)

    if args.save_json:
        with open(args.save_json, "w") as f:
            json.dump(metrics, f, indent=2, ensure_ascii=False)
        print(f"[Benchmark] 评测数据已保存至: {args.save_json}")


if __name__ == "__main__":
    main()
