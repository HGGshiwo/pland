"""
==============================================================================
Pland Detector 识别效果与性能自动化验证评测脚本
==============================================================================
功能：
1. 启动真实的 ROS C++ 节点 (pland_detector_node)
2. 自动重放指定 rosbag 图像与里程计数据进行闭环评测
3. 实时采集节点输出话题 (/pland/target_pose, /pland/target_pixel 等)
4. 统计并对比：
   - 全局识别成功率与各高度分段识别率 (每 5m 一个高度区间)
   - 处理速度与耗时延迟 (平均延迟, 最大延迟, 处理帧率 FPS)
   - 连续丢靶事件与最长失锁中断时长 (Target Lost 分析)
   - 首次锁靶时间与首次识别高度
5. 评测结束自动弹出逐帧统计图窗口 (每条消息的识别状态与滑动识别率、tag 检出数量、飞行高度折线 + 每 5m 分段明细)
6. 支持基线 (修复前) 与优化版 (修复后) A/B 双向对比报表输出

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
    from std_msgs.msg import Float64, Int32
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


def compute_altitude_bins(matched_results, bin_size=5.0):
    """按固定高度区间 (默认每 5m) 统计识别率与平均延迟，替代旧的高/中/低空三段分类"""
    num_bins = 0
    for r in matched_results:
        idx = int(max(0.0, r["alt"]) // bin_size)
        num_bins = max(num_bins, idx + 1)

    bins = []
    for idx in range(num_bins):
        lo, hi = idx * bin_size, (idx + 1) * bin_size
        frames = [r for r in matched_results if lo <= max(0.0, r["alt"]) < hi]
        valid_c = sum(1 for f in frames if f["valid"])
        lats = [f["latency_ms"] for f in frames if f["latency_ms"] is not None]
        bins.append({
            "lo": lo,
            "hi": hi,
            "label": f"{lo:.0f}~{hi:.0f}m",
            "count": len(frames),
            "valid": valid_c,
            "rate": (valid_c / len(frames)) * 100.0 if frames else 0.0,
            "avg_lat": float(np.mean(lats)) if lats else 0.0
        })
    return bins


def show_altitude_rate_chart(metrics, save_path=None, show=True):
    """弹出逐帧评测统计图: 以飞行高度为横轴逐条绘制, 包含识别状态与滑动识别率、tag 检出数量两条折线 + 每 5m 分段文字明细"""
    frame_valid = metrics.get("frame_valid") or []
    if not frame_valid:
        print("[Chart] 无逐帧统计数据，跳过绘图")
        return
    frame_tags = metrics.get("frame_tag_count") or [0] * len(frame_valid)
    frame_alt = metrics.get("frame_alt") or [0.0] * len(frame_valid)
    bins = metrics.get("alt_bins_5m") or []

    import matplotlib
    matplotlib.use("TkAgg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager

    # 选择中文字体，避免图中中文显示为方框 (WSL 下可直接借用 Windows 字体)
    cjk = None
    for name in ["WenQuanYi Micro Hei", "WenQuanYi Zen Hei", "Microsoft YaHei", "SimHei", "Noto Sans CJK SC"]:
        if any(f.name == name for f in font_manager.fontManager.ttflist):
            cjk = name
            break
    if cjk is None:
        for path in ["/mnt/c/Windows/Fonts/msyh.ttc", "/mnt/c/Windows/Fonts/simhei.ttf"]:
            if os.path.exists(path):
                try:
                    if hasattr(font_manager, "addfont"):
                        font_manager.fontManager.addfont(path)
                    cjk = font_manager.FontProperties(fname=path).get_name()
                    break
                except Exception:
                    pass
    if cjk:
        plt.rcParams["font.family"] = cjk
    plt.rcParams["axes.unicode_minus"] = False

    overall_rate = metrics.get("overall_rate_pct", 0.0)

    fig = plt.figure(figsize=(15.5, 9.0))
    fig.suptitle(f"Pland Detector 逐帧识别统计  ({metrics.get('label', '')})   "
                 f"全局识别率 {overall_rate:.2f}% ({metrics.get('valid_detections', 0)}/{metrics.get('total_frames', 0)})",
                 fontsize=14, fontweight="bold")

    # 横轴: 飞行高度 (m), 刻度约取 10 个
    alt_min, alt_max = min(frame_alt), max(frame_alt)
    span = max(1.0, alt_max - alt_min)
    step = next((s for s in [0.5, 1.0, 2.0, 5.0, 10.0] if span / s <= 12), 10.0)
    x0 = np.floor(alt_min / step) * step
    x1 = np.ceil(alt_max / step) * step
    xticks = list(np.arange(x0, x1 + step / 2, step))

    left, width = 0.06, 0.55
    rows = [0.53, 0.08]               # 两个子图底部位置
    h = 0.38

    # 子图1: 逐帧识别状态 (0/1) + 滑动窗口识别率, 横轴为高度
    ax1 = fig.add_axes([left, rows[0], width, h])
    ax1.step(frame_alt, frame_valid, where="post", color="#9e9e9e", linewidth=0.8)
    ax1.set_ylim(-0.15, 1.15)
    ax1.set_yticks([0, 1])
    ax1.set_yticklabels(["未识别", "识别"])
    ax1.set_ylabel("识别状态")
    ax1.grid(axis="both", linestyle=":", alpha=0.5, zorder=0)
    win = 20
    if len(frame_valid) >= win:
        rolling = np.convolve(frame_valid, np.ones(win) / win, mode="valid") * 100.0
        ax1r = ax1.twinx()
        ax1r.plot(frame_alt[win - 1:], rolling, color="#1f77b4", linewidth=1.8,
                  label=f"滑动识别率 (每{win}帧)")
        ax1r.axhline(overall_rate, color="#d9483b", linestyle="--", linewidth=1.2,
                     label=f"全局 {overall_rate:.1f}%")
        ax1r.set_ylim(0, 108)
        ax1r.set_ylabel("滑动识别率 (%)")
        ax1r.legend(loc="upper right", fontsize=8.5)

    # 子图2: 逐帧识别出的 tag 数量, 横轴为高度
    ax2 = fig.add_axes([left, rows[1], width, h], sharex=ax1)
    ax2.plot(frame_alt, frame_tags, color="#2e9e4f", linewidth=1.1)
    ax2.fill_between(frame_alt, frame_tags, color="#2e9e4f", alpha=0.15)
    ax2.set_ylabel("识别 tag 数量")
    ax2.set_xlabel("飞行高度 (m)")
    ax2.set_ylim(0, max(1.5, max(frame_tags) + 0.5))
    ax2.grid(axis="both", linestyle=":", alpha=0.5, zorder=0)
    for ax in (ax1, ax2):
        ax.set_xticks(xticks)
        ax.set_xlim(alt_min - span * 0.01, alt_max + span * 0.01)
        ax.tick_params(axis="x", labelsize=9)

    # 右侧: 每 5m 分段文字明细
    ax4 = fig.add_axes([0.66, 0.06, 0.32, 0.86])
    ax4.axis("off")
    bin_size = metrics.get("alt_bin_size", 5.0)
    lines = [f"高度分段明细 (每 {bin_size:.0f}m 统计)", "-" * 36]
    for b in bins:
        if b["count"] == 0:
            lines.append(f"{b['label']}: 无数据")
        else:
            lines.append(f"{b['label']}: {b['rate']:.2f}% ({b['valid']}/{b['count']})"
                         f"  平均延迟 {b['avg_lat']:.1f}ms")
    lines.append("-" * 36)
    lines.append(f"全局: {overall_rate:.2f}% ({metrics.get('valid_detections', 0)}/{metrics.get('total_frames', 0)})"
                 f"  平均延迟 {metrics.get('avg_latency_ms', 0.0):.1f}ms")
    ax4.text(0.0, 1.0, "\n".join(lines), va="top", ha="left", fontsize=11.5, linespacing=1.7,
             bbox=dict(boxstyle="round,pad=0.6", facecolor="#f5f5f0", edgecolor="#999999"))

    if save_path:
        try:
            fig.savefig(save_path, dpi=130)
            print(f"[Chart] 统计图已保存: {save_path}")
        except Exception as e:
            print(f"[Chart] 统计图保存失败: {e}")

    if show:
        try:
            plt.show()
        except Exception as e:
            print(f"[Chart] 图形窗口显示失败 ({e})，请直接查看保存的图片文件")
    else:
        plt.close(fig)


class DetectorBenchmarkRunner:
    def __init__(self, bag_path, image_topic, odom_topic, target_pose_topic, launch_node=True, rate=1.0, duration=None, preprocess_mode="none", tag_count_topic=None, disable_node_enhancement=False, enhance_mode=None):
        self.bag_path = bag_path
        self.image_topic = image_topic
        self.odom_topic = odom_topic
        self.target_pose_topic = target_pose_topic
        self.tag_count_topic = tag_count_topic or (target_pose_topic + "_tag_count")
        self.launch_node = launch_node
        self.playback_rate = rate
        self.duration = duration
        self.preprocess_mode = preprocess_mode
        self.disable_node_enhancement = disable_node_enhancement
        self.enhance_mode = enhance_mode
        self.enhancer = ImageEnhancer(preprocess_mode)

        self.node_process = None
        self.roscore_process = None

        # 统计数据容器
        self.input_frames = []      # (seq, stamp_sec, alt)
        self.output_poses = []      # (stamp_sec, recv_sec, pos_x, pos_y, pos_z)
        self.tag_counts = []        # (recv_sec, tag_count) 逐条 tag 计数消息
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

        disable_param = ('        <param name="disable_all_enhancement" value="true" />\n'
                         if self.disable_node_enhancement else '')
        enhance_mode_param = (f'        <param name="enhance_mode" value="{self.enhance_mode}" />\n'
                              if self.enhance_mode else '')
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
{disable_param}{enhance_mode_param}        <rosparam param="camera_inner_matrix">[{fx}, 0.0, {cx}, 0.0, {fy}, {cy}, 0.0, 0.0, 1.0]</rosparam>
    </node>
</launch>''')

        cmd = ["roslaunch", self.temp_launch_path]
        extra = ""
        if self.disable_node_enhancement:
            extra += ", 节点内部增强已关闭"
        if self.enhance_mode:
            extra += f", enhance_mode={self.enhance_mode}"
        print(f"[Benchmark] Launching pland_detector_node (图像缩放倍率: {scale}x, 相机内参: fx={fx:.1f}, cx={cx:.1f}{extra})")
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

    def tag_count_callback(self, msg):
        self.tag_counts.append({
            "recv": rospy.Time.now().to_sec(),
            "count": int(msg.data)
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
        rospy.Subscriber(self.tag_count_topic, Int32, self.tag_count_callback)
        print(f"[Benchmark] 订阅 tag 计数话题: {self.tag_count_topic}")

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

        # 将 tag 计数消息按接收时间就近对齐到输入帧 (阈值 0.2s)
        tag_counts_sorted = sorted(self.tag_counts, key=lambda c: c["recv"])

        def nearest_tag_count(t_pub):
            best, best_dt = None, 999.0
            for c in tag_counts_sorted:
                dt = abs(c["recv"] - t_pub)
                if dt < best_dt:
                    best_dt, best = dt, c
                if c["recv"] > t_pub + 0.5:
                    break
            return best["count"] if best is not None and best_dt <= 0.2 else None

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
                    "tag_count": nearest_tag_count(in_f["pub_time"]),
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
                    "tag_count": nearest_tag_count(in_f["pub_time"]),
                    "x": None, "y": None, "z": None
                })

        # 总体统计
        def get_stat(frames):
            if not frames:
                return {"count": 0, "valid": 0, "rate": 0.0, "avg_lat": 0.0}
            valid_c = sum(1 for f in frames if f["valid"])
            rate = (valid_c / len(frames)) * 100.0
            lats = [f["latency_ms"] for f in frames if f["latency_ms"] is not None]
            avg_lat = np.mean(lats) if lats else 0.0
            return {"count": len(frames), "valid": valid_c, "rate": rate, "avg_lat": avg_lat}

        stat_all  = get_stat(matched_results)

        # 高度分段统计: 每 5m 一个区间
        alt_bin_size = 5.0
        alt_bins = compute_altitude_bins(matched_results, alt_bin_size)

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
            "alt_bin_size": alt_bin_size,
            "alt_bins_5m": alt_bins,
            # 逐帧数据: 每条图像消息的识别结果 / tag 检出数量 / 飞行高度
            "frame_valid": [1 if r["valid"] else 0 for r in matched_results],
            "frame_tag_count": [r["tag_count"] if r["tag_count"] is not None else 0 for r in matched_results],
            "frame_alt": [r["alt"] for r in matched_results]
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
    # 2. 高度分段识别率 (每 5m 一个区间)
    bins_before = res_before["alt_bins_5m"]
    bins_after = res_after["alt_bins_5m"] if res_after else []

    def fmt_bin(bins, lo):
        for x in bins:
            if abs(x["lo"] - lo) < 0.01:
                if x["count"] == 0:
                    return "无数据"
                return f"{x['rate']:.2f} % ({x['valid']}/{x['count']})"
        return "无数据"

    if not bins_before:
        row("高度分段识别率", "无分段数据", "无分段数据" if res_after else None)
    else:
        for b in bins_before:
            row(f"{b['lo']:.0f}~{b['hi']:.0f}m 识别率",
                fmt_bin(bins_before, b["lo"]),
                fmt_bin(bins_after, b["lo"]) if res_after else None)

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
    parser.add_argument("--tag_count_topic", type=str, default=None,
                        help="Per-frame detected tag count topic (default: <target_pose_topic>_tag_count)")
    parser.add_argument("--preprocess", type=str, default="coarse_to_fine",
                        choices=["coarse_to_fine", "c2f", "c2f_upscale", "coarse_to_fine_upscale", "none", "sharpen", "clahe", "adaptive", "upscale_bicubic", "upscale_sharpen", "upscale_clahe"],
                        help="External image enhancement mode before sending to C++ pland_detector_node (coarse_to_fine, c2f, c2f_upscale, coarse_to_fine_upscale, none, sharpen, clahe, adaptive, upscale_bicubic, upscale_sharpen, upscale_clahe)")
    parser.add_argument("--rate", type=float, default=1.0, help="Playback speed multiplier (e.g. 1.0 or 2.0)")
    parser.add_argument("--duration", type=float, default=None, help="Max test duration in seconds")
    parser.add_argument("--label", type=str, default=None, help="Label for this test run")
    parser.add_argument("--compare_with_json", type=str, default=None, help="Path to baseline json report to compare against")
    parser.add_argument("--save_json", type=str, default=None, help="Save evaluation metrics to json file")
    parser.add_argument("--save_chart", type=str, default=None, help="识别率-高度统计图保存路径 (默认 altitude_rate_report.png, 或与 --save_json 同名)")
    parser.add_argument("--no_show", action="store_true", help="不弹出统计图窗口，仅保存图片文件")
    parser.add_argument("--no_launch", action="store_true", help="Do not launch node (if node is already running externally)")
    parser.add_argument("--disable_node_enhancement", action="store_true",
                        help="Disable node-internal C2F/CLAHE enhancement (disable_all_enhancement=true) for a true raw baseline")
    parser.add_argument("--enhance_mode", type=str, default=None,
                        choices=["none", "sharpen", "roi", "adaptive"],
                        help="Node enhance_mode injected into the launch (overrides pland_detector.yaml); external preprocessing is expected to be 'none' when used")

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
        preprocess_mode=args.preprocess,
        tag_count_topic=args.tag_count_topic,
        disable_node_enhancement=args.disable_node_enhancement,
        enhance_mode=args.enhance_mode
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

    # 弹出「识别率-高度」统计图窗口 (每 5m 分段)
    if args.save_chart:
        chart_path = args.save_chart
    elif args.save_json:
        chart_path = os.path.splitext(args.save_json)[0] + "_altitude_rate.png"
    else:
        chart_path = "altitude_rate_report.png"
    show_altitude_rate_chart(metrics, save_path=chart_path, show=not args.no_show)


if __name__ == "__main__":
    main()
