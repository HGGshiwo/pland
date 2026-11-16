#!/usr/bin/env python
# coding=utf-8
"""融合高度测试的数据注入器: 20Hz 周期发布四路传感器输入,
值可动态修改, rel_alt / rangefinder 置 None 即停发该通道 (模拟断流)。

注入通道 (与 pland_detector_node 的订阅一致):
  - /mavros/local_position/odom          odom z (最终兜底)
  - /mavros/global_position/rel_alt      rel_alt (Float64)
  - /mavros/distance_sensor/rangefinder_pub  测距仪 (Range)
  - /roscam/cam/image_raw                小黑图 (仅驱动 detect 流水线)
"""
import threading
import time

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, NavSatFix, NavSatStatus, Range
from std_msgs.msg import Empty, Float64
from geometry_msgs.msg import TwistStamped


class SensorInjector:
    def __init__(self, rate_hz=20.0):
        self._lock = threading.Lock()
        self._stop_flag = False
        self.odom_z = 0.0   # odom 系高度 (米)
        self.rel_alt = None  # None = 停发该通道
        self.rf_range = None
        # 测距仪消息静态字段 (节点有效性判定: 0.01 < range <= max_range)
        self.rf_max_range = 12.0

        self.pub_odom = rospy.Publisher(
            "/mavros/local_position/odom", Odometry, queue_size=1)
        self.pub_rel = rospy.Publisher(
            "/mavros/global_position/rel_alt", Float64, queue_size=1)
        self.pub_rf = rospy.Publisher(
            "/mavros/distance_sensor/rangefinder_pub", Range, queue_size=1)
        self.pub_img = rospy.Publisher(
            "/roscam/cam/image_raw", Image, queue_size=1)
        # 控制器测试扩展通道
        self.pub_gps = rospy.Publisher(
            "/mavros/global_position/global", NavSatFix, queue_size=1)
        self.pub_start = rospy.Publisher("/pland/start", Empty, queue_size=1)
        self.pub_inject = rospy.Publisher(
            "/pland/inject_target_pose", NavSatFix, queue_size=1)
        self.gps_lat = 30.1405937
        self.gps_lon = 120.0843665
        self.gps_alt = 10.15

        img = np.zeros((8, 8, 3), dtype=np.uint8)
        self._img_msg = Image()
        self._img_msg.height = 8
        self._img_msg.width = 8
        self._img_msg.encoding = "bgr8"
        self._img_msg.is_bigendian = 0
        self._img_msg.step = 8 * 3
        self._img_msg.data = img.tobytes()

        self._rate = rospy.Rate(rate_hz)
        self._thread = threading.Thread(target=self._spin, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop_flag = True
        self._thread.join(timeout=2.0)

    def set(self, odom_z=None, rel_alt=None, rf_range=None):
        """动态修改注入值; rel_alt / rf_range 传 None 表示停发该通道。"""
        with self._lock:
            if odom_z is not None:
                self.odom_z = odom_z
            if rel_alt is not None:
                self.rel_alt = rel_alt
            if rf_range is not None:
                self.rf_range = rf_range
            if rel_alt is None and "rel_set" in dir(self):
                pass

    def stop_rel_alt(self):
        with self._lock:
            self.rel_alt = None

    def stop_rf(self):
        with self._lock:
            self.rf_range = None

    def send_start(self):
        self.pub_start.publish(Empty())

    def send_inject_target(self, lat, lon, alt=0.0):
        """单次发布 GPS 注入目标 (控制器 target_timeout=1s, 需 >1Hz 持续注入)。"""
        m = NavSatFix()
        m.header.stamp = rospy.Time.now()
        m.status.status = NavSatStatus.STATUS_FIX
        m.latitude = lat
        m.longitude = lon
        m.altitude = alt
        self.pub_inject.publish(m)

    def _spin(self):
        while not self._stop_flag and not rospy.is_shutdown():
            now = rospy.Time.now()
            with self._lock:
                odom_z, rel_alt, rf_range = (
                    self.odom_z, self.rel_alt, self.rf_range)

            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = "map"
            odom.pose.pose.position.z = odom_z
            odom.pose.pose.orientation.w = 1.0
            self.pub_odom.publish(odom)

            if rel_alt is not None:
                m = Float64()
                m.data = rel_alt
                self.pub_rel.publish(m)

            if rf_range is not None:
                r = Range()
                r.header.stamp = now
                r.header.frame_id = "rangefinder"
                r.radiation_type = Range.INFRARED
                r.field_of_view = 0.1
                r.min_range = 0.01
                r.max_range = self.rf_max_range
                r.range = rf_range
                self.pub_rf.publish(r)

            img = self._img_msg
            img.header.stamp = now
            img.header.frame_id = "camera"
            self.pub_img.publish(img)

            # 控制器测试: 机载 GPS (与 odom 同步发布)
            with self._lock:
                gps_lat, gps_lon, gps_alt = (
                    self.gps_lat, self.gps_lon, self.gps_alt)
            gps = NavSatFix()
            gps.header.stamp = now
            gps.status.status = NavSatStatus.STATUS_FIX
            gps.latitude = gps_lat
            gps.longitude = gps_lon
            gps.altitude = gps_alt
            self.pub_gps.publish(gps)

            self._rate.sleep()


class ZWatcher:
    """收集 /pland/current_z 输出, 提供最新值与清屏。"""

    def __init__(self):
        self.latest = None
        self.latest_stamp = None
        self.sub = rospy.Subscriber(
            "/pland/current_z", Float64, self._cb, queue_size=10)

    def _cb(self, msg):
        self.latest = msg.data
        self.latest_stamp = rospy.Time.now().to_sec()

    def clear(self):
        self.latest = None

    def get(self, wait=5.0):
        """等待新样本并返回最新值, 超时抛错。"""
        deadline = time.time() + wait
        stamp0 = self.latest_stamp
        while time.time() < deadline:
            if self.latest is not None and self.latest_stamp != stamp0:
                return self.latest
            time.sleep(0.1)
        raise AssertionError("超时未收到 /pland/current_z 新样本")
