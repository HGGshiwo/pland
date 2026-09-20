#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import os
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

import cv2
import numpy as np
from PIL import Image, ImageTk

import rospy
from geometry_msgs.msg import Twist, TwistStamped, PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Image as RosImage
from std_msgs.msg import Empty, String, Float64
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode, StreamRate, MessageInterval
from mavros_msgs.msg import State as MavrosState

try:
    from cv_bridge import CvBridge
    bridge = CvBridge()
except ImportError:
    bridge = None


class SimControllerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Pland Simulation Controller")
        self.root.geometry("1060x820")
        self.root.minsize(960, 720)

        # ----------------- State Variables -----------------
        self.board_x = 0.0
        self.board_y = 0.0
        self.board_z = 0.0
        self.board_yaw = 0.0
        self.board_vx = 0.0
        self.board_vy = 0.0
        self.board_vz = 0.0

        self.drone_lat = 0.0
        self.drone_lon = 0.0
        self.drone_alt = 0.0
        self.drone_received_gps = False

        self.drone_connected = False
        self.drone_armed = False
        self.drone_mode = "UNKNOWN"

        self.pland_state = "UNKNOWN"
        self.target_pixel_u = 0.0
        self.target_pixel_v = 0.0
        self.has_target_pixel = False

        # Gimbal state (Default: 90.0 deg pitch down, body mode)
        self.gimbal_angle = 90.0
        self.gimbal_mode = "body"
        self.gimbal_current_pitch = None

        # Board motion
        self.motion_mode = "STOP"  # "STOP", "FORWARD", "BACKWARD", "SNAKE"
        self.motion_speed = 0.5    # m/s
        self.motion_variance = 0.0 # (m/s)^2
        self.snake_radius = 3.0    # 转弯半径 (m)
        self.snake_radian = 0.25   # 偏航弧度 (rad)
        self.snake_start_time = time.time()
        self.snake_base_yaw = 0.0

        # GPS injection
        self.auto_inject_gps = False
        self.calc_b_lat = 0.0
        self.calc_b_lon = 0.0
        self.calc_b_alt = 0.0
        self.calc_vel_e = 0.0
        self.calc_vel_n = 0.0
        self.calc_vel_u = 0.0

        # Camera frame
        self.latest_cv_img = None
        self.img_source_topic = "None"
        self.last_detect_time = 0.0

        self.lock = threading.Lock()

        # ----------------- ROS Node & Topics -----------------
        rospy.init_node('pland_sim_controller', anonymous=True)

        self.pub_board_cmd_vel = rospy.Publisher('/apriltag/cmd_vel', Twist, queue_size=1)
        self.pub_board_cmd_vel_backup = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.pub_pland_start = rospy.Publisher('/pland/start', Empty, queue_size=1)
        self.pub_pland_cancel = rospy.Publisher('/pland/cancel', Empty, queue_size=1)
        self.pub_pland_reset = rospy.Publisher('/pland/reset', Empty, queue_size=1)
        self.pub_inject_gps = rospy.Publisher('/pland/inject_target_pose', NavSatFix, queue_size=1)
        self.pub_inject_vel = rospy.Publisher('/pland/inject_target_vel', TwistStamped, queue_size=1)

        # Gimbal publishers
        self.pub_gimbal_angle = rospy.Publisher('/set_gimbal_angle', Float64, queue_size=1)
        self.pub_gimbal_mode = rospy.Publisher('/set_gimbal_angle_mode', String, queue_size=1)

        rospy.Subscriber('/pland/state', String, self.cb_pland_state)
        rospy.Subscriber('/pland/target_pixel', PointStamped, self.cb_target_pixel)
        rospy.Subscriber('/mavros/state', MavrosState, self.cb_mavros_state)
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.cb_drone_gps)
        rospy.Subscriber('/apriltag/odom', Odometry, self.cb_board_odom)
        rospy.Subscriber('/odom', Odometry, self.cb_board_odom)
        rospy.Subscriber('/gimbal/current_pitch', Float64, self.cb_gimbal_pitch)

        rospy.Subscriber('/pland/detect', RosImage, self.cb_detect_image, queue_size=1, buff_size=2**24)
        rospy.Subscriber('/roscam/cam/image_raw', RosImage, self.cb_raw_image, queue_size=1, buff_size=2**24)

        self.srv_get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.srv_set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.srv_set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.srv_arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.srv_takeoff = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

        # ----------------- Build UI -----------------
        self.build_ui()

        # ----------------- Worker Threads -----------------
        self.running = True
        self.motion_thread = threading.Thread(target=self.loop_board_motion, daemon=True)
        self.motion_thread.start()

        self.gps_thread = threading.Thread(target=self.loop_gps_calc_and_inject, daemon=True)
        self.gps_thread.start()

        # Send default gimbal settings and stream rates after ROS connections establish
        self.root.after(1000, self.init_gimbal_defaults)
        self.root.after(1200, lambda: self.setup_mavros_streams(20.0))
        self.root.after(50, self.refresh_ui)

    def init_gimbal_defaults(self):
        self.cmd_set_gimbal_mode("body")
        self.cmd_set_gimbal_angle(90.0)

    def build_ui(self):
        main_paned = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_paned.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

        # =================== Left Panel (Camera & Status) ===================
        left_frame = ttk.Frame(main_paned, width=450)
        main_paned.add(left_frame, weight=1)

        # 1. Camera live view
        cam_frame = ttk.LabelFrame(left_frame, text="Camera View", padding=4)
        cam_frame.pack(fill=tk.BOTH, expand=True, padx=4, pady=3)

        self.lbl_image = tk.Label(cam_frame, text="No Camera Image", bg="#202020", fg="#888888")
        self.lbl_image.pack(fill=tk.BOTH, expand=True)

        self.lbl_cam_source = ttk.Label(cam_frame, text="Source: None", font=("Arial", 9))
        self.lbl_cam_source.pack(anchor=tk.E, pady=1)

        # 2. System status
        status_frame = ttk.LabelFrame(left_frame, text="Telemetry & Status", padding=6)
        status_frame.pack(fill=tk.X, padx=4, pady=3)

        # FSM State
        f_state = ttk.Frame(status_frame)
        f_state.pack(fill=tk.X, pady=2)
        ttk.Label(f_state, text="Pland State:", font=("Arial", 10, "bold")).pack(side=tk.LEFT)
        self.lbl_fsm_state = tk.Label(f_state, text="UNKNOWN", font=("Arial", 10, "bold"),
                                      bg="#555555", fg="white", width=18)
        self.lbl_fsm_state.pack(side=tk.LEFT, padx=6)

        # Target Pixel
        f_pixel = ttk.Frame(status_frame)
        f_pixel.pack(fill=tk.X, pady=2)
        ttk.Label(f_pixel, text="Target Pixel (u, v):", font=("Arial", 9)).pack(side=tk.LEFT)
        self.lbl_pixel = ttk.Label(f_pixel, text="u: -- , v: --", font=("Arial", 9, "bold"))
        self.lbl_pixel.pack(side=tk.LEFT, padx=6)

        # Drone State
        f_drone = ttk.Frame(status_frame)
        f_drone.pack(fill=tk.X, pady=2)
        ttk.Label(f_drone, text="Drone Mode / Arm:", font=("Arial", 9)).pack(side=tk.LEFT)
        self.lbl_drone_state = ttk.Label(f_drone, text="-- / --", font=("Arial", 9))
        self.lbl_drone_state.pack(side=tk.LEFT, padx=6)

        # Gimbal Status
        f_g_status = ttk.Frame(status_frame)
        f_g_status.pack(fill=tk.X, pady=2)
        ttk.Label(f_g_status, text="Gimbal Mode / Pitch:", font=("Arial", 9)).pack(side=tk.LEFT)
        self.lbl_gimbal_status = ttk.Label(f_g_status, text="body | 90.0 deg", font=("Arial", 9))
        self.lbl_gimbal_status.pack(side=tk.LEFT, padx=6)

        # Board Pose
        f_board = ttk.Frame(status_frame)
        f_board.pack(fill=tk.X, pady=2)
        ttk.Label(f_board, text="Board Pose (Gazebo):", font=("Arial", 9)).pack(side=tk.LEFT)
        self.lbl_board_pose = ttk.Label(f_board, text="x=0.00, y=0.00, z=0.00", font=("Arial", 9))
        self.lbl_board_pose.pack(side=tk.LEFT, padx=6)

        # Event Log
        log_frame = ttk.LabelFrame(left_frame, text="Log", padding=4)
        log_frame.pack(fill=tk.X, padx=4, pady=3)
        self.txt_log = tk.Text(log_frame, height=5, state=tk.DISABLED, bg="#f5f5f5", font=("Arial", 9))
        self.txt_log.pack(fill=tk.BOTH, expand=True)

        # =================== Right Panel (Controls) ===================
        right_frame = ttk.Frame(main_paned, width=520)
        main_paned.add(right_frame, weight=1)

        # 1. Precision Landing Execution
        pland_frame = ttk.LabelFrame(right_frame, text="Precision Landing Control (/pland)", padding=6)
        pland_frame.pack(fill=tk.X, padx=4, pady=3)

        btn_pland_row = ttk.Frame(pland_frame)
        btn_pland_row.pack(fill=tk.X, pady=2)

        self.btn_start = tk.Button(btn_pland_row, text="Start Landing", font=("Arial", 10, "bold"),
                                   bg="#388E3C", fg="white", activebackground="#2E7D32",
                                   command=self.cmd_pland_start, width=14, height=1)
        self.btn_start.pack(side=tk.LEFT, padx=4, pady=2)

        self.btn_cancel = tk.Button(btn_pland_row, text="Cancel Landing", font=("Arial", 10, "bold"),
                                    bg="#D32F2F", fg="white", activebackground="#C62828",
                                    command=self.cmd_pland_cancel, width=14, height=1)
        self.btn_cancel.pack(side=tk.LEFT, padx=4, pady=2)

        self.btn_reset = tk.Button(btn_pland_row, text="Reset Tracker", font=("Arial", 9),
                                   bg="#F57C00", fg="white", activebackground="#EF6C00",
                                   command=self.cmd_pland_reset, width=12, height=1)
        self.btn_reset.pack(side=tk.LEFT, padx=4, pady=2)

        # 2. Drone Takeoff & Flight Control
        drone_frame = ttk.LabelFrame(right_frame, text="Drone Flight Control (MAVROS)", padding=6)
        drone_frame.pack(fill=tk.X, padx=4, pady=3)

        f_to_row = ttk.Frame(drone_frame)
        f_to_row.pack(fill=tk.X, pady=2)

        ttk.Label(f_to_row, text="Takeoff Alt (m):", font=("Arial", 9)).pack(side=tk.LEFT, padx=3)
        self.entry_takeoff_alt = ttk.Entry(f_to_row, width=6)
        self.entry_takeoff_alt.insert(0, "10.0")
        self.entry_takeoff_alt.pack(side=tk.LEFT, padx=3)

        self.btn_takeoff = tk.Button(f_to_row, text="Takeoff", font=("Arial", 9, "bold"),
                                     bg="#1976D2", fg="white", activebackground="#1565C0",
                                     command=self.cmd_drone_takeoff, width=10)
        self.btn_takeoff.pack(side=tk.LEFT, padx=8)

        f_drone_row2 = ttk.Frame(drone_frame)
        f_drone_row2.pack(fill=tk.X, pady=3)

        tk.Button(f_drone_row2, text="Arm", bg="#E0E0E0", command=lambda: self.cmd_drone_arm(True), width=8).pack(side=tk.LEFT, padx=3)
        tk.Button(f_drone_row2, text="Disarm", bg="#E0E0E0", command=lambda: self.cmd_drone_arm(False), width=8).pack(side=tk.LEFT, padx=3)
        tk.Button(f_drone_row2, text="Mode GUIDED", bg="#E0E0E0", command=lambda: self.cmd_drone_mode("GUIDED"), width=12).pack(side=tk.LEFT, padx=3)
        tk.Button(f_drone_row2, text="Mode LAND", bg="#E0E0E0", command=lambda: self.cmd_drone_mode("LAND"), width=10).pack(side=tk.LEFT, padx=3)
        tk.Button(f_drone_row2, text="Stream Rate", bg="#E0E0E0", command=lambda: self.setup_mavros_streams(20.0), width=11).pack(side=tk.LEFT, padx=3)

        # 3. Gimbal Control (/set_gimbal_angle, /set_gimbal_angle_mode)
        gimbal_frame = ttk.LabelFrame(right_frame, text="Gimbal Control (/set_gimbal_angle, mode)", padding=6)
        gimbal_frame.pack(fill=tk.X, padx=4, pady=3)

        f_g_mode = ttk.Frame(gimbal_frame)
        f_g_mode.pack(fill=tk.X, pady=2)

        ttk.Label(f_g_mode, text="Mode:", font=("Arial", 9)).pack(side=tk.LEFT, padx=3)
        self.btn_g_body = tk.Button(f_g_mode, text="body (Body Frame)", font=("Arial", 9, "bold"),
                                    bg="#00796B", fg="white", activebackground="#004D40",
                                    command=lambda: self.cmd_set_gimbal_mode("body"), width=16)
        self.btn_g_body.pack(side=tk.LEFT, padx=4)

        self.btn_g_abs = tk.Button(f_g_mode, text="abs (Earth Abs)", font=("Arial", 9),
                                   bg="#E0E0E0", fg="black", activebackground="#BDBDBD",
                                   command=lambda: self.cmd_set_gimbal_mode("abs"), width=14)
        self.btn_g_abs.pack(side=tk.LEFT, padx=4)

        f_g_angle = ttk.Frame(gimbal_frame)
        f_g_angle.pack(fill=tk.X, pady=3)

        ttk.Label(f_g_angle, text="Pitch Angle (deg):", font=("Arial", 9)).pack(side=tk.LEFT, padx=3)
        self.entry_gimbal_angle = ttk.Entry(f_g_angle, width=6)
        self.entry_gimbal_angle.insert(0, "90.0")
        self.entry_gimbal_angle.pack(side=tk.LEFT, padx=3)

        tk.Button(f_g_angle, text="Set Angle", font=("Arial", 9, "bold"),
                  bg="#1976D2", fg="white", activebackground="#1565C0",
                  command=self.cmd_set_gimbal_angle, width=9).pack(side=tk.LEFT, padx=4)

        tk.Button(f_g_angle, text="90 deg", bg="#E0E0E0", command=lambda: self.cmd_set_gimbal_angle(90.0), width=6).pack(side=tk.LEFT, padx=2)
        tk.Button(f_g_angle, text="45 deg", bg="#E0E0E0", command=lambda: self.cmd_set_gimbal_angle(45.0), width=6).pack(side=tk.LEFT, padx=2)
        tk.Button(f_g_angle, text="0 deg", bg="#E0E0E0", command=lambda: self.cmd_set_gimbal_angle(0.0), width=6).pack(side=tk.LEFT, padx=2)

        # 4. Board Height Control (Input height, default 0.8)
        height_frame = ttk.LabelFrame(right_frame, text="Board Height Control (Gazebo: apriltag)", padding=6)
        height_frame.pack(fill=tk.X, padx=4, pady=3)

        f_h_row = ttk.Frame(height_frame)
        f_h_row.pack(fill=tk.X, pady=2)

        ttk.Label(f_h_row, text="Target Height (m):", font=("Arial", 9)).pack(side=tk.LEFT, padx=3)
        self.entry_board_height = ttk.Entry(f_h_row, width=8)
        self.entry_board_height.insert(0, "0.8")
        self.entry_board_height.pack(side=tk.LEFT, padx=4)

        self.btn_set_height = tk.Button(f_h_row, text="Set Height", font=("Arial", 9, "bold"),
                                        bg="#00796B", fg="white", activebackground="#004D40",
                                        command=self.cmd_apply_board_height, width=12)
        self.btn_set_height.pack(side=tk.LEFT, padx=6)

        # 5. Board Motion Control (Straight / Stop, with Speed & Variance, Snake Yaw)
        motion_frame = ttk.LabelFrame(right_frame, text="Board Motion Control", padding=6)
        motion_frame.pack(fill=tk.X, padx=4, pady=3)

        f_param_row1 = ttk.Frame(motion_frame)
        f_param_row1.pack(fill=tk.X, pady=2)

        ttk.Label(f_param_row1, text="Base Speed (m/s):", font=("Arial", 9)).pack(side=tk.LEFT, padx=3)
        self.entry_speed = ttk.Entry(f_param_row1, width=6)
        self.entry_speed.insert(0, "0.5")
        self.entry_speed.pack(side=tk.LEFT, padx=3)

        ttk.Label(f_param_row1, text="Speed Variance:", font=("Arial", 9)).pack(side=tk.LEFT, padx=8)
        self.entry_variance = ttk.Entry(f_param_row1, width=6)
        self.entry_variance.insert(0, "0.0")
        self.entry_variance.pack(side=tk.LEFT, padx=3)

        f_param_row2 = ttk.Frame(motion_frame)
        f_param_row2.pack(fill=tk.X, pady=2)

        ttk.Label(f_param_row2, text="Radius (m):", font=("Arial", 9)).pack(side=tk.LEFT, padx=3)
        self.entry_snake_radius = ttk.Entry(f_param_row2, width=6)
        self.entry_snake_radius.insert(0, "3.0")
        self.entry_snake_radius.pack(side=tk.LEFT, padx=3)

        ttk.Label(f_param_row2, text="Radian (rad):", font=("Arial", 9)).pack(side=tk.LEFT, padx=8)
        self.entry_snake_radian = ttk.Entry(f_param_row2, width=6)
        self.entry_snake_radian.insert(0, "0.25")
        self.entry_snake_radian.pack(side=tk.LEFT, padx=3)

        self.lbl_radian_deg = ttk.Label(f_param_row2, text="≈ 14.3°", font=("Arial", 9), foreground="#555555")
        self.lbl_radian_deg.pack(side=tk.LEFT, padx=3)

        ttk.Button(f_param_row2, text="Apply Parameters", command=self.cmd_apply_motion_params).pack(side=tk.LEFT, padx=8)

        f_m_btns = ttk.Frame(motion_frame)
        f_m_btns.pack(fill=tk.X, pady=4)

        self.btn_m_stop = tk.Button(f_m_btns, text="Stop", font=("Arial", 9, "bold"),
                                    bg="#455A64", fg="white", activebackground="#37474F",
                                    command=lambda: self.set_motion_mode("STOP"), width=10)
        self.btn_m_stop.pack(side=tk.LEFT, padx=3)

        self.btn_m_fwd = tk.Button(f_m_btns, text="Forward", font=("Arial", 9),
                                   bg="#E0E0E0", command=lambda: self.set_motion_mode("FORWARD"), width=10)
        self.btn_m_fwd.pack(side=tk.LEFT, padx=3)

        self.btn_m_back = tk.Button(f_m_btns, text="Backward", font=("Arial", 9),
                                    bg="#E0E0E0", command=lambda: self.set_motion_mode("BACKWARD"), width=10)
        self.btn_m_back.pack(side=tk.LEFT, padx=3)

        self.btn_m_snake = tk.Button(f_m_btns, text="Snake", font=("Arial", 9),
                                     bg="#E0E0E0", command=lambda: self.set_motion_mode("SNAKE"), width=14)
        self.btn_m_snake.pack(side=tk.LEFT, padx=3)

        # 6. GPS Injection Control (Continuous Injection Switch Only)
        gps_frame = ttk.LabelFrame(right_frame, text="Target GPS Injection (/pland/inject_target_pose)", padding=6)
        gps_frame.pack(fill=tk.X, padx=4, pady=3)

        f_gps_info = ttk.Frame(gps_frame)
        f_gps_info.pack(fill=tk.X, pady=2)
        self.lbl_calc_gps = ttk.Label(f_gps_info, text="Target GPS: Lat=0.000000, Lon=0.000000, Alt=0.00m", font=("Arial", 9))
        self.lbl_calc_gps.pack(anchor=tk.W)

        self.lbl_calc_vel = ttk.Label(f_gps_info, text="Target Velocity (ENU): Ve=0.00, Vn=0.00, Vu=0.00 m/s", font=("Arial", 9))
        self.lbl_calc_vel.pack(anchor=tk.W)

        f_gps_switch = ttk.Frame(gps_frame)
        f_gps_switch.pack(fill=tk.X, pady=4)

        self.chk_var_auto_inject = tk.BooleanVar(value=False)
        self.chk_auto_inject = ttk.Checkbutton(f_gps_switch, text="Continuous GPS Injection (10Hz)",
                                               variable=self.chk_var_auto_inject,
                                               command=self.on_toggle_auto_inject)
        self.chk_auto_inject.pack(side=tk.LEFT, padx=4)

    # ========================== Log ==========================
    def log(self, text):
        timestamp = time.strftime("%H:%M:%S")
        msg = f"[{timestamp}] {text}\n"
        self.txt_log.config(state=tk.NORMAL)
        self.txt_log.insert(tk.END, msg)
        self.txt_log.see(tk.END)
        self.txt_log.config(state=tk.DISABLED)
        rospy.loginfo(text)

    # ========================== ROS Callbacks ==========================
    def cb_pland_state(self, msg):
        self.pland_state = msg.data

    def cb_target_pixel(self, msg):
        self.target_pixel_u = msg.point.x
        self.target_pixel_v = msg.point.y
        self.has_target_pixel = True

    def cb_mavros_state(self, msg):
        was_connected = self.drone_connected
        self.drone_connected = msg.connected
        self.drone_armed = msg.armed
        self.drone_mode = msg.mode

        if msg.connected and not was_connected:
            self.log("FCU connected. Requesting MAVROS stream rates and message intervals...")
            self.setup_mavros_streams(20.0)

    def cb_drone_gps(self, msg):
        self.drone_lat = msg.latitude
        self.drone_lon = msg.longitude
        self.drone_alt = msg.altitude
        self.drone_received_gps = True

    def cb_board_odom(self, msg):
        self.board_x = msg.pose.pose.position.x
        self.board_y = msg.pose.pose.position.y
        self.board_z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.board_yaw = math.atan2(siny_cosp, cosy_cosp)

    def cb_gimbal_pitch(self, msg):
        self.gimbal_current_pitch = msg.data

    def cb_detect_image(self, msg):
        try:
            if bridge:
                cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            else:
                np_arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
                cv_img = np_arr if msg.encoding == "bgr8" else cv2.cvtColor(np_arr, cv2.COLOR_RGB2BGR)
            with self.lock:
                self.latest_cv_img = cv_img
                self.img_source_topic = "/pland/detect"
                self.last_detect_time = time.time()
        except Exception:
            pass

    def cb_raw_image(self, msg):
        if time.time() - self.last_detect_time < 0.8:
            return
        try:
            if bridge:
                cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            else:
                np_arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, -1))
                cv_img = np_arr if msg.encoding == "bgr8" else cv2.cvtColor(np_arr, cv2.COLOR_RGB2BGR)
            with self.lock:
                self.latest_cv_img = cv_img
                self.img_source_topic = "/roscam/cam/image_raw"
        except Exception:
            pass

    # ========================== Action Handlers ==========================
    # 0. Gimbal Control
    def cmd_set_gimbal_angle(self, angle=None):
        if angle is None:
            try:
                angle = float(self.entry_gimbal_angle.get())
            except ValueError:
                self.log(f"Invalid gimbal angle input: '{self.entry_gimbal_angle.get()}'")
                return
        else:
            self.entry_gimbal_angle.delete(0, tk.END)
            self.entry_gimbal_angle.insert(0, str(angle))

        self.gimbal_angle = float(angle)
        msg = Float64(data=self.gimbal_angle)
        self.pub_gimbal_angle.publish(msg)
        self.log(f"Published /set_gimbal_angle: {self.gimbal_angle:.1f} deg")

    def cmd_set_gimbal_mode(self, mode):
        self.gimbal_mode = mode
        msg = String(data=mode)
        self.pub_gimbal_mode.publish(msg)
        self.log(f"Published /set_gimbal_angle_mode: '{mode}'")

        if mode == "body":
            self.btn_g_body.config(bg="#00796B", fg="white")
            self.btn_g_abs.config(bg="#E0E0E0", fg="black")
        else:
            self.btn_g_abs.config(bg="#00796B", fg="white")
            self.btn_g_body.config(bg="#E0E0E0", fg="black")

    # 1. Height
    def cmd_apply_board_height(self):
        try:
            target_z = float(self.entry_board_height.get())
        except ValueError:
            self.log(f"Invalid height value: '{self.entry_board_height.get()}'")
            return

        def _worker():
            try:
                res = self.srv_get_model_state('apriltag', 'world')
                state_msg = ModelState()
                state_msg.model_name = 'apriltag'
                state_msg.reference_frame = 'world'
                if res.success:
                    state_msg.pose = res.pose
                    state_msg.twist = res.twist
                else:
                    state_msg.pose.orientation.w = 1.0

                state_msg.pose.position.z = target_z
                state_msg.twist.linear.z = 0.0
                state_msg.twist.angular.x = 0.0
                state_msg.twist.angular.y = 0.0

                resp = self.srv_set_model_state(state_msg)
                if resp.success:
                    self.log(f"Board height set to {target_z:.3f} m")
                else:
                    self.log(f"Failed to set board height: {resp.status_message}")
            except Exception as e:
                self.log(f"Set board height exception: {e}")

        threading.Thread(target=_worker, daemon=True).start()

    # 2. Motion Parameters & Loop
    def cmd_apply_motion_params(self):
        try:
            spd = float(self.entry_speed.get())
            var = float(self.entry_variance.get())
            radius = float(self.entry_snake_radius.get())
            radian = float(self.entry_snake_radian.get())
            if var < 0:
                var = 0.0
            if radius <= 0.1:
                radius = 0.1
            if radian <= 0.001:
                radian = 0.001

            self.motion_speed = spd
            self.motion_variance = var
            self.snake_radius = radius
            self.snake_radian = radian

            deg = math.degrees(radian)
            self.lbl_radian_deg.config(text=f"≈ {deg:.1f}°")
            self.log(f"Updated motion params: speed={self.motion_speed:.2f} m/s, variance={self.motion_variance:.4f}, "
                     f"Radius={self.snake_radius:.2f} m, Radian={self.snake_radian:.3f} rad ({deg:.1f}°)")
        except ValueError:
            self.log("Invalid parameter input (speed, variance, radius, or radian)")

    def set_motion_mode(self, mode):
        self.cmd_apply_motion_params()
        self.motion_mode = mode
        if mode == "SNAKE":
            self.snake_start_time = time.time()
            self.snake_base_yaw = self.board_yaw

        modes = {
            "STOP": self.btn_m_stop,
            "FORWARD": self.btn_m_fwd,
            "BACKWARD": self.btn_m_back,
            "SNAKE": self.btn_m_snake
        }
        for m, btn in modes.items():
            if m == mode:
                btn.config(bg="#388E3C", fg="white")
            else:
                btn.config(bg="#E0E0E0" if m != "STOP" else "#455A64", fg="black" if m != "STOP" else "white")

        self.log(f"Motion mode: {mode}")

    def loop_board_motion(self):
        rate = rospy.Rate(30)
        while self.running and not rospy.is_shutdown():
            cmd = Twist()
            mode = self.motion_mode
            base_spd = self.motion_speed

            cur_spd = base_spd
            if self.motion_variance > 0:
                noise = np.random.normal(0, math.sqrt(self.motion_variance))
                cur_spd += noise

            if mode == "STOP":
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.angular.z = 0.0
            elif mode == "FORWARD":
                cmd.linear.x = cur_spd
                cmd.angular.z = 0.0
            elif mode == "BACKWARD":
                cmd.linear.x = -cur_spd
                cmd.angular.z = 0.0
            elif mode == "SNAKE":
                cmd.linear.x = cur_spd

                R = max(self.snake_radius, 0.1)
                theta_max = max(self.snake_radian, 0.005)
                base_v = max(abs(self.motion_speed), 0.05)

                omega_0 = base_v / R
                t1 = theta_max / omega_0
                t_half = 2.0 * theta_max / omega_0
                T_period = 2.0 * t_half

                tau = (time.time() - self.snake_start_time) % T_period

                if tau < t1:
                    delta_psi = omega_0 * tau
                    omega_ff = omega_0
                elif tau < t1 + t_half:
                    delta_psi = theta_max - omega_0 * (tau - t1)
                    omega_ff = -omega_0
                else:
                    delta_psi = -theta_max + omega_0 * (tau - (t1 + t_half))
                    omega_ff = omega_0

                target_yaw = self.snake_base_yaw + delta_psi
                yaw_err = math.atan2(math.sin(target_yaw - self.board_yaw),
                                     math.cos(target_yaw - self.board_yaw))

                kp_yaw = 2.0
                omega_cmd = omega_ff + kp_yaw * yaw_err

                max_omega = max(2.5 * omega_0, 1.5)
                cmd.angular.z = max(-max_omega, min(max_omega, omega_cmd))

            self.pub_board_cmd_vel.publish(cmd)
            self.pub_board_cmd_vel_backup.publish(cmd)
            rate.sleep()

    # 3. Takeoff & Flight Control
    def cmd_drone_takeoff(self):
        try:
            alt = float(self.entry_takeoff_alt.get())
        except ValueError:
            self.log("Invalid takeoff altitude value")
            return

        def _takeoff_worker():
            self.log(f"Starting takeoff sequence to {alt:.1f} m ...")
            try:
                self.log("Requesting GUIDED mode...")
                res_m = self.srv_set_mode(custom_mode="GUIDED")
                if not res_m.mode_sent:
                    self.log("GUIDED mode request not acknowledged, proceeding...")
                time.sleep(0.5)

                self.log("Requesting Arming...")
                res_a = self.srv_arming(value=True)
                if not res_a.success:
                    self.log("Arming failed! Check prearm status.")
                time.sleep(0.5)

                self.log(f"Sending takeoff command (alt={alt} m)...")
                res_t = self.srv_takeoff(altitude=alt)
                if res_t.success:
                    self.log("Takeoff command accepted by FCU")
                else:
                    self.log("Takeoff command rejected")
            except Exception as e:
                self.log(f"Takeoff exception: {e}")

        threading.Thread(target=_takeoff_worker, daemon=True).start()

    def cmd_drone_arm(self, arm_state):
        def _worker():
            action_name = "Arm" if arm_state else "Disarm"
            try:
                res = self.srv_arming(value=arm_state)
                if res.success:
                    self.log(f"{action_name} succeeded")
                else:
                    self.log(f"{action_name} failed")
            except Exception as e:
                self.log(f"{action_name} error: {e}")
        threading.Thread(target=_worker, daemon=True).start()

    def cmd_drone_mode(self, mode_name):
        def _worker():
            try:
                res = self.srv_set_mode(custom_mode=mode_name)
                if res.mode_sent:
                    self.log(f"Mode set to {mode_name}")
                else:
                    self.log(f"Failed to set mode {mode_name}")
            except Exception as e:
                self.log(f"Set mode error: {e}")
        threading.Thread(target=_worker, daemon=True).start()

    def setup_mavros_streams(self, rate=20.0):
        def _worker():
            self.log(f"Setting MAVROS stream rate to {rate:.1f} Hz...")
            # 1. /mavros/set_stream_rate (stream_id=0 for all streams)
            try:
                rospy.wait_for_service('/mavros/set_stream_rate', timeout=5.0)
                client_stream = rospy.ServiceProxy('/mavros/set_stream_rate', StreamRate)
                res = client_stream(stream_id=0, message_rate=int(rate), on_off=True)
                self.log(f"Successfully requested all MAVROS streams at {rate:.1f} Hz via /mavros/set_stream_rate.")
            except Exception as e:
                self.log(f"Service /mavros/set_stream_rate call failed: {e}")

            # 2. /mavros/set_message_interval (ATTITUDE: 30, LOCAL_POSITION_NED: 32, DISTANCE_SENSOR: 132)
            try:
                rospy.wait_for_service('/mavros/set_message_interval', timeout=3.0)
                client_interval = rospy.ServiceProxy('/mavros/set_message_interval', MessageInterval)
                for msg_id in [30, 32, 132]:
                    try:
                        res_int = client_interval(message_id=msg_id, message_rate=float(rate))
                        if res_int.success:
                            rospy.loginfo(f"Successfully set message interval for message ID {msg_id} to {rate:.1f} Hz.")
                    except Exception:
                        pass
            except Exception:
                pass

        threading.Thread(target=_worker, daemon=True).start()

    # 4. Precision Landing Commands
    def cmd_pland_start(self):
        self.pub_pland_start.publish(Empty())
        self.log("Published /pland/start")

    def cmd_pland_cancel(self):
        self.pub_pland_cancel.publish(Empty())
        self.log("Published /pland/cancel")

    def cmd_pland_reset(self):
        self.pub_pland_reset.publish(Empty())
        self.log("Published /pland/reset")

    # 5. GPS Injection
    def on_toggle_auto_inject(self):
        self.auto_inject_gps = self.chk_var_auto_inject.get()
        state_str = "ENABLED" if self.auto_inject_gps else "DISABLED"
        self.log(f"Continuous GPS injection {state_str}")

    def loop_gps_calc_and_inject(self):
        rate = rospy.Rate(10)
        while self.running and not rospy.is_shutdown():
            try:
                res_b = self.srv_get_model_state('apriltag', 'world')
                res_r = self.srv_get_model_state('iris_demo', 'world')
                if not res_r.success:
                    res_r = self.srv_get_model_state('iris', 'world')

                if res_b.success:
                    b_x = res_b.pose.position.x
                    b_y = res_b.pose.position.y
                    b_z = res_b.pose.position.z
                    vx = res_b.twist.linear.x
                    vy = res_b.twist.linear.y
                    vz = res_b.twist.linear.z

                    self.board_x = b_x
                    self.board_y = b_y
                    self.board_z = b_z

                    q = res_b.pose.orientation
                    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                    self.board_yaw = math.atan2(siny_cosp, cosy_cosp)

                    if res_r.success and self.drone_received_gps:
                        r_x = res_r.pose.position.x
                        r_y = res_r.pose.position.y
                        r_z = res_r.pose.position.z

                        d_north = b_x - r_x
                        d_east = r_y - b_y
                        d_up = b_z - r_z

                        R_earth = 6378137.0
                        d_lat = d_north / R_earth
                        cos_lat = math.cos(math.radians(self.drone_lat))
                        d_lon = d_east / (R_earth * max(cos_lat, 1e-6))
                        b_lat = self.drone_lat + math.degrees(d_lat)
                        b_lon = self.drone_lon + math.degrees(d_lon)
                        b_alt = self.drone_alt + d_up

                        vel_east = -vy
                        vel_north = vx
                        vel_up = vz

                        self.calc_b_lat = b_lat
                        self.calc_b_lon = b_lon
                        self.calc_b_alt = b_alt
                        self.calc_vel_e = vel_east
                        self.calc_vel_n = vel_north
                        self.calc_vel_u = vel_up

                        if self.auto_inject_gps:
                            self.do_inject_gps()
            except Exception as e:
                rospy.logwarn_throttle(2.0, f"GPS calc and inject error: {e}")
            rate.sleep()

    def do_inject_gps(self):
        if self.calc_b_lat == 0.0 and self.calc_b_lon == 0.0:
            return
        gps_msg = NavSatFix()
        gps_msg.header.stamp = rospy.Time.now()
        gps_msg.header.frame_id = "world"
        gps_msg.latitude = self.calc_b_lat
        gps_msg.longitude = self.calc_b_lon
        gps_msg.altitude = self.calc_b_alt
        self.pub_inject_gps.publish(gps_msg)

        vel_msg = TwistStamped()
        vel_msg.header.stamp = rospy.Time.now()
        vel_msg.header.frame_id = "map"
        vel_msg.twist.linear.x = self.calc_vel_e
        vel_msg.twist.linear.y = self.calc_vel_n
        vel_msg.twist.linear.z = self.calc_vel_u
        self.pub_inject_vel.publish(vel_msg)

    # ========================== Periodic Refresh ==========================
    def refresh_ui(self):
        # 1. State machine label
        state_colors = {
            "IDLE": ("#9E9E9E", "white"),
            "TRACING_GPS": ("#FF9800", "black"),
            "TRACING_DETECTOR": ("#388E3C", "white"),
            "BLIND_DROP": ("#1976D2", "white"),
            "TARGET_LOST": ("#D32F2F", "white"),
            "LANDED": ("#00796B", "white")
        }
        bg_col, fg_col = state_colors.get(self.pland_state, ("#555555", "white"))
        self.lbl_fsm_state.config(text=self.pland_state, bg=bg_col, fg=fg_col)

        # 2. Pixel coords
        if self.has_target_pixel:
            self.lbl_pixel.config(text=f"u: {self.target_pixel_u:.3f} , v: {self.target_pixel_v:.3f}")
        else:
            self.lbl_pixel.config(text="No target detected")

        # 3. Drone status
        conn_text = "CONN" if self.drone_connected else "DISC"
        arm_text = "ARMED" if self.drone_armed else "DISARMED"
        self.lbl_drone_state.config(text=f"{self.drone_mode}  |  {arm_text}  |  {conn_text}")

        # 4. Gimbal status
        cur_pitch_str = f"{self.gimbal_current_pitch:.1f} deg" if self.gimbal_current_pitch is not None else f"cmd: {self.gimbal_angle:.1f} deg"
        self.lbl_gimbal_status.config(text=f"{self.gimbal_mode} | {cur_pitch_str}")

        # 5. Board Pose
        self.lbl_board_pose.config(text=f"x={self.board_x:.2f}, y={self.board_y:.2f}, z={self.board_z:.2f}, yaw={math.degrees(self.board_yaw):.1f}°")

        # 6. Calculated target GPS
        self.lbl_calc_gps.config(text=f"Target GPS: Lat={self.calc_b_lat:.6f}, Lon={self.calc_b_lon:.6f}, Alt={self.calc_b_alt:.2f} m")
        self.lbl_calc_vel.config(text=f"Target Velocity (ENU): Ve={self.calc_vel_e:.2f}, Vn={self.calc_vel_n:.2f}, Vu={self.calc_vel_u:.2f} m/s")

        # 7. Render camera frame
        with self.lock:
            img = self.latest_cv_img
            src = self.img_source_topic

        if img is not None:
            try:
                h, w = img.shape[:2]
                disp_w = 430
                disp_h = int(disp_w * h / w)
                resized = cv2.resize(img, (disp_w, disp_h))
                rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
                pil_img = Image.fromarray(rgb)
                tk_img = ImageTk.PhotoImage(image=pil_img)
                self.lbl_image.config(image=tk_img, text="")
                self.lbl_image.image = tk_img
                self.lbl_cam_source.config(text=f"Source: {src} ({w}x{h})")
            except Exception:
                pass

        if self.running:
            self.root.after(40, self.refresh_ui)

    def on_closing(self):
        self.running = False
        self.root.destroy()


def main():
    root = tk.Tk()
    app = SimControllerGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == '__main__':
    main()