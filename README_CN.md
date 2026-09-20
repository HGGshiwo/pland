# Pland: 高精度无人机自主降落系统

<p align="center">
  <a href="README.md">English</a> |
  <a href="README_CN.md"><b>简体中文</b></a>
</p>

<p align="center">
  <a href="https://ros.org/"><img src="https://img.shields.io/badge/ROS-Noetic-blue.svg" alt="ROS Noetic"></a>
  <a href="https://ubuntu.com/"><img src="https://img.shields.io/badge/Ubuntu-20.04-orange.svg" alt="Ubuntu 20.04"></a>
  <img src="https://img.shields.io/badge/C%2B%2B-17-green.svg" alt="C++17">
  <img src="https://img.shields.io/badge/Python-3.8-yellow.svg" alt="Python 3.8">
  <a href="LICENSE"><img src="https://img.shields.io/badge/License-Apache%202.0-lightgrey.svg" alt="License"></a>
</p>

**Pland** 是一套面向多旋翼无人机的开源高精度自主着陆系统，支持静止地面标靶与动态移动平台的自主对接与平稳触地。

---

## 1. 核心特性 (Features)

- **机身体系 (FLU) 纯视觉闭环**：目标三维相对位姿直接解算在飞机机身坐标系（`base_link`，前-左-上），消除全局大地坐标系转换的时延与坐标漂移。
- **多尺度嵌套 AprilTag 阵列**：高空依靠大尺寸 Tag 远距离捕获，低空近地平滑切换至中心小 Tag，防止标靶超出视野导致失锁。
- **CTRV 扩展卡尔曼滤波 (EKF)**：基于二维恒定转弯率与速度模型实时估计目标合速度、航向角与角速度，为控制器提供平滑前馈速度补偿。
- **自适应降落漏斗与增益调度**：对齐容差随高度降低呈漏斗状线性收敛（$r_{\text{tol}} = r_{\min} + k \cdot z$），近地阶段控制增益自适应平滑衰减，有效抑制地面效应气流干扰。
- **六状态鲁棒状态机 (FSM)**：统一调度 `IDLE`、`TRACING_GPS`、`TRACING_DETECTOR`、`BLIND_DROP`、`TARGET_LOST` 与 `LANDED` 状态，支持 GPS 到视觉的无缝平滑衔接。

---

## 2. 支持的三大降落场景 (Supported Scenarios)

| 场景 | 对应模式 | 说明 |
| :--- | :--- | :--- |
| **1. 静止目标 (Stationary)** | `STOP` | 针对静止标靶的高精度悬停对齐、漏斗收敛与末端触地盲降。 |
| **2. 直线运动目标 (Straight-Line)** | `FORWARD` / `BACKWARD` | 跟踪直线恒速或加减速运动目标，通过 CTRV 速度前馈补偿消除跟随滞后。 |
| **3. 蛇形/变道机动目标 (Snake Maneuver)** | `SNAKE` | 模拟车辆在道路行驶中的拐弯与变道机动，平板车身**沿切线方向自转偏航**并沿圆弧运动（转弯半径满足 $R = v / \|\omega\|$），支持在界面动态调节**旋转半径 ($R$)** 与**旋转弧度 ($\theta$)**。 |

---

## 3. 环境依赖与编译 (Installation)

### 环境要求
- Ubuntu 20.04 LTS & ROS Noetic
- 基础依赖库安装：
  ```bash
  sudo apt-get update
  sudo apt-get install -y \
      ros-noetic-mavros \
      ros-noetic-mavros-extras \
      ros-noetic-cv-bridge \
      ros-noetic-geographic-msgs \
      libgeographic-dev \
      libopencv-dev \
      libeigen3-dev \
      python3-tk
  ```

### 源码编译
```bash
cd ~/catkin_ws/src
git clone https://github.com/hggshiwo/pland.git
catkin_make -DCMAKE_BUILD_TYPE=Release
source ~/catkin_ws/devel/setup.bash
```

---

## 4. 快速使用 (Quick Start)

### 1. 一键启动仿真 (Gazebo + ArduPilot SITL + GUI)
```bash
roslaunch pland_sim pland_sim.launch
```

### 2. 交互式控制台 GUI
仿真启动后将弹出图形化交互面板：
- **无人机控制**：一键起飞（指定高度）、解锁/加锁、GUIDED/LAND 模式切换。
- **目标运动控制**：切换 `Stop`（静止场景）、`Forward`（直线运动）、或 `Snake`（蛇形/变道机动，支持自定义半径与弧度）。
- **降落任务控制**：点击 `Start Landing`（启动降落）或 `Cancel Landing`（中止降落）。

### 3. 命令行交互
```bash
# 启动自主降落任务
rostopic pub /pland/start std_msgs/Empty "{}" -1

# 中止任务并复位
rostopic pub /pland/cancel std_msgs/Empty "{}" -1

# 监控当前状态机状态
rostopic echo /pland/state
```

---

## 5. 话题与接口规范 (Topics & Interfaces)

### `pland_detector` 视觉检测模块
| 话题名称 | 消息类型 | 属性 | 说明 |
| :--- | :--- | :--- | :--- |
| `/roscam/cam/image_raw` | `sensor_msgs/Image` | 订阅 | 原始相机图像输入 |
| `/mavros/local_position/odom` | `nav_msgs/Odometry` | 订阅 | 飞控本体系里程计与姿态 |
| `/pland/target_pose` | `geometry_msgs/PoseStamped` | 发布 | 目标在机体系 (`base_link`, FLU) 下的 3D 相对位姿 |
| `/pland/target_vel` | `geometry_msgs/TwistStamped` | 发布 | 目标在世界系 (`map`, ENU) 下的估计前馈速度 |
| `/pland/target_pixel` | `geometry_msgs/PointStamped` | 发布 | 目标在图像平面的归一化像素坐标 $[0, 1]$ |

### `pland_controller` 降落控制模块
| 话题名称 | 消息类型 | 属性 | 说明 |
| :--- | :--- | :--- | :--- |
| `/pland/start` | `std_msgs/Empty` | 订阅 | 启动降落任务指令 |
| `/pland/cancel` | `std_msgs/Empty` | 订阅 | 中止降落并复位至 `IDLE` |
| `/pland/inject_target_pose` | `sensor_msgs/NavSatFix` | 订阅 | 外部注入的远端目标 GNSS 经纬高 |
| `/pland/state` | `std_msgs/String` | 发布 | 当前状态机阶段字符串 |
| `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | 发布 | 下发至飞控的速度制导指令通道 |

---

## 6. 核心参数说明 (Configuration)

配置位于 `pland_controller/config/pland_controller.yaml`：

| 参数项 | 默认值 | 单位 | 说明 |
| :--- | :--- | :--- | :--- |
| `touchdown_velocity` | `0.4` | m/s | 触地阶段恒定下降速度 |
| `blind_drop_alt` | `0.4` | m | 进入盲降阶段的高度阈值 |
| `blind_drop_xy_thresh`| `0.15`| m | 允许进入盲降的最大水平偏差 |
| `lost_target_alt` | `15.0` | m | 目标丢失安全重捕爬升高度 |
| `max_speed_xy` | `3.0` | m/s | 水平最大反馈控制速度 |
| `target_distance` | `8.0` | m | GPS 巡航转视觉跟踪的切换距离阈值 |
| `use_ff_vel` | `true` | bool | 是否启用目标速度前馈补偿 |

---

## 7. 引用 (Citation)

```bibtex
@misc{pland2026,
  author = {Pland Developers},
  title = {Pland: Precision Aerial Landing System for UAVs},
  year = {2026},
  howpublished = {\url{https://github.com/hggshiwo/pland}}
}
```

## 8. 开源协议 (License)
本项目采用 [Apache-2.0](LICENSE) 开源协议。
