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

- **自适应多尺度图像增强与检测 (`pland_detector`)**：
  - **高空段 ($Z > 5.0\text{m}$)**：采用纯视觉纹理能量快速粗定位 + 局域 $2\times$ 双三次超分辨率插值（C2F）与动态范围归一化拉伸，实现 12m 极限高空秒级锁靶（高空检出率 $100\%$），解算角点高精度逆映射回原图物理空间。
  - **中低空段 ($Z \le 5.0\text{m}$)**：平滑切入全图 CLAHE（对比度受限自适应直方图均衡化）增强，保证 4m 过渡段 4 个大尺寸 Tag 完整在线，近地段全面激活中心 25 个微型小 Tag 阵列（最高 29 个 Tag 联合解算），大幅提升 PnP 刚性与空间几何抗噪性。
- **机身体系 (FLU) 纯视觉闭环与微分阻尼 (`pland_controller`)**：
  - 目标三维相对位姿直接解算在飞机机身坐标系（`base_link`，前-左-上），消除全局坐标系转换时延与漂移。
  - 控制器微分反馈直接绑定机体系真实运动速度，提供强阻尼负反馈，配合位置优先自适应偏航角解耦，彻底消除高空及下降过程中的水平画圈与发散振荡。
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
catkin build pland_detector pland_controller pland_sim
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

## 6. 参数手册与调优指南 (Configuration & Tuning)

系统采用 ROS 动态参数同步（`ros_param_sync`），支持运行期间通过 `rosparam set` 实时修改并自动写回持久化文件。

### 1. `pland_detector` 配置 (`pland_detector.yaml`)
| 参数项 | 默认值 | 类型 | 说明 |
| :--- | :--- | :--- | :--- |
| `enable_c2f_enhancement` | `true` | bool | 开启自适应多尺度图像增强（高空 C2F 超分 + 低空全图 CLAHE） |
| `velocity_deadzone` | `0.2` | double (m/s) | 目标速度滤波死区，低于该值视目标为静止 |
| `publish_debug_image` | `true` | bool | 是否发布绘制了角点与位姿信息的调试图像 |
| `gimbal_abs` | `false` | bool | 云台固定角模式（true: 垂直地面绝对角; false: 相对机体角） |

### 2. `pland_controller` 配置 (`pland_controller.yaml`)
| 参数项 | 默认值 | 单位 | 说明 |
| :--- | :--- | :--- | :--- |
| `vision_kp` | `1.0` | - | 视觉水平位置比例反馈增益 ($P$) |
| `vision_kd` | `0.35` | - | 视觉机体系速度微分阻尼增益 ($D$)，有效抑制水平画圈振荡 |
| `max_yaw_rate` | `1.2` | rad/s | 偏航角速度限幅，防止大偏航误差时机动过猛 |
| `xy_align_thresh` | `0.45` | m | 水平位置优先收敛阈值，大于该误差时优先水平对齐并限速下降 |
| `touchdown_velocity` | `0.4` | m/s | 触地阶段恒定下降速度 |
| `blind_drop_alt` | `0.3` | m | 进入盲降阶段的高度阈值 |
| `blind_drop_xy_thresh`| `0.2` | m | 允许进入盲降的最大水平偏差 |
| `lost_target_alt` | `8.0` | m | 目标丢失安全重捕爬升高度 |
| `max_speed_xy` | `1.0` | m/s | 水平最大反馈控制速度 |
| `target_distance` | `3.0` | m | GPS 巡航转视觉跟踪的切换距离阈值 |
| `use_ff_vel` | `true` | bool | 是否启用目标 CTRV 速度前馈补偿 |

---

## 7. 自动化性能评测工具 (Validation & Benchmark)

在 `pland_detector/scripts/` 下提供了离线闭环评测与性能对比脚本 `validate_pland_detector.py`：

```bash
# 1. 运行基准评测并对比基线数据
python3 ~/catkin_ws/src/pland/pland_detector/scripts/validate_pland_detector.py \
    --bag ~/catkin_ws/src/task_20260923_074125_372.bag \
    --rate 2.0 \
    --preprocess none \
    --compare_with_json ~/catkin_ws/src/pland/pland_detector/scripts/baseline_metrics.json
```

该工具将自动启动被测节点、回放 Rosbag、采集 `/pland/target_pose` 并输出包含全局识别率、高/中/低空分段召回率、端到端延迟、最大连续丢靶帧数的性能报表。

---

## 8. 引用 (Citation)

```bibtex
@misc{pland2026,
  author = {Pland Developers},
  title = {Pland: Precision Aerial Landing System for UAVs},
  year = {2026},
  howpublished = {\url{https://github.com/hggshiwo/pland}}
}
```

## 9. 开源协议 (License)
本项目采用 [Apache-2.0](LICENSE) 开源协议。
