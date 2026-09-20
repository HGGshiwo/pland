# Pland: Precision Aerial Landing System for UAVs

<p align="center">
  <a href="README.md"><b>English</b></a> |
  <a href="README_CN.md">简体中文</a>
</p>

<p align="center">
  <a href="https://ros.org/"><img src="https://img.shields.io/badge/ROS-Noetic-blue.svg" alt="ROS Noetic"></a>
  <a href="https://ubuntu.com/"><img src="https://img.shields.io/badge/Ubuntu-20.04-orange.svg" alt="Ubuntu 20.04"></a>
  <img src="https://img.shields.io/badge/C%2B%2B-17-green.svg" alt="C++17">
  <img src="https://img.shields.io/badge/Python-3.8-yellow.svg" alt="Python 3.8">
  <a href="LICENSE"><img src="https://img.shields.io/badge/License-Apache%202.0-lightgrey.svg" alt="License"></a>
</p>

**Pland** is an open-source, high-precision autonomous landing framework for multirotor UAVs, delivering robust touchdown on both stationary and dynamic moving platforms.




https://github.com/user-attachments/assets/cce8fab0-cbee-4ad5-b0c5-5d859571ec92





---

## 1. Features

- **Direct Body-Frame (FLU) Visual Closed Loop**: Solves 3D target pose directly in the aircraft's body frame (`base_link`, Forward-Left-Up), eliminating coordinate transformation latency and Earth-frame drift.
- **Hierarchical Multi-Scale AprilTag Array**: Uses outer large tags for high-altitude acquisition and inner small tags for low-altitude alignment to prevent field-of-view (FOV) clipping.
- **CTRV Extended Kalman Filter (EKF)**: Fuses observations via a 2D Constant Turn Rate and Velocity model to estimate target motion and provide velocity feedforward.
- **Adaptive Descent Funnel & Gain Scheduling**: Tightens horizontal alignment tolerance as altitude decreases ($r_{\text{tol}} = r_{\min} + k \cdot z$) and scales down control gains near the ground to mitigate ground effect.
- **6-State Finite State Machine (FSM)**: Coordinates `IDLE`, `TRACING_GPS`, `TRACING_DETECTOR`, `BLIND_DROP`, `TARGET_LOST`, and `LANDED` with seamless GPS-to-vision handover.

---

## 2. Supported Scenarios

| Scenario | Mode | Description |
| :--- | :--- | :--- |
| **1. Stationary Target** | `STOP` | High-precision hovering, funnel alignment, and terminal blind touchdown on static ground pads. |
| **2. Straight-Line Motion** | `FORWARD` / `BACKWARD` | Autonomous tracking and landing on targets moving at constant or time-varying linear speeds, supported by EKF velocity feedforward compensation. |
| **3. Snake / Lane-Change Maneuver** | `SNAKE` | Tracking targets undergoing road-like weaving and lane changes where the platform continuously rotates and turns along circular arcs ($R = v / \|\omega\|$), with configurable **Turn Radius ($R$)** and **Yaw Sweep ($\theta$)**. |

---

## 3. Installation

### Prerequisites
- Ubuntu 20.04 LTS & ROS Noetic
- Core libraries:
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

### Build
```bash
cd ~/catkin_ws/src
git clone https://github.com/hggshiwo/pland.git
catkin_make -DCMAKE_BUILD_TYPE=Release
source ~/catkin_ws/devel/setup.bash
```

---

## 4. Quick Start

### 1. Launch Simulation (Gazebo + ArduPilot SITL + GUI)
```bash
roslaunch pland_sim pland_sim.launch
```

### 2. Interactive GUI Dashboard
The built-in GUI panel allows one-click flight control and target scenario switching:
- **Drone**: Takeoff, Arm/Disarm, GUIDED/LAND mode.
- **Platform Motion**: Select `Stop` (Stationary), `Forward` (Straight-Line), or `Snake` (Snake/Lane-Change with custom `Radius` and `Radian`).
- **Pland Mission**: Click `Start Landing` or `Cancel Landing`.

### 3. Command-Line Interface
```bash
# Start landing task
rostopic pub /pland/start std_msgs/Empty "{}" -1

# Cancel landing task
rostopic pub /pland/cancel std_msgs/Empty "{}" -1

# Monitor FSM state
rostopic echo /pland/state
```

---

## 5. Topics & Interfaces

### `pland_detector`
| Topic | Type | Direction | Description |
| :--- | :--- | :--- | :--- |
| `/roscam/cam/image_raw` | `sensor_msgs/Image` | Sub | Raw camera input |
| `/mavros/local_position/odom` | `nav_msgs/Odometry` | Sub | UAV odometry & orientation |
| `/pland/target_pose` | `geometry_msgs/PoseStamped` | Pub | Target 3D pose in body frame (`base_link`, FLU) |
| `/pland/target_vel` | `geometry_msgs/TwistStamped` | Pub | Target feedforward velocity in ENU frame |
| `/pland/target_pixel` | `geometry_msgs/PointStamped` | Pub | Normalized image pixel coordinates $[0, 1]$ |

### `pland_controller`
| Topic | Type | Direction | Description |
| :--- | :--- | :--- | :--- |
| `/pland/start` | `std_msgs/Empty` | Sub | Triggers precision landing |
| `/pland/cancel` | `std_msgs/Empty` | Sub | Aborts mission and resets to `IDLE` |
| `/pland/inject_target_pose` | `sensor_msgs/NavSatFix` | Sub | Injected remote target GNSS position |
| `/pland/state` | `std_msgs/String` | Pub | Current FSM status string |
| `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | Pub | Velocity setpoints sent to flight controller |

---

## 6. Key Configuration Parameters

Configured in `pland_controller/config/pland_controller.yaml`:

| Parameter | Default | Unit | Description |
| :--- | :--- | :--- | :--- |
| `touchdown_velocity` | `0.4` | m/s | Descent velocity during touchdown |
| `blind_drop_alt` | `0.4` | m | Altitude threshold to enter blind drop |
| `blind_drop_xy_thresh`| `0.15`| m | Max allowed horizontal error for blind drop |
| `lost_target_alt` | `15.0` | m | Recovery climb altitude if target is lost |
| `max_speed_xy` | `3.0` | m/s | Saturation limit on horizontal feedback velocity |
| `target_distance` | `8.0` | m | Switch distance threshold from GPS to vision |
| `use_ff_vel` | `true` | bool | Enable velocity feedforward compensation |

---

## 7. Citation

```bibtex
@misc{pland2026,
  author = {Pland Developers},
  title = {Pland: Precision Aerial Landing System for UAVs},
  year = {2026},
  howpublished = {\url{https://github.com/hggshiwo/pland}}
}
```

## 8. License
Licensed under [Apache-2.0](LICENSE).
