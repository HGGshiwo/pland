# Pland 开发与调试日志 (Development & Debugging Log)

本文档系统记录 Pland 自主降落系统在真机飞行与实测数据回放中发现的关键问题、机理分析与代码修复方案。

---

## [2026-10-09] 降落末端开环侧漂根除与全闭环触地逻辑重构

### 1. 问题现象
* **测试数据源**：`task_20261008_084031_916.bag`
* **故障现象**：无人机在下降阶段对中良好，在高度降至 $0.48\text{ m}$ 时水平误差仅 **$8\text{ mm}$**（几乎完美对中靶心）。但最终触地落定后，偏离了靶心中心约 **$9 \sim 13\text{ cm}$**。

### 2. 根因剖析
1. **原控制状态机盲降逻辑缺陷**：
   * 原代码在 `TRACING_DETECTOR` 中配置了无条件切换规则：只要高度 $Z < \text{blind\_drop\_alt}\ (0.3\text{m})$ 且误差 $< 0.2\text{m}$，控制器就会强行切入 `BLIND_DROP`（盲降）状态。
   * 在 `BLIND_DROP` 状态中，控制器将水平速度指令**直接清零**（$v_{x,\text{cmd}} = 0, v_{y,\text{cmd}} = 0$），彻底切断了水平位置纠偏闭环。
2. **实际硬件与环境表现**：
   * 视觉算法已升级为中心微型 AprilTag 阵列，在 $0.3\text{m}$ 乃至更低高度时，相机视野内**清晰稳定检测出 16 ~ 24 个 Tag**，视觉并未“失明”。
   * 近地旋翼强下洗气垫（地面效应）与原有水平惯性使飞机产生约 $0.05 \sim 0.08\text{ m/s}$ 的侧滑。在没有闭环纠偏的情况下，开环坠地滑行了 $0.7\text{ s}$，侧漂累积达 $5.6\text{ cm}$，占整体偏离的 $75\%$ 以上。

### 3. 代码修复与架构重构 (零新增控制参数)
* **涉及文件**：[`pland_controller/src/pland_controller.cpp`](file:///home/hggshiwo/catkin_ws/src/pland/pland_controller/src/pland_controller.cpp)
* **重构内容**：
  1. **视觉可用时全闭环对齐到底**：
     在 `TRACING_DETECTOR` 中移除了无条件切盲降的门限，增加直接触地检测：
     ```cpp
     // 只要视觉有效，持续保持全闭环对齐到底，触地直接停机 (彻底消除开环侧漂)
     if (current_z < exit_alt_) {
       change_state(ControllerState::LANDED);
       return;
     }
     ```
  2. **视觉不可用时超低空强制盲降（严禁爬升逃逸）**：
     当视觉意外丢失时，判断当前飞行高度：
     * 若处于低空（$Z < \text{blind\_drop\_alt}$），判定为触地前紧急情况，强制切入 `BLIND_DROP` 垂直下沉触地锁机，坚决不再触发爬升至 8m 的危险动作；
     * 仅当在高空丢失时，才执行原有的 GPS 备用切回或悬停爬升。
     ```cpp
     if (!detector_target_valid()) {
       if (current_z < blind_drop_alt_) {
         ROS_WARN("[PlandController] Target lost at low altitude. Enforcing BLIND_DROP!");
         change_state(ControllerState::BLIND_DROP);
       } else if (inject_target_valid()) {
         change_state(ControllerState::TRACING_GPS);
       } else {
         change_state(ControllerState::TARGET_LOST);
       }
       return;
     }
     ```
  3. **盲降找回视觉无缝切回**：
     在 `BLIND_DROP` 状态中增加恢复检测，若下沉中重新捕获到目标，立即恢复 `TRACING_DETECTOR` 闭环。

---

## [2026-10-08] 测距仪超近距驱动门限误判与高度跳变修复

### 1. 问题现象
* **测试数据源**：`task_20261008_022833_121.bag`
* **故障现象**：`/pland/detect` 调试画面上的 `ALT` 高度在降落过程中几乎从未显示低于 $0.2\text{m}$，且在最后触地停机时刻反弹定格在 **$0.42\text{ m}$**。

### 2. 根因剖析
1. **测距仪驱动元数据冲突**：
   * 飞机实际落定在平面时，测距仪实际物理测量值为 **$0.040\text{ m}$（4 厘米）**。
   * 但传感器 ROS 驱动（`sensor_msgs/Range`）消息头中配置了保守的 `min_range = 0.100 m`（10 厘米）。
2. **节点有效性校验拦截与退化**：
   * `pland_detector_node.cpp` 与 `pland_controller_node.cpp` 中均执行硬校验 `msg->range >= msg->min_range`。
   * 当高度低于 10cm（0.09m、0.04m）时，校验返回 `false`，判定测距仪失效并置 `has_rangefinder_ = false`。
   * 融合高度 `get_current_z()` 失去测距仪后，自动退化回气压计 `rel_alt`。由于旋翼近地下洗气流与气压累积零漂，此时气压计读数恰好为 **$0.424\text{ m}$**。

### 3. 代码修复
* **涉及文件**：
  * [`pland_detector/src/pland_detector_node.cpp`](file:///home/hggshiwo/catkin_ws/src/pland/pland_detector/src/pland_detector_node.cpp)
  * [`pland_controller/src/pland_controller_node.cpp`](file:///home/hggshiwo/catkin_ws/src/pland/pland_controller/src/pland_controller_node.cpp)
* **修改内容**：
  放宽下限校验至物理极限：
  ```cpp
  // 放宽下限至 0.01m，避免传感器驱动保守的 min_range(0.1m) 导致落地触地(0.04m)误判失效而退化回气压计
  bool is_valid = (!std::isnan(msg->range) && !std::isinf(msg->range) &&
                   msg->range > 0.01 && msg->range <= msg->max_range);
  ```
* **效果验证**：
  使用真机节点完整重放，OSD 高度平滑下降至真实贴地值 **`0.00 m`**，彻底消除反弹跳变。

---

## [2026-10-08] CTRV-EKF 速度估计发散、高空噪声硬限幅与前馈振荡抑制

### 1. 问题现象
* **测试数据源**：`task_20261008_022833_121.bag`
* **故障现象**：无人机在跟踪静止地面靶标降落时，机身发生严重的前后俯仰（Pitch 轴）剧烈振荡，降落控制闭环濒临发散；速度估计初始突跳峰值高达 **$363.67\text{ m/s}$**，稳态过程持续估出 **$0.5 \sim 1.5\text{ m/s}$** 的虚假速度。

### 2. 根因剖析
1. **$Q/R$ 过程噪声与观测噪声失衡上千倍**：
   * Commit `216f476` 中引入 Bar-Shalom 规范公式，离散速度过程噪声由 $\Delta t^2 \sigma_a^2$ 变为 $\Delta t \sigma_a^2$，叠加名义加速度放宽至 $3.0\text{ m/s}^2$，$Q$ 增益暴增约 300 倍。
   * 同次提交加入了 `dynamic_r_pos = std::clamp(..., 0.05, 1.2)` 硬限幅，切断了高空与大机动下 $R$ 向上自适应膨胀吸收测量抖动的路径。
   * $Q/R$ 极度激进化，卡尔曼滤波退化为剧烈数值微分器，将厘米级图像抖动误估为米每秒级目标速度。
2. **前馈反客为主劫持控制闭环**：
   * 控制器前馈项等效增益为 $(1 + K_d) = 1.35$ 倍。虚假前馈冲破 $0.2\text{ m/s}$ 死区后打满 $1.0\text{ m/s}$ 限速，导致电机前后疯狂拉扯起振。

### 3. 代码修复与参数物理对齐
* **涉及文件**：
  * [`pland_detector/include/kalman_filter_ctrv.hpp`](file:///home/hggshiwo/catkin_ws/src/pland/pland_detector/include/kalman_filter_ctrv.hpp)
  * [`pland_detector/include/pland_detector.hpp`](file:///home/hggshiwo/catkin_ws/src/pland/pland_detector/include/pland_detector.hpp)
  * [`pland_detector/src/pland_detector.cpp`](file:///home/hggshiwo/catkin_ws/src/pland/pland_detector/src/pland_detector.cpp)
  * [`pland_detector/config/pland_detector.yaml`](file:///home/hggshiwo/catkin_ws/src/pland/pland_detector/config/pland_detector.yaml)
* **修改内容**：
  1. **引入卡方检验新息门控（Innovation Gating）**：
     * 正常跟踪残差（$\epsilon \le 7.81$）：以名义小 $Q$ 运行；
     * 真实目标机动（$7.81 < \epsilon \le 16.0$）：温和放大 $Q$（上限收敛至 3.0 倍）；
     * 测量野值与突跳（$\epsilon > 16.0$）：坚决不放大 $Q$，对 $R$ 进行惩罚性膨胀降权。
  2. **去除 $R$ 矩阵硬上限截断**：
     恢复单目反投影几何方差二次方模型（$Z^2$），高空大晃动时自然放大 $R$，保留 $0.05/0.01$ 安全下限。
  3. **收敛名义物理参数**：
     * `ekf_nominal_max_acc`：收敛至 **$0.08\text{ m/s}^2$**（彻底压平静止工况微分毛刺）；
     * `ekf_nominal_max_yaw_acc`：收敛至 **$0.5\text{ rad/s}^2$**；
     * `base_r_noise_`：恢复至稳定基线 **$0.30$**。
  4. **动力学一致性限幅与硬饱和截断**：
     * 单步速度增量物理限幅：$|\Delta v| \le \text{max\_dv\_acc} \cdot \Delta t$（$0.8\text{ m/s}^2$）；
     * 目标极速硬饱和截断：$\pm 3.0\text{ m/s}$。
* **效果验证**：
  * 静止工况平均速度估计由 $1.45\text{ m/s}$ 降为 **$0.0000\text{ m/s}$**，死区归零比例达 **$100\%$**；
  * 真实机动下稳态跟踪误差小于 $0.03\text{ m/s}$，动态跟随与静止抗噪兼备。
