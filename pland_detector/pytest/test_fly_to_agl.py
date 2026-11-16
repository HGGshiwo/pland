#!/usr/bin/env python
# coding=utf-8
"""fly_to AGL 高度换算用例 (回归 task_20261116 bag 的 GPS 巡航持续下降事故):

注入 odom/rel_alt/GPS + GPS 注入目标, 驱动 pland_controller 进入 TRACING_GPS,
校验 /mavros/setpoint_raw/local 的期望高度:
  - position.z ≈ lost_target_alt(8) + (odom_z − rel_alt)   [AGL→odom 实时换算]
  - rel_alt 断流时退化为 odom 绝对值 (旧行为)
"""
import threading
import time

import pytest
import rospy
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import String

from injector import SensorInjector

# 现网配置 (pland_controller.yaml): lost_target_alt=8, rel_alt 超时 1.0s
LOST_TARGET_ALT = 8.0
ODOM_Z = 20.9
REL_ALT = 10.15
TOL = 0.05


@pytest.fixture(scope="module")
def controller_rig(controller_env):
    inj = SensorInjector(rate_hz=20.0)
    inj.set(odom_z=ODOM_Z, rel_alt=REL_ALT, rf_range=10.55)  # rf_z>5 → 高空用 rel_alt
    state = {"current": None}

    def state_cb(msg):
        state["current"] = msg.data

    sp = {"latest": None}
    rospy.Subscriber("/pland/state", String, state_cb, queue_size=10)
    rospy.Subscriber(
        "/mavros/setpoint_raw/local", PositionTarget,
        lambda m: sp.update(latest=m), queue_size=10)
    inj.start()

    # GPS 注入循环: flag 开启时以 2.5Hz 持续注入 (>1/target_timeout), 保证
    # TRACING_GPS 状态在断言期间不因 inject 超时退出
    gps_inject = {"on": False}
    tgt = {"lat": inj.gps_lat + 0.0003, "lon": inj.gps_lon}

    def inject_loop():
        while not rospy.is_shutdown():
            if gps_inject["on"]:
                inj.send_inject_target(tgt["lat"], tgt["lon"], 0.0)
            time.sleep(0.4)

    threading.Thread(target=inject_loop, daemon=True).start()
    yield inj, state, sp, gps_inject, tgt
    gps_inject["on"] = False
    inj.stop()


def _enter_tracing_gps(inj, state, gps_inject):
    """发送 start (多次重发防连接未建立丢包), 等待进入 TRACING_GPS。"""
    gps_inject["on"] = True
    deadline = time.time() + 10
    while time.time() < deadline:
        inj.send_start()  # 周期重发, 直到确认状态切换
        if state["current"] == "TRACING_GPS":
            return
        time.sleep(0.3)
    raise AssertionError(f"未进入 TRACING_GPS (当前: {state['current']})")


def test_fly_to_z_is_agl_converted(controller_rig):
    """TRACING_GPS 期望高度 = lost_target_alt + (odom_z − rel_alt) = 8 + 10.75 = 18.75。"""
    inj, state, sp, gps_inject, _ = controller_rig
    _enter_tracing_gps(inj, state, gps_inject)
    time.sleep(1.0)  # 等 setpoint 稳定流出
    msg = sp["latest"]
    assert msg is not None, "未收到 /mavros/setpoint_raw/local"
    expected = LOST_TARGET_ALT + (ODOM_Z - REL_ALT)
    assert abs(msg.position.z - expected) < TOL, (
        f"setpoint z={msg.position.z:.2f}, 期望 {expected:.2f} "
        f"(事故旧行为: 直发 odom 系 8.0 → 持续下降砸地)")


def test_fly_to_z_stale_rel_alt_fallback(controller_rig):
    """rel_alt 断流 >1s: 不换算, setpoint z 退化为 odom 绝对值 (旧行为 8.0)。"""
    inj, state, sp, gps_inject, _ = controller_rig
    _enter_tracing_gps(inj, state, gps_inject)  # 若上例后已退出则重新进入
    inj.stop_rel_alt()
    time.sleep(1.6)
    msg = sp["latest"]
    assert msg is not None
    assert abs(msg.position.z - LOST_TARGET_ALT) < TOL, (
        f"setpoint z={msg.position.z:.2f}, 期望退化值 {LOST_TARGET_ALT:.2f}")
