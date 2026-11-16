#!/usr/bin/env python
# coding=utf-8
"""get_current_z 融合高度用例: 注入传感器数据, 校验 /pland/current_z 输出。

调度逻辑 (fusion_min_alt=3.0 / fusion_max_alt=5.0 / frame_height=0.4, 来自现网配置):
  - 无测距仪 → rel_alt (失效退 odom)
  - rf_z < 3.0 → 纯测距仪 (含机架补偿)
  - rf_z > 5.0 → 纯 rel_alt (超测距仪量程)
  - 3~5m → rf_z ↔ rel_alt 线性混合
"""
import time

import pytest
import rospy

from injector import SensorInjector, ZWatcher

TOL = 0.01


@pytest.fixture(scope="module")
def rig(detector_env):
    inj = SensorInjector(rate_hz=20.0)
    watcher = ZWatcher()
    inj.start()
    yield inj, watcher
    inj.stop()


def settle(inj, watcher, seconds=1.5):
    """注入稳定 seconds 后取当前融合高度输出。"""
    time.sleep(seconds)
    watcher.clear()
    return watcher.get()


def test_rf_low_ground(rig):
    """低空 (rf_z<3m): 纯测距仪, 含机架高度补偿 (旧逻辑此处会误用 rel_alt)。"""
    inj, w = rig
    inj.set(odom_z=1.4, rel_alt=1.4, rf_range=0.44)  # rf_z = 0.44-0.4 = 0.04
    assert abs(settle(inj, w) - 0.04) < TOL


def test_no_rf(rig):
    """无测距仪: 直接使用 rel_alt。"""
    inj, w = rig
    inj.set(odom_z=2.0, rel_alt=2.0)
    inj.stop_rf()
    assert abs(settle(inj, w) - 2.0) < TOL


def test_rf_beyond_max(rig):
    """rf_z 超过 fusion_max_alt (测距仪量程外): 纯 rel_alt。"""
    inj, w = rig
    inj.set(odom_z=8.0, rel_alt=8.0, rf_range=8.4)  # rf_z = 8.0 > 5.0
    assert abs(settle(inj, w) - 8.0) < TOL


def test_blend_zone(rig):
    """3~5m 混合带: rf_z=4.0, rel_alt=4.5, ratio=0.5 → (4.0+4.5)/2 = 4.25。"""
    inj, w = rig
    inj.set(odom_z=4.5, rel_alt=4.5, rf_range=4.4)  # rf_z = 4.0
    assert abs(settle(inj, w) - 4.25) < TOL


def test_rel_alt_stale(rig):
    """回归用例 (task_20261116 bag 失败路径): rel_alt=12 但 rf_z=0.3 贴地,
    旧逻辑 base_z 落 odom/rel_alt 会输出 12 (近地判定失效);
    新逻辑调度变量是 rf_z, 任何时刻都应输出 0.3。
    """
    inj, w = rig
    inj.set(odom_z=12.0, rel_alt=12.0, rf_range=0.7)  # rf_z = 0.3
    assert abs(settle(inj, w) - 0.3) < TOL, "rel_alt 新鲜时也应信测距仪"
    # 停发 rel_alt (模拟断流 >1s), 测距仪仍在 → 依旧纯测距仪
    inj.stop_rel_alt()
    time.sleep(1.6)
    w.clear()
    assert abs(w.get() - 0.3) < TOL, "rel_alt 断流后不得退到 odom z"


def test_both_stale_fallback(rig):
    """测距仪与 rel_alt 双断流: 最终兜底 odom z。"""
    inj, w = rig
    inj.set(odom_z=12.0)
    inj.stop_rel_alt()
    inj.stop_rf()
    time.sleep(1.6)  # 超过 rel_alt 1.0s 与测距仪 0.5s 超时窗
    w.clear()
    assert abs(w.get() - 12.0) < TOL
