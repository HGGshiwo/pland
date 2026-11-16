#!/usr/bin/env python
# coding=utf-8
"""pytest 公共 fixture: 按需拉起 pland_detector/controller 的 roslaunch, 会话结束确定性停掉。

仿照 dankong/pytest/conftest.py 的做法:
  - 不显式管理 roscore —— roslaunch 在无 master 时会自动拉起, 退出时带走
    自己启动的 master (外部已有 master 则保留不动);
  - 环境由 fixture 全权管理 (按需启动 / 用完即停), 测试用例只管注入与断言;
  - 进程以独立会话启动, 清理走 SIGINT -> 轮询 -> SIGKILL 三段式;
  - rospy.init_node 必须在 roslaunch (master) 就绪之后执行 —— rospy 联系不上
    master 会无限重试, 所以 rig fixture 里带守卫地 init, 不做独立前置 fixture。
"""
import subprocess
import time
from pathlib import Path

import pytest

PKG_DIR = Path(__file__).resolve().parent.parent
LOG_DIR = Path(__file__).parent / "logs"

_launch_proc = None
_controller_proc = None


def _wait_topic(topic: str, timeout: float) -> bool:
    """轮询 rostopic list 直到目标话题出现 (master + 节点均就绪的可靠信号)。"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        r = subprocess.run(["rostopic", "list"], capture_output=True, text=True)
        if topic in r.stdout.split():
            return True
        time.sleep(0.5)
    return False


def _stop_launch(pattern: str, node_pattern: str, proc):
    """确定性停止 roslaunch (SIGINT -> 轮询 -> SIGKILL)。"""
    subprocess.run(["pkill", "-INT", "-f", pattern], check=False)
    deadline = time.time() + 10
    while time.time() < deadline:
        if proc.poll() is not None:
            return
        alive = subprocess.run(
            ["pgrep", "-f", node_pattern], capture_output=True
        ).returncode == 0
        if not alive:
            return
        time.sleep(0.5)
    print(f"[conftest] SIGINT 未停干净 ({pattern}), 升级 SIGKILL")
    subprocess.run(["pkill", "-9", "-f", pattern], check=False)
    subprocess.run(["pkill", "-9", "-f", node_pattern], check=False)


def _ensure_ros_node():
    """整个 pytest 进程只 init_node 一次 (rospy 二次 init 不同名会抛 ROSException)。
    仅可在 env fixture (master 已就绪) 内调用, 否则 rospy 会无限重试卡死。"""
    import rospy
    from rospy.exceptions import ROSException
    if not rospy.core.is_initialized():
        rospy.init_node("pland_pytest", anonymous=True, disable_signals=True)


@pytest.fixture(scope="session")
def detector_env():
    """会话级环境: pland_detector.launch (无 master 时 roslaunch 自动拉起)。"""
    global _launch_proc
    LOG_DIR.mkdir(exist_ok=True)

    _launch_proc = subprocess.Popen(
        ["roslaunch", "pland_detector", "pland_detector.launch"],
        stdout=open(LOG_DIR / "detector.log", "ab"),
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    assert _wait_topic("/pland/current_z", 30), (
        "pland_detector 节点未就绪 (topic /pland/current_z 未出现), "
        "详见 pytest/logs/detector.log"
    )
    _ensure_ros_node()

    yield

    _stop_launch("pland_detector.launch", "pland_detector_node", _launch_proc)
    _launch_proc = None


@pytest.fixture(scope="session")
def controller_env():
    """会话级环境: pland_controller.launch (无 master 时 roslaunch 自动拉起)。"""
    global _controller_proc
    LOG_DIR.mkdir(exist_ok=True)

    _controller_proc = subprocess.Popen(
        ["roslaunch", "pland_controller", "pland_controller.launch"],
        stdout=open(LOG_DIR / "controller.log", "ab"),
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    assert _wait_topic("/pland/state", 30), (
        "pland_controller 节点未就绪 (topic /pland/state 未出现), "
        "详见 pytest/logs/controller.log"
    )
    _ensure_ros_node()

    yield

    _stop_launch(
        "pland_controller.launch", "pland_controller_node", _controller_proc)
    _controller_proc = None
