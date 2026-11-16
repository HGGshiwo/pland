#!/usr/bin/env bash
# 一键测试脚本: source ROS/工作空间环境, 运行 pytest, 结束后清理残留。
# roscore 与 pland_detector.launch 的启动/停止由 conftest fixture 负责。
# 用法:
#   ./pytest/run_test.sh                              # 跑全部用例
#   ./pytest/run_test.sh pytest/test_get_current_z.py -v   # 参数透传给 pytest
# 注意: 不能用 set -u, ROS 的 setup.bash 依赖未预定义变量 (ROS_DISTRO 等)
set -o pipefail

PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="$PKG_DIR/pytest/logs"
mkdir -p "$LOG_DIR"

cleanup() {
    echo "[run_test] 清理残留节点..."
    pkill -INT -f "pland_detector.launch" 2>/dev/null
    sleep 2
    pkill -9 -f "pland_detector_node" 2>/dev/null
    return 0
}
trap cleanup EXIT

# ROS 环境
if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
else
    source /opt/ros/*/setup.bash
fi
# 工作空间 (兼容 devel/install 两种布局)
if [ -f /home/hggshiwo/catkin_ws/devel/setup.bash ]; then
    source /home/hggshiwo/catkin_ws/devel/setup.bash
elif [ -f /home/hggshiwo/catkin_ws/install/setup.bash ]; then
    source /home/hggshiwo/catkin_ws/install/setup.bash
fi

cd "$PKG_DIR"
# 未给参数时默认跑整个 pytest/ 目录; 给了参数则只跑指定文件/用例
if [ "$#" -gt 0 ]; then
    python3 -m pytest "$@" 2>&1 | tee "$LOG_DIR/pytest_out.log"
else
    python3 -m pytest pytest/ -v 2>&1 | tee "$LOG_DIR/pytest_out.log"
fi
