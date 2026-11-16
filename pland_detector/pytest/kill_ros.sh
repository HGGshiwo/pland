#!/usr/bin/env bash
# 清理本测试套件相关的 ROS 残留进程 (独立文件: 避免 pkill 模式与调用方命令行自匹配)
pkill -9 -f "roslaunch" 2>/dev/null
pkill -9 -f "roscore" 2>/dev/null
pkill -9 -f "rosmaster" 2>/dev/null
pkill -9 -f "pland_detector_node" 2>/dev/null
pkill -9 -f "pland_controller_node" 2>/dev/null
sleep 2
echo "cleaned: $(ps aux | grep -cE 'roscore|rosmaster|pland.*node|roslaunch' | head -1)"
