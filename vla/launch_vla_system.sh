#!/bin/bash
# VLA系统启动脚本
# 使用方法: ./launch_vla_system.sh

cd /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python

echo "=== 启动 VLA 控制系统 ==="
echo ""
echo "请按以下顺序启动（每个命令在单独的终端中运行）："
echo ""
echo "1️⃣  启动夹爪驱动节点:"
echo "    source install/setup.bash"
echo "    ros2 run jiazhua_driver jiazhua_node"
echo ""
echo "2️⃣  启动机器人状态转换节点:"
echo "    source install/setup.bash"
echo "    ros2 run robot_state_converter converter"
echo ""
echo "3️⃣  启动VLA HTTP客户端:"
echo "    source install/setup.bash"
echo "    python3 vla/http_client.py"
echo ""
echo "=== 话题架构说明 ==="
echo "jiazhua_node → /left_arm/jiazhua_state → robot_state_converter → /left_arm/gripper_state → http_client.py"
echo ""

