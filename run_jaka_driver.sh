#!/bin/bash

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 加载 ROS2 Humble 环境
source /opt/ros/humble/setup.bash

# 加载本地工作空间环境
source ${SCRIPT_DIR}/install/setup.bash

# 设置库路径
export LD_LIBRARY_PATH=${SCRIPT_DIR}/install/jaka_robot_driver/lib:$LD_LIBRARY_PATH

# 运行 ROS2 driver launch 文件
ros2 launch jaka_robot_driver jaka_driver.launch.py

