#!/bin/bash

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 加载 ROS2 Humble 环境
source /opt/ros/humble/setup.bash

# 加载本地工作空间环境（包含 jaka_robot_interfaces）
source ${SCRIPT_DIR}/install/setup.bash

# 激活 conda 环境
source /opt/miniconda3/etc/profile.d/conda.sh
conda activate xr-robotics

# 设置库路径
export LD_LIBRARY_PATH=${SCRIPT_DIR}/install/jaka_robot_driver/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${SCRIPT_DIR}/install/jaka_robot_interfaces/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=${SCRIPT_DIR}/dependencies/XRoboToolkit-PC-Service-Pybind/lib:$LD_LIBRARY_PATH

# 运行程序
python ${SCRIPT_DIR}/scripts/ros2/teleop_jaka_ros2.py

