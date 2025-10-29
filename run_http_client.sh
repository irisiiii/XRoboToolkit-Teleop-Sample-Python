#!/bin/bash

# 激活conda环境
source /opt/miniconda3/bin/activate client2

# 加载ROS2环境
source /opt/ros/humble/setup.bash

# 加载本地工作空间（包含自定义接口）
source /root/jwq/xr/XRoboToolkit-Teleop-Sample-Python/install/setup.bash

# 运行客户端
python vla/http_client.py "$@"
