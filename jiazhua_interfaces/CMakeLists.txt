cmake_minimum_required(VERSION 3.25)
project(jiazhua_interfaces)

# 找到 ROS2 必要的依赖
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)  # 确保加载 std_msgs 包

# 生成消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JiaZhuaDualCmd.msg"
  DEPENDENCIES std_msgs  # 注意，这里需要正确引用 std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
