cmake_minimum_required(VERSION 3.8)
project(frame_sync_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成消息接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SyncedData.msg"
  "msg/StampedFloat64MultiArray.msg"
  DEPENDENCIES std_msgs sensor_msgs
)
ament_package()
