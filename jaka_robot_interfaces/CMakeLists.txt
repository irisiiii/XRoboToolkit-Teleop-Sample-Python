cmake_minimum_required(VERSION 3.25)  

project(jaka_robot_interfaces)

# 支持 C++11
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)  # 你 package.xml 里也声明了它

# 生成接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MultiMovJ.srv"
  "srv/ServoMovJ.srv"
  "srv/MultiMovL.srv"
  "srv/JointToCartesian.srv"
  "srv/ClearError.srv"
  "msg/JointValue.msg"
  "msg/CartesianPose.msg"
  "msg/MoveMode.msg"
  "msg/RobotStateDual.msg"
  "msg/ServoJointCommand.msg"
  "msg/ServoCartCommand.msg"
  DEPENDENCIES builtin_interfaces std_msgs  # std_msgs 是你 msg 中可能依赖的类型
)

# 导出依赖
ament_export_dependencies(rosidl_default_runtime)
ament_package()
