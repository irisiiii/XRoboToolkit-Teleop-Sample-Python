# cmake_minimum_required(VERSION 3.25)
# project(jiazhua_driver)

# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(jiazhua_interfaces REQUIRED) # 依赖于 jiazhua_interfaces 包
# find_package(frame_sync_msgs REQUIRED)

# include_directories(
#   include # 或者其他包含头文件的路径
# )

# add_executable(jiazhua_node
#   src/jiazhua.cpp
#   src/jiazhua_node.cpp
# )



# ament_target_dependencies(jiazhua_node
#   rclcpp
#   std_msgs
#   jiazhua_interfaces # 引入消息包
#   rosidl_default_runtime
#   "frame_sync_msgs"
# )

# install(TARGETS
#   jiazhua_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# ament_package()

cmake_minimum_required(VERSION 3.25)
project(jiazhua_driver)

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(jiazhua_interfaces REQUIRED)
find_package(frame_sync_msgs REQUIRED)

# 包含头文件目录
include_directories(
  include
)

# 创建可执行文件
add_executable(jiazhua_node
  src/jiazhua.cpp
  src/jiazhua_node.cpp
)

# 设置目标属性
target_include_directories(jiazhua_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(jiazhua_node PUBLIC c_std_99 cxx_std_17)

# 链接依赖
ament_target_dependencies(jiazhua_node
  rclcpp
  std_msgs
  jiazhua_interfaces
  frame_sync_msgs
)

# 安装可执行文件
install(TARGETS
  jiazhua_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装头文件（如果需要其他包使用）
install(DIRECTORY include/
  DESTINATION include/
)

# 测试配置
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()