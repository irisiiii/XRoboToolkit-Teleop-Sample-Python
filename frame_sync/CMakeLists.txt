cmake_minimum_required(VERSION 3.8)
project(frame_sync)
add_compile_options(-O2)
set(CMAKE_CXX_STANDARD 20)  # 设置C++标准为C++23
set(CMAKE_CXX_STANDARD_REQUIRED ON)  # 确保使用C++23标
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(message_filters REQUIRED)  # Add this for message_filters
find_package(frame_sync_msgs REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)
pkg_check_modules(CURL REQUIRED libcurl)
set(CURSES_NEED_WIDE TRUE)
set(CURSES_NEED_NCURSES TRUE)  # 可选：确保使用 ncurses 实现
find_package(Curses REQUIRED)
include_directories(
  include
  ${frame_sync_msgs_INCLUDE_DIRS}
  ${CURSES_INCLUDE_DIRS}
)


add_executable(frame_sync_node 
src/frame_sync_node.cpp
src/ConsoleUI.cpp
src/DataCollecter.cpp
src/DataWriter.cpp
)
target_link_libraries(frame_sync_node 
  ${cpp_typesupport_target} 
  nlohmann_json::nlohmann_json
    ${OpenCV_LIBRARIES} 
  ncursesw  # 强制链接宽库
  ${CURL_LIBRARIES}
  curl
)
ament_target_dependencies(frame_sync_node
  rclcpp
  geometry_msgs
  sensor_msgs
  message_filters
  frame_sync_msgs
  std_srvs
  cv_bridge
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
install(TARGETS
  frame_sync_node
  DESTINATION lib/${PROJECT_NAME})
# install(TARGETS
# ARCHIVE DESTINATION lib
# LIBRARY DESTINATION lib
# RUNTIME DESTINATION bin
# )
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
