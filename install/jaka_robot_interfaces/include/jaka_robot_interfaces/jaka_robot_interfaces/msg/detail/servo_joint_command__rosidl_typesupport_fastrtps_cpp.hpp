// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from jaka_robot_interfaces:msg/ServoJointCommand.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__MSG__DETAIL__SERVO_JOINT_COMMAND__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define JAKA_ROBOT_INTERFACES__MSG__DETAIL__SERVO_JOINT_COMMAND__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "jaka_robot_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "jaka_robot_interfaces/msg/detail/servo_joint_command__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace jaka_robot_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_jaka_robot_interfaces
cdr_serialize(
  const jaka_robot_interfaces::msg::ServoJointCommand & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_jaka_robot_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  jaka_robot_interfaces::msg::ServoJointCommand & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_jaka_robot_interfaces
get_serialized_size(
  const jaka_robot_interfaces::msg::ServoJointCommand & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_jaka_robot_interfaces
max_serialized_size_ServoJointCommand(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace jaka_robot_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_jaka_robot_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, jaka_robot_interfaces, msg, ServoJointCommand)();

#ifdef __cplusplus
}
#endif

#endif  // JAKA_ROBOT_INTERFACES__MSG__DETAIL__SERVO_JOINT_COMMAND__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
