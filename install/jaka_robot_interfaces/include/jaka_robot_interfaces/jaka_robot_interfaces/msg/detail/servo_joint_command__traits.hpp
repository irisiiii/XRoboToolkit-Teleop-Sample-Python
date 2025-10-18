// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from jaka_robot_interfaces:msg/ServoJointCommand.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__MSG__DETAIL__SERVO_JOINT_COMMAND__TRAITS_HPP_
#define JAKA_ROBOT_INTERFACES__MSG__DETAIL__SERVO_JOINT_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "jaka_robot_interfaces/msg/detail/servo_joint_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'joint_pos_left'
// Member 'joint_pos_right'
#include "jaka_robot_interfaces/msg/detail/joint_value__traits.hpp"

namespace jaka_robot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ServoJointCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: joint_pos_left
  {
    out << "joint_pos_left: ";
    to_flow_style_yaml(msg.joint_pos_left, out);
    out << ", ";
  }

  // member: joint_pos_right
  {
    out << "joint_pos_right: ";
    to_flow_style_yaml(msg.joint_pos_right, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ServoJointCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: joint_pos_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joint_pos_left:\n";
    to_block_style_yaml(msg.joint_pos_left, out, indentation + 2);
  }

  // member: joint_pos_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joint_pos_right:\n";
    to_block_style_yaml(msg.joint_pos_right, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ServoJointCommand & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace jaka_robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use jaka_robot_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const jaka_robot_interfaces::msg::ServoJointCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  jaka_robot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use jaka_robot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const jaka_robot_interfaces::msg::ServoJointCommand & msg)
{
  return jaka_robot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<jaka_robot_interfaces::msg::ServoJointCommand>()
{
  return "jaka_robot_interfaces::msg::ServoJointCommand";
}

template<>
inline const char * name<jaka_robot_interfaces::msg::ServoJointCommand>()
{
  return "jaka_robot_interfaces/msg/ServoJointCommand";
}

template<>
struct has_fixed_size<jaka_robot_interfaces::msg::ServoJointCommand>
  : std::integral_constant<bool, has_fixed_size<jaka_robot_interfaces::msg::JointValue>::value> {};

template<>
struct has_bounded_size<jaka_robot_interfaces::msg::ServoJointCommand>
  : std::integral_constant<bool, has_bounded_size<jaka_robot_interfaces::msg::JointValue>::value> {};

template<>
struct is_message<jaka_robot_interfaces::msg::ServoJointCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // JAKA_ROBOT_INTERFACES__MSG__DETAIL__SERVO_JOINT_COMMAND__TRAITS_HPP_
