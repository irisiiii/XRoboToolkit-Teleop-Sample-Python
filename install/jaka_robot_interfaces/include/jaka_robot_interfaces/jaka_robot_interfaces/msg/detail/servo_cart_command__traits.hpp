// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from jaka_robot_interfaces:msg/ServoCartCommand.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__MSG__DETAIL__SERVO_CART_COMMAND__TRAITS_HPP_
#define JAKA_ROBOT_INTERFACES__MSG__DETAIL__SERVO_CART_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "jaka_robot_interfaces/msg/detail/servo_cart_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'end_pose_left'
// Member 'end_pose_right'
#include "jaka_robot_interfaces/msg/detail/cartesian_pose__traits.hpp"

namespace jaka_robot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ServoCartCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: end_pose_left
  {
    out << "end_pose_left: ";
    to_flow_style_yaml(msg.end_pose_left, out);
    out << ", ";
  }

  // member: end_pose_right
  {
    out << "end_pose_right: ";
    to_flow_style_yaml(msg.end_pose_right, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ServoCartCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: end_pose_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "end_pose_left:\n";
    to_block_style_yaml(msg.end_pose_left, out, indentation + 2);
  }

  // member: end_pose_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "end_pose_right:\n";
    to_block_style_yaml(msg.end_pose_right, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ServoCartCommand & msg, bool use_flow_style = false)
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
  const jaka_robot_interfaces::msg::ServoCartCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  jaka_robot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use jaka_robot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const jaka_robot_interfaces::msg::ServoCartCommand & msg)
{
  return jaka_robot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<jaka_robot_interfaces::msg::ServoCartCommand>()
{
  return "jaka_robot_interfaces::msg::ServoCartCommand";
}

template<>
inline const char * name<jaka_robot_interfaces::msg::ServoCartCommand>()
{
  return "jaka_robot_interfaces/msg/ServoCartCommand";
}

template<>
struct has_fixed_size<jaka_robot_interfaces::msg::ServoCartCommand>
  : std::integral_constant<bool, has_fixed_size<jaka_robot_interfaces::msg::CartesianPose>::value> {};

template<>
struct has_bounded_size<jaka_robot_interfaces::msg::ServoCartCommand>
  : std::integral_constant<bool, has_bounded_size<jaka_robot_interfaces::msg::CartesianPose>::value> {};

template<>
struct is_message<jaka_robot_interfaces::msg::ServoCartCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // JAKA_ROBOT_INTERFACES__MSG__DETAIL__SERVO_CART_COMMAND__TRAITS_HPP_
