// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from jaka_robot_interfaces:msg/JointValue.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__MSG__DETAIL__JOINT_VALUE__TRAITS_HPP_
#define JAKA_ROBOT_INTERFACES__MSG__DETAIL__JOINT_VALUE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "jaka_robot_interfaces/msg/detail/joint_value__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace jaka_robot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const JointValue & msg,
  std::ostream & out)
{
  out << "{";
  // member: joint_values
  {
    if (msg.joint_values.size() == 0) {
      out << "joint_values: []";
    } else {
      out << "joint_values: [";
      size_t pending_items = msg.joint_values.size();
      for (auto item : msg.joint_values) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const JointValue & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: joint_values
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joint_values.size() == 0) {
      out << "joint_values: []\n";
    } else {
      out << "joint_values:\n";
      for (auto item : msg.joint_values) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const JointValue & msg, bool use_flow_style = false)
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
  const jaka_robot_interfaces::msg::JointValue & msg,
  std::ostream & out, size_t indentation = 0)
{
  jaka_robot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use jaka_robot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const jaka_robot_interfaces::msg::JointValue & msg)
{
  return jaka_robot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<jaka_robot_interfaces::msg::JointValue>()
{
  return "jaka_robot_interfaces::msg::JointValue";
}

template<>
inline const char * name<jaka_robot_interfaces::msg::JointValue>()
{
  return "jaka_robot_interfaces/msg/JointValue";
}

template<>
struct has_fixed_size<jaka_robot_interfaces::msg::JointValue>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<jaka_robot_interfaces::msg::JointValue>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<jaka_robot_interfaces::msg::JointValue>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // JAKA_ROBOT_INTERFACES__MSG__DETAIL__JOINT_VALUE__TRAITS_HPP_
