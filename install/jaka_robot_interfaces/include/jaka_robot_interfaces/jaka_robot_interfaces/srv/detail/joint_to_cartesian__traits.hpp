// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from jaka_robot_interfaces:srv/JointToCartesian.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__SRV__DETAIL__JOINT_TO_CARTESIAN__TRAITS_HPP_
#define JAKA_ROBOT_INTERFACES__SRV__DETAIL__JOINT_TO_CARTESIAN__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "jaka_robot_interfaces/srv/detail/joint_to_cartesian__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'joint_value'
#include "jaka_robot_interfaces/msg/detail/joint_value__traits.hpp"

namespace jaka_robot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const JointToCartesian_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: joint_value
  {
    out << "joint_value: ";
    to_flow_style_yaml(msg.joint_value, out);
    out << ", ";
  }

  // member: robot_id
  {
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const JointToCartesian_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: joint_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "joint_value:\n";
    to_block_style_yaml(msg.joint_value, out, indentation + 2);
  }

  // member: robot_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const JointToCartesian_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace jaka_robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use jaka_robot_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const jaka_robot_interfaces::srv::JointToCartesian_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  jaka_robot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use jaka_robot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const jaka_robot_interfaces::srv::JointToCartesian_Request & msg)
{
  return jaka_robot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<jaka_robot_interfaces::srv::JointToCartesian_Request>()
{
  return "jaka_robot_interfaces::srv::JointToCartesian_Request";
}

template<>
inline const char * name<jaka_robot_interfaces::srv::JointToCartesian_Request>()
{
  return "jaka_robot_interfaces/srv/JointToCartesian_Request";
}

template<>
struct has_fixed_size<jaka_robot_interfaces::srv::JointToCartesian_Request>
  : std::integral_constant<bool, has_fixed_size<jaka_robot_interfaces::msg::JointValue>::value> {};

template<>
struct has_bounded_size<jaka_robot_interfaces::srv::JointToCartesian_Request>
  : std::integral_constant<bool, has_bounded_size<jaka_robot_interfaces::msg::JointValue>::value> {};

template<>
struct is_message<jaka_robot_interfaces::srv::JointToCartesian_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'cartesian_pose'
#include "jaka_robot_interfaces/msg/detail/cartesian_pose__traits.hpp"

namespace jaka_robot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const JointToCartesian_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: cartesian_pose
  {
    out << "cartesian_pose: ";
    to_flow_style_yaml(msg.cartesian_pose, out);
    out << ", ";
  }

  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const JointToCartesian_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: cartesian_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cartesian_pose:\n";
    to_block_style_yaml(msg.cartesian_pose, out, indentation + 2);
  }

  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const JointToCartesian_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace jaka_robot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use jaka_robot_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const jaka_robot_interfaces::srv::JointToCartesian_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  jaka_robot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use jaka_robot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const jaka_robot_interfaces::srv::JointToCartesian_Response & msg)
{
  return jaka_robot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<jaka_robot_interfaces::srv::JointToCartesian_Response>()
{
  return "jaka_robot_interfaces::srv::JointToCartesian_Response";
}

template<>
inline const char * name<jaka_robot_interfaces::srv::JointToCartesian_Response>()
{
  return "jaka_robot_interfaces/srv/JointToCartesian_Response";
}

template<>
struct has_fixed_size<jaka_robot_interfaces::srv::JointToCartesian_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<jaka_robot_interfaces::srv::JointToCartesian_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<jaka_robot_interfaces::srv::JointToCartesian_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<jaka_robot_interfaces::srv::JointToCartesian>()
{
  return "jaka_robot_interfaces::srv::JointToCartesian";
}

template<>
inline const char * name<jaka_robot_interfaces::srv::JointToCartesian>()
{
  return "jaka_robot_interfaces/srv/JointToCartesian";
}

template<>
struct has_fixed_size<jaka_robot_interfaces::srv::JointToCartesian>
  : std::integral_constant<
    bool,
    has_fixed_size<jaka_robot_interfaces::srv::JointToCartesian_Request>::value &&
    has_fixed_size<jaka_robot_interfaces::srv::JointToCartesian_Response>::value
  >
{
};

template<>
struct has_bounded_size<jaka_robot_interfaces::srv::JointToCartesian>
  : std::integral_constant<
    bool,
    has_bounded_size<jaka_robot_interfaces::srv::JointToCartesian_Request>::value &&
    has_bounded_size<jaka_robot_interfaces::srv::JointToCartesian_Response>::value
  >
{
};

template<>
struct is_service<jaka_robot_interfaces::srv::JointToCartesian>
  : std::true_type
{
};

template<>
struct is_service_request<jaka_robot_interfaces::srv::JointToCartesian_Request>
  : std::true_type
{
};

template<>
struct is_service_response<jaka_robot_interfaces::srv::JointToCartesian_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // JAKA_ROBOT_INTERFACES__SRV__DETAIL__JOINT_TO_CARTESIAN__TRAITS_HPP_
