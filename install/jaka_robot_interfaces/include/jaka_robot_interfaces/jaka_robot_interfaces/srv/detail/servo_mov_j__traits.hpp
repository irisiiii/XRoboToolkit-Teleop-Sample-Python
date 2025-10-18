// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from jaka_robot_interfaces:srv/ServoMovJ.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__SRV__DETAIL__SERVO_MOV_J__TRAITS_HPP_
#define JAKA_ROBOT_INTERFACES__SRV__DETAIL__SERVO_MOV_J__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "jaka_robot_interfaces/srv/detail/servo_mov_j__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'joint_pos_left'
// Member 'joint_pos_right'
#include "jaka_robot_interfaces/msg/detail/joint_value__traits.hpp"
// Member 'move_mode'
#include "jaka_robot_interfaces/msg/detail/move_mode__traits.hpp"

namespace jaka_robot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ServoMovJ_Request & msg,
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
    out << ", ";
  }

  // member: move_mode
  {
    out << "move_mode: ";
    to_flow_style_yaml(msg.move_mode, out);
    out << ", ";
  }

  // member: step_num
  {
    out << "step_num: ";
    rosidl_generator_traits::value_to_yaml(msg.step_num, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ServoMovJ_Request & msg,
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

  // member: move_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "move_mode:\n";
    to_block_style_yaml(msg.move_mode, out, indentation + 2);
  }

  // member: step_num
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "step_num: ";
    rosidl_generator_traits::value_to_yaml(msg.step_num, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ServoMovJ_Request & msg, bool use_flow_style = false)
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
  const jaka_robot_interfaces::srv::ServoMovJ_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  jaka_robot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use jaka_robot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const jaka_robot_interfaces::srv::ServoMovJ_Request & msg)
{
  return jaka_robot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<jaka_robot_interfaces::srv::ServoMovJ_Request>()
{
  return "jaka_robot_interfaces::srv::ServoMovJ_Request";
}

template<>
inline const char * name<jaka_robot_interfaces::srv::ServoMovJ_Request>()
{
  return "jaka_robot_interfaces/srv/ServoMovJ_Request";
}

template<>
struct has_fixed_size<jaka_robot_interfaces::srv::ServoMovJ_Request>
  : std::integral_constant<bool, has_fixed_size<jaka_robot_interfaces::msg::JointValue>::value && has_fixed_size<jaka_robot_interfaces::msg::MoveMode>::value> {};

template<>
struct has_bounded_size<jaka_robot_interfaces::srv::ServoMovJ_Request>
  : std::integral_constant<bool, has_bounded_size<jaka_robot_interfaces::msg::JointValue>::value && has_bounded_size<jaka_robot_interfaces::msg::MoveMode>::value> {};

template<>
struct is_message<jaka_robot_interfaces::srv::ServoMovJ_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace jaka_robot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ServoMovJ_Response & msg,
  std::ostream & out)
{
  out << "{";
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
  const ServoMovJ_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
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

inline std::string to_yaml(const ServoMovJ_Response & msg, bool use_flow_style = false)
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
  const jaka_robot_interfaces::srv::ServoMovJ_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  jaka_robot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use jaka_robot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const jaka_robot_interfaces::srv::ServoMovJ_Response & msg)
{
  return jaka_robot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<jaka_robot_interfaces::srv::ServoMovJ_Response>()
{
  return "jaka_robot_interfaces::srv::ServoMovJ_Response";
}

template<>
inline const char * name<jaka_robot_interfaces::srv::ServoMovJ_Response>()
{
  return "jaka_robot_interfaces/srv/ServoMovJ_Response";
}

template<>
struct has_fixed_size<jaka_robot_interfaces::srv::ServoMovJ_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<jaka_robot_interfaces::srv::ServoMovJ_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<jaka_robot_interfaces::srv::ServoMovJ_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<jaka_robot_interfaces::srv::ServoMovJ>()
{
  return "jaka_robot_interfaces::srv::ServoMovJ";
}

template<>
inline const char * name<jaka_robot_interfaces::srv::ServoMovJ>()
{
  return "jaka_robot_interfaces/srv/ServoMovJ";
}

template<>
struct has_fixed_size<jaka_robot_interfaces::srv::ServoMovJ>
  : std::integral_constant<
    bool,
    has_fixed_size<jaka_robot_interfaces::srv::ServoMovJ_Request>::value &&
    has_fixed_size<jaka_robot_interfaces::srv::ServoMovJ_Response>::value
  >
{
};

template<>
struct has_bounded_size<jaka_robot_interfaces::srv::ServoMovJ>
  : std::integral_constant<
    bool,
    has_bounded_size<jaka_robot_interfaces::srv::ServoMovJ_Request>::value &&
    has_bounded_size<jaka_robot_interfaces::srv::ServoMovJ_Response>::value
  >
{
};

template<>
struct is_service<jaka_robot_interfaces::srv::ServoMovJ>
  : std::true_type
{
};

template<>
struct is_service_request<jaka_robot_interfaces::srv::ServoMovJ_Request>
  : std::true_type
{
};

template<>
struct is_service_response<jaka_robot_interfaces::srv::ServoMovJ_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // JAKA_ROBOT_INTERFACES__SRV__DETAIL__SERVO_MOV_J__TRAITS_HPP_
