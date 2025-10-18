// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from jaka_robot_interfaces:srv/MultiMovL.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__SRV__DETAIL__MULTI_MOV_L__TRAITS_HPP_
#define JAKA_ROBOT_INTERFACES__SRV__DETAIL__MULTI_MOV_L__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "jaka_robot_interfaces/srv/detail/multi_mov_l__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'left_move_mode'
// Member 'right_move_mode'
#include "jaka_robot_interfaces/msg/detail/move_mode__traits.hpp"
// Member 'end_pos_left'
// Member 'end_pos_right'
#include "jaka_robot_interfaces/msg/detail/cartesian_pose__traits.hpp"

namespace jaka_robot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const MultiMovL_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: robot_id
  {
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << ", ";
  }

  // member: left_move_mode
  {
    out << "left_move_mode: ";
    to_flow_style_yaml(msg.left_move_mode, out);
    out << ", ";
  }

  // member: right_move_mode
  {
    out << "right_move_mode: ";
    to_flow_style_yaml(msg.right_move_mode, out);
    out << ", ";
  }

  // member: is_block
  {
    out << "is_block: ";
    rosidl_generator_traits::value_to_yaml(msg.is_block, out);
    out << ", ";
  }

  // member: end_pos_left
  {
    out << "end_pos_left: ";
    to_flow_style_yaml(msg.end_pos_left, out);
    out << ", ";
  }

  // member: end_pos_right
  {
    out << "end_pos_right: ";
    to_flow_style_yaml(msg.end_pos_right, out);
    out << ", ";
  }

  // member: vel
  {
    if (msg.vel.size() == 0) {
      out << "vel: []";
    } else {
      out << "vel: [";
      size_t pending_items = msg.vel.size();
      for (auto item : msg.vel) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: acc
  {
    if (msg.acc.size() == 0) {
      out << "acc: []";
    } else {
      out << "acc: [";
      size_t pending_items = msg.acc.size();
      for (auto item : msg.acc) {
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
  const MultiMovL_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: robot_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_id: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_id, out);
    out << "\n";
  }

  // member: left_move_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "left_move_mode:\n";
    to_block_style_yaml(msg.left_move_mode, out, indentation + 2);
  }

  // member: right_move_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "right_move_mode:\n";
    to_block_style_yaml(msg.right_move_mode, out, indentation + 2);
  }

  // member: is_block
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_block: ";
    rosidl_generator_traits::value_to_yaml(msg.is_block, out);
    out << "\n";
  }

  // member: end_pos_left
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "end_pos_left:\n";
    to_block_style_yaml(msg.end_pos_left, out, indentation + 2);
  }

  // member: end_pos_right
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "end_pos_right:\n";
    to_block_style_yaml(msg.end_pos_right, out, indentation + 2);
  }

  // member: vel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.vel.size() == 0) {
      out << "vel: []\n";
    } else {
      out << "vel:\n";
      for (auto item : msg.vel) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: acc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.acc.size() == 0) {
      out << "acc: []\n";
    } else {
      out << "acc:\n";
      for (auto item : msg.acc) {
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

inline std::string to_yaml(const MultiMovL_Request & msg, bool use_flow_style = false)
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
  const jaka_robot_interfaces::srv::MultiMovL_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  jaka_robot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use jaka_robot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const jaka_robot_interfaces::srv::MultiMovL_Request & msg)
{
  return jaka_robot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<jaka_robot_interfaces::srv::MultiMovL_Request>()
{
  return "jaka_robot_interfaces::srv::MultiMovL_Request";
}

template<>
inline const char * name<jaka_robot_interfaces::srv::MultiMovL_Request>()
{
  return "jaka_robot_interfaces/srv/MultiMovL_Request";
}

template<>
struct has_fixed_size<jaka_robot_interfaces::srv::MultiMovL_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<jaka_robot_interfaces::srv::MultiMovL_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<jaka_robot_interfaces::srv::MultiMovL_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace jaka_robot_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const MultiMovL_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: ret_code
  {
    out << "ret_code: ";
    rosidl_generator_traits::value_to_yaml(msg.ret_code, out);
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
  const MultiMovL_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: ret_code
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ret_code: ";
    rosidl_generator_traits::value_to_yaml(msg.ret_code, out);
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

inline std::string to_yaml(const MultiMovL_Response & msg, bool use_flow_style = false)
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
  const jaka_robot_interfaces::srv::MultiMovL_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  jaka_robot_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use jaka_robot_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const jaka_robot_interfaces::srv::MultiMovL_Response & msg)
{
  return jaka_robot_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<jaka_robot_interfaces::srv::MultiMovL_Response>()
{
  return "jaka_robot_interfaces::srv::MultiMovL_Response";
}

template<>
inline const char * name<jaka_robot_interfaces::srv::MultiMovL_Response>()
{
  return "jaka_robot_interfaces/srv/MultiMovL_Response";
}

template<>
struct has_fixed_size<jaka_robot_interfaces::srv::MultiMovL_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<jaka_robot_interfaces::srv::MultiMovL_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<jaka_robot_interfaces::srv::MultiMovL_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<jaka_robot_interfaces::srv::MultiMovL>()
{
  return "jaka_robot_interfaces::srv::MultiMovL";
}

template<>
inline const char * name<jaka_robot_interfaces::srv::MultiMovL>()
{
  return "jaka_robot_interfaces/srv/MultiMovL";
}

template<>
struct has_fixed_size<jaka_robot_interfaces::srv::MultiMovL>
  : std::integral_constant<
    bool,
    has_fixed_size<jaka_robot_interfaces::srv::MultiMovL_Request>::value &&
    has_fixed_size<jaka_robot_interfaces::srv::MultiMovL_Response>::value
  >
{
};

template<>
struct has_bounded_size<jaka_robot_interfaces::srv::MultiMovL>
  : std::integral_constant<
    bool,
    has_bounded_size<jaka_robot_interfaces::srv::MultiMovL_Request>::value &&
    has_bounded_size<jaka_robot_interfaces::srv::MultiMovL_Response>::value
  >
{
};

template<>
struct is_service<jaka_robot_interfaces::srv::MultiMovL>
  : std::true_type
{
};

template<>
struct is_service_request<jaka_robot_interfaces::srv::MultiMovL_Request>
  : std::true_type
{
};

template<>
struct is_service_response<jaka_robot_interfaces::srv::MultiMovL_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // JAKA_ROBOT_INTERFACES__SRV__DETAIL__MULTI_MOV_L__TRAITS_HPP_
