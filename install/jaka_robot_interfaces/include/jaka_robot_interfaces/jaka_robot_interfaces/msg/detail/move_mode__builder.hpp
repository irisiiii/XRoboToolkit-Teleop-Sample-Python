// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from jaka_robot_interfaces:msg/MoveMode.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__MSG__DETAIL__MOVE_MODE__BUILDER_HPP_
#define JAKA_ROBOT_INTERFACES__MSG__DETAIL__MOVE_MODE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "jaka_robot_interfaces/msg/detail/move_mode__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace jaka_robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_MoveMode_mode
{
public:
  Init_MoveMode_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::jaka_robot_interfaces::msg::MoveMode mode(::jaka_robot_interfaces::msg::MoveMode::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jaka_robot_interfaces::msg::MoveMode msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::jaka_robot_interfaces::msg::MoveMode>()
{
  return jaka_robot_interfaces::msg::builder::Init_MoveMode_mode();
}

}  // namespace jaka_robot_interfaces

#endif  // JAKA_ROBOT_INTERFACES__MSG__DETAIL__MOVE_MODE__BUILDER_HPP_
