// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from jaka_robot_interfaces:msg/ServoCartCommand.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__MSG__DETAIL__SERVO_CART_COMMAND__BUILDER_HPP_
#define JAKA_ROBOT_INTERFACES__MSG__DETAIL__SERVO_CART_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "jaka_robot_interfaces/msg/detail/servo_cart_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace jaka_robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_ServoCartCommand_end_pose_right
{
public:
  explicit Init_ServoCartCommand_end_pose_right(::jaka_robot_interfaces::msg::ServoCartCommand & msg)
  : msg_(msg)
  {}
  ::jaka_robot_interfaces::msg::ServoCartCommand end_pose_right(::jaka_robot_interfaces::msg::ServoCartCommand::_end_pose_right_type arg)
  {
    msg_.end_pose_right = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jaka_robot_interfaces::msg::ServoCartCommand msg_;
};

class Init_ServoCartCommand_end_pose_left
{
public:
  Init_ServoCartCommand_end_pose_left()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ServoCartCommand_end_pose_right end_pose_left(::jaka_robot_interfaces::msg::ServoCartCommand::_end_pose_left_type arg)
  {
    msg_.end_pose_left = std::move(arg);
    return Init_ServoCartCommand_end_pose_right(msg_);
  }

private:
  ::jaka_robot_interfaces::msg::ServoCartCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::jaka_robot_interfaces::msg::ServoCartCommand>()
{
  return jaka_robot_interfaces::msg::builder::Init_ServoCartCommand_end_pose_left();
}

}  // namespace jaka_robot_interfaces

#endif  // JAKA_ROBOT_INTERFACES__MSG__DETAIL__SERVO_CART_COMMAND__BUILDER_HPP_
