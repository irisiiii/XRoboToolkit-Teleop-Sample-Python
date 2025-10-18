// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from jaka_robot_interfaces:msg/ServoJointCommand.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__MSG__DETAIL__SERVO_JOINT_COMMAND__BUILDER_HPP_
#define JAKA_ROBOT_INTERFACES__MSG__DETAIL__SERVO_JOINT_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "jaka_robot_interfaces/msg/detail/servo_joint_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace jaka_robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_ServoJointCommand_joint_pos_right
{
public:
  explicit Init_ServoJointCommand_joint_pos_right(::jaka_robot_interfaces::msg::ServoJointCommand & msg)
  : msg_(msg)
  {}
  ::jaka_robot_interfaces::msg::ServoJointCommand joint_pos_right(::jaka_robot_interfaces::msg::ServoJointCommand::_joint_pos_right_type arg)
  {
    msg_.joint_pos_right = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jaka_robot_interfaces::msg::ServoJointCommand msg_;
};

class Init_ServoJointCommand_joint_pos_left
{
public:
  Init_ServoJointCommand_joint_pos_left()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ServoJointCommand_joint_pos_right joint_pos_left(::jaka_robot_interfaces::msg::ServoJointCommand::_joint_pos_left_type arg)
  {
    msg_.joint_pos_left = std::move(arg);
    return Init_ServoJointCommand_joint_pos_right(msg_);
  }

private:
  ::jaka_robot_interfaces::msg::ServoJointCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::jaka_robot_interfaces::msg::ServoJointCommand>()
{
  return jaka_robot_interfaces::msg::builder::Init_ServoJointCommand_joint_pos_left();
}

}  // namespace jaka_robot_interfaces

#endif  // JAKA_ROBOT_INTERFACES__MSG__DETAIL__SERVO_JOINT_COMMAND__BUILDER_HPP_
