// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from jaka_robot_interfaces:msg/RobotStateDual.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__MSG__DETAIL__ROBOT_STATE_DUAL__BUILDER_HPP_
#define JAKA_ROBOT_INTERFACES__MSG__DETAIL__ROBOT_STATE_DUAL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "jaka_robot_interfaces/msg/detail/robot_state_dual__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace jaka_robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_RobotStateDual_end_pose_right
{
public:
  explicit Init_RobotStateDual_end_pose_right(::jaka_robot_interfaces::msg::RobotStateDual & msg)
  : msg_(msg)
  {}
  ::jaka_robot_interfaces::msg::RobotStateDual end_pose_right(::jaka_robot_interfaces::msg::RobotStateDual::_end_pose_right_type arg)
  {
    msg_.end_pose_right = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jaka_robot_interfaces::msg::RobotStateDual msg_;
};

class Init_RobotStateDual_end_pose_left
{
public:
  explicit Init_RobotStateDual_end_pose_left(::jaka_robot_interfaces::msg::RobotStateDual & msg)
  : msg_(msg)
  {}
  Init_RobotStateDual_end_pose_right end_pose_left(::jaka_robot_interfaces::msg::RobotStateDual::_end_pose_left_type arg)
  {
    msg_.end_pose_left = std::move(arg);
    return Init_RobotStateDual_end_pose_right(msg_);
  }

private:
  ::jaka_robot_interfaces::msg::RobotStateDual msg_;
};

class Init_RobotStateDual_joint_pos_right
{
public:
  explicit Init_RobotStateDual_joint_pos_right(::jaka_robot_interfaces::msg::RobotStateDual & msg)
  : msg_(msg)
  {}
  Init_RobotStateDual_end_pose_left joint_pos_right(::jaka_robot_interfaces::msg::RobotStateDual::_joint_pos_right_type arg)
  {
    msg_.joint_pos_right = std::move(arg);
    return Init_RobotStateDual_end_pose_left(msg_);
  }

private:
  ::jaka_robot_interfaces::msg::RobotStateDual msg_;
};

class Init_RobotStateDual_joint_pos_left
{
public:
  Init_RobotStateDual_joint_pos_left()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotStateDual_joint_pos_right joint_pos_left(::jaka_robot_interfaces::msg::RobotStateDual::_joint_pos_left_type arg)
  {
    msg_.joint_pos_left = std::move(arg);
    return Init_RobotStateDual_joint_pos_right(msg_);
  }

private:
  ::jaka_robot_interfaces::msg::RobotStateDual msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::jaka_robot_interfaces::msg::RobotStateDual>()
{
  return jaka_robot_interfaces::msg::builder::Init_RobotStateDual_joint_pos_left();
}

}  // namespace jaka_robot_interfaces

#endif  // JAKA_ROBOT_INTERFACES__MSG__DETAIL__ROBOT_STATE_DUAL__BUILDER_HPP_
