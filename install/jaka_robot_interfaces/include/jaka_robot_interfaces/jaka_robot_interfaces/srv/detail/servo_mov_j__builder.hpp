// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from jaka_robot_interfaces:srv/ServoMovJ.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__SRV__DETAIL__SERVO_MOV_J__BUILDER_HPP_
#define JAKA_ROBOT_INTERFACES__SRV__DETAIL__SERVO_MOV_J__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "jaka_robot_interfaces/srv/detail/servo_mov_j__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace jaka_robot_interfaces
{

namespace srv
{

namespace builder
{

class Init_ServoMovJ_Request_step_num
{
public:
  explicit Init_ServoMovJ_Request_step_num(::jaka_robot_interfaces::srv::ServoMovJ_Request & msg)
  : msg_(msg)
  {}
  ::jaka_robot_interfaces::srv::ServoMovJ_Request step_num(::jaka_robot_interfaces::srv::ServoMovJ_Request::_step_num_type arg)
  {
    msg_.step_num = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jaka_robot_interfaces::srv::ServoMovJ_Request msg_;
};

class Init_ServoMovJ_Request_move_mode
{
public:
  explicit Init_ServoMovJ_Request_move_mode(::jaka_robot_interfaces::srv::ServoMovJ_Request & msg)
  : msg_(msg)
  {}
  Init_ServoMovJ_Request_step_num move_mode(::jaka_robot_interfaces::srv::ServoMovJ_Request::_move_mode_type arg)
  {
    msg_.move_mode = std::move(arg);
    return Init_ServoMovJ_Request_step_num(msg_);
  }

private:
  ::jaka_robot_interfaces::srv::ServoMovJ_Request msg_;
};

class Init_ServoMovJ_Request_joint_pos_right
{
public:
  explicit Init_ServoMovJ_Request_joint_pos_right(::jaka_robot_interfaces::srv::ServoMovJ_Request & msg)
  : msg_(msg)
  {}
  Init_ServoMovJ_Request_move_mode joint_pos_right(::jaka_robot_interfaces::srv::ServoMovJ_Request::_joint_pos_right_type arg)
  {
    msg_.joint_pos_right = std::move(arg);
    return Init_ServoMovJ_Request_move_mode(msg_);
  }

private:
  ::jaka_robot_interfaces::srv::ServoMovJ_Request msg_;
};

class Init_ServoMovJ_Request_joint_pos_left
{
public:
  Init_ServoMovJ_Request_joint_pos_left()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ServoMovJ_Request_joint_pos_right joint_pos_left(::jaka_robot_interfaces::srv::ServoMovJ_Request::_joint_pos_left_type arg)
  {
    msg_.joint_pos_left = std::move(arg);
    return Init_ServoMovJ_Request_joint_pos_right(msg_);
  }

private:
  ::jaka_robot_interfaces::srv::ServoMovJ_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::jaka_robot_interfaces::srv::ServoMovJ_Request>()
{
  return jaka_robot_interfaces::srv::builder::Init_ServoMovJ_Request_joint_pos_left();
}

}  // namespace jaka_robot_interfaces


namespace jaka_robot_interfaces
{

namespace srv
{

namespace builder
{

class Init_ServoMovJ_Response_message
{
public:
  explicit Init_ServoMovJ_Response_message(::jaka_robot_interfaces::srv::ServoMovJ_Response & msg)
  : msg_(msg)
  {}
  ::jaka_robot_interfaces::srv::ServoMovJ_Response message(::jaka_robot_interfaces::srv::ServoMovJ_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jaka_robot_interfaces::srv::ServoMovJ_Response msg_;
};

class Init_ServoMovJ_Response_success
{
public:
  Init_ServoMovJ_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ServoMovJ_Response_message success(::jaka_robot_interfaces::srv::ServoMovJ_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ServoMovJ_Response_message(msg_);
  }

private:
  ::jaka_robot_interfaces::srv::ServoMovJ_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::jaka_robot_interfaces::srv::ServoMovJ_Response>()
{
  return jaka_robot_interfaces::srv::builder::Init_ServoMovJ_Response_success();
}

}  // namespace jaka_robot_interfaces

#endif  // JAKA_ROBOT_INTERFACES__SRV__DETAIL__SERVO_MOV_J__BUILDER_HPP_
