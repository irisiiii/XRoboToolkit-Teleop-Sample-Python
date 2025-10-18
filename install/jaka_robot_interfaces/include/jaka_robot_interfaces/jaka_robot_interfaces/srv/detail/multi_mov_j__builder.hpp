// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from jaka_robot_interfaces:srv/MultiMovJ.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__SRV__DETAIL__MULTI_MOV_J__BUILDER_HPP_
#define JAKA_ROBOT_INTERFACES__SRV__DETAIL__MULTI_MOV_J__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "jaka_robot_interfaces/srv/detail/multi_mov_j__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace jaka_robot_interfaces
{

namespace srv
{

namespace builder
{

class Init_MultiMovJ_Request_acc
{
public:
  explicit Init_MultiMovJ_Request_acc(::jaka_robot_interfaces::srv::MultiMovJ_Request & msg)
  : msg_(msg)
  {}
  ::jaka_robot_interfaces::srv::MultiMovJ_Request acc(::jaka_robot_interfaces::srv::MultiMovJ_Request::_acc_type arg)
  {
    msg_.acc = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jaka_robot_interfaces::srv::MultiMovJ_Request msg_;
};

class Init_MultiMovJ_Request_vel
{
public:
  explicit Init_MultiMovJ_Request_vel(::jaka_robot_interfaces::srv::MultiMovJ_Request & msg)
  : msg_(msg)
  {}
  Init_MultiMovJ_Request_acc vel(::jaka_robot_interfaces::srv::MultiMovJ_Request::_vel_type arg)
  {
    msg_.vel = std::move(arg);
    return Init_MultiMovJ_Request_acc(msg_);
  }

private:
  ::jaka_robot_interfaces::srv::MultiMovJ_Request msg_;
};

class Init_MultiMovJ_Request_joint_pos_right
{
public:
  explicit Init_MultiMovJ_Request_joint_pos_right(::jaka_robot_interfaces::srv::MultiMovJ_Request & msg)
  : msg_(msg)
  {}
  Init_MultiMovJ_Request_vel joint_pos_right(::jaka_robot_interfaces::srv::MultiMovJ_Request::_joint_pos_right_type arg)
  {
    msg_.joint_pos_right = std::move(arg);
    return Init_MultiMovJ_Request_vel(msg_);
  }

private:
  ::jaka_robot_interfaces::srv::MultiMovJ_Request msg_;
};

class Init_MultiMovJ_Request_joint_pos_left
{
public:
  explicit Init_MultiMovJ_Request_joint_pos_left(::jaka_robot_interfaces::srv::MultiMovJ_Request & msg)
  : msg_(msg)
  {}
  Init_MultiMovJ_Request_joint_pos_right joint_pos_left(::jaka_robot_interfaces::srv::MultiMovJ_Request::_joint_pos_left_type arg)
  {
    msg_.joint_pos_left = std::move(arg);
    return Init_MultiMovJ_Request_joint_pos_right(msg_);
  }

private:
  ::jaka_robot_interfaces::srv::MultiMovJ_Request msg_;
};

class Init_MultiMovJ_Request_is_block
{
public:
  explicit Init_MultiMovJ_Request_is_block(::jaka_robot_interfaces::srv::MultiMovJ_Request & msg)
  : msg_(msg)
  {}
  Init_MultiMovJ_Request_joint_pos_left is_block(::jaka_robot_interfaces::srv::MultiMovJ_Request::_is_block_type arg)
  {
    msg_.is_block = std::move(arg);
    return Init_MultiMovJ_Request_joint_pos_left(msg_);
  }

private:
  ::jaka_robot_interfaces::srv::MultiMovJ_Request msg_;
};

class Init_MultiMovJ_Request_right_move_mode
{
public:
  explicit Init_MultiMovJ_Request_right_move_mode(::jaka_robot_interfaces::srv::MultiMovJ_Request & msg)
  : msg_(msg)
  {}
  Init_MultiMovJ_Request_is_block right_move_mode(::jaka_robot_interfaces::srv::MultiMovJ_Request::_right_move_mode_type arg)
  {
    msg_.right_move_mode = std::move(arg);
    return Init_MultiMovJ_Request_is_block(msg_);
  }

private:
  ::jaka_robot_interfaces::srv::MultiMovJ_Request msg_;
};

class Init_MultiMovJ_Request_left_move_mode
{
public:
  explicit Init_MultiMovJ_Request_left_move_mode(::jaka_robot_interfaces::srv::MultiMovJ_Request & msg)
  : msg_(msg)
  {}
  Init_MultiMovJ_Request_right_move_mode left_move_mode(::jaka_robot_interfaces::srv::MultiMovJ_Request::_left_move_mode_type arg)
  {
    msg_.left_move_mode = std::move(arg);
    return Init_MultiMovJ_Request_right_move_mode(msg_);
  }

private:
  ::jaka_robot_interfaces::srv::MultiMovJ_Request msg_;
};

class Init_MultiMovJ_Request_robot_id
{
public:
  Init_MultiMovJ_Request_robot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MultiMovJ_Request_left_move_mode robot_id(::jaka_robot_interfaces::srv::MultiMovJ_Request::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_MultiMovJ_Request_left_move_mode(msg_);
  }

private:
  ::jaka_robot_interfaces::srv::MultiMovJ_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::jaka_robot_interfaces::srv::MultiMovJ_Request>()
{
  return jaka_robot_interfaces::srv::builder::Init_MultiMovJ_Request_robot_id();
}

}  // namespace jaka_robot_interfaces


namespace jaka_robot_interfaces
{

namespace srv
{

namespace builder
{

class Init_MultiMovJ_Response_success
{
public:
  explicit Init_MultiMovJ_Response_success(::jaka_robot_interfaces::srv::MultiMovJ_Response & msg)
  : msg_(msg)
  {}
  ::jaka_robot_interfaces::srv::MultiMovJ_Response success(::jaka_robot_interfaces::srv::MultiMovJ_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jaka_robot_interfaces::srv::MultiMovJ_Response msg_;
};

class Init_MultiMovJ_Response_message
{
public:
  explicit Init_MultiMovJ_Response_message(::jaka_robot_interfaces::srv::MultiMovJ_Response & msg)
  : msg_(msg)
  {}
  Init_MultiMovJ_Response_success message(::jaka_robot_interfaces::srv::MultiMovJ_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_MultiMovJ_Response_success(msg_);
  }

private:
  ::jaka_robot_interfaces::srv::MultiMovJ_Response msg_;
};

class Init_MultiMovJ_Response_ret_code
{
public:
  Init_MultiMovJ_Response_ret_code()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MultiMovJ_Response_message ret_code(::jaka_robot_interfaces::srv::MultiMovJ_Response::_ret_code_type arg)
  {
    msg_.ret_code = std::move(arg);
    return Init_MultiMovJ_Response_message(msg_);
  }

private:
  ::jaka_robot_interfaces::srv::MultiMovJ_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::jaka_robot_interfaces::srv::MultiMovJ_Response>()
{
  return jaka_robot_interfaces::srv::builder::Init_MultiMovJ_Response_ret_code();
}

}  // namespace jaka_robot_interfaces

#endif  // JAKA_ROBOT_INTERFACES__SRV__DETAIL__MULTI_MOV_J__BUILDER_HPP_
