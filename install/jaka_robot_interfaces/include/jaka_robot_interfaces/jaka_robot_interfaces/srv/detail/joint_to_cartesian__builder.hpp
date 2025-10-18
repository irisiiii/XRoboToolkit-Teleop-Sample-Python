// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from jaka_robot_interfaces:srv/JointToCartesian.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__SRV__DETAIL__JOINT_TO_CARTESIAN__BUILDER_HPP_
#define JAKA_ROBOT_INTERFACES__SRV__DETAIL__JOINT_TO_CARTESIAN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "jaka_robot_interfaces/srv/detail/joint_to_cartesian__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace jaka_robot_interfaces
{

namespace srv
{

namespace builder
{

class Init_JointToCartesian_Request_robot_id
{
public:
  explicit Init_JointToCartesian_Request_robot_id(::jaka_robot_interfaces::srv::JointToCartesian_Request & msg)
  : msg_(msg)
  {}
  ::jaka_robot_interfaces::srv::JointToCartesian_Request robot_id(::jaka_robot_interfaces::srv::JointToCartesian_Request::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jaka_robot_interfaces::srv::JointToCartesian_Request msg_;
};

class Init_JointToCartesian_Request_joint_value
{
public:
  Init_JointToCartesian_Request_joint_value()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_JointToCartesian_Request_robot_id joint_value(::jaka_robot_interfaces::srv::JointToCartesian_Request::_joint_value_type arg)
  {
    msg_.joint_value = std::move(arg);
    return Init_JointToCartesian_Request_robot_id(msg_);
  }

private:
  ::jaka_robot_interfaces::srv::JointToCartesian_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::jaka_robot_interfaces::srv::JointToCartesian_Request>()
{
  return jaka_robot_interfaces::srv::builder::Init_JointToCartesian_Request_joint_value();
}

}  // namespace jaka_robot_interfaces


namespace jaka_robot_interfaces
{

namespace srv
{

namespace builder
{

class Init_JointToCartesian_Response_message
{
public:
  explicit Init_JointToCartesian_Response_message(::jaka_robot_interfaces::srv::JointToCartesian_Response & msg)
  : msg_(msg)
  {}
  ::jaka_robot_interfaces::srv::JointToCartesian_Response message(::jaka_robot_interfaces::srv::JointToCartesian_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jaka_robot_interfaces::srv::JointToCartesian_Response msg_;
};

class Init_JointToCartesian_Response_success
{
public:
  explicit Init_JointToCartesian_Response_success(::jaka_robot_interfaces::srv::JointToCartesian_Response & msg)
  : msg_(msg)
  {}
  Init_JointToCartesian_Response_message success(::jaka_robot_interfaces::srv::JointToCartesian_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_JointToCartesian_Response_message(msg_);
  }

private:
  ::jaka_robot_interfaces::srv::JointToCartesian_Response msg_;
};

class Init_JointToCartesian_Response_cartesian_pose
{
public:
  Init_JointToCartesian_Response_cartesian_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_JointToCartesian_Response_success cartesian_pose(::jaka_robot_interfaces::srv::JointToCartesian_Response::_cartesian_pose_type arg)
  {
    msg_.cartesian_pose = std::move(arg);
    return Init_JointToCartesian_Response_success(msg_);
  }

private:
  ::jaka_robot_interfaces::srv::JointToCartesian_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::jaka_robot_interfaces::srv::JointToCartesian_Response>()
{
  return jaka_robot_interfaces::srv::builder::Init_JointToCartesian_Response_cartesian_pose();
}

}  // namespace jaka_robot_interfaces

#endif  // JAKA_ROBOT_INTERFACES__SRV__DETAIL__JOINT_TO_CARTESIAN__BUILDER_HPP_
