// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from jaka_robot_interfaces:srv/ClearError.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__SRV__DETAIL__CLEAR_ERROR__BUILDER_HPP_
#define JAKA_ROBOT_INTERFACES__SRV__DETAIL__CLEAR_ERROR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "jaka_robot_interfaces/srv/detail/clear_error__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace jaka_robot_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::jaka_robot_interfaces::srv::ClearError_Request>()
{
  return ::jaka_robot_interfaces::srv::ClearError_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace jaka_robot_interfaces


namespace jaka_robot_interfaces
{

namespace srv
{

namespace builder
{

class Init_ClearError_Response_message
{
public:
  explicit Init_ClearError_Response_message(::jaka_robot_interfaces::srv::ClearError_Response & msg)
  : msg_(msg)
  {}
  ::jaka_robot_interfaces::srv::ClearError_Response message(::jaka_robot_interfaces::srv::ClearError_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jaka_robot_interfaces::srv::ClearError_Response msg_;
};

class Init_ClearError_Response_success
{
public:
  Init_ClearError_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ClearError_Response_message success(::jaka_robot_interfaces::srv::ClearError_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_ClearError_Response_message(msg_);
  }

private:
  ::jaka_robot_interfaces::srv::ClearError_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::jaka_robot_interfaces::srv::ClearError_Response>()
{
  return jaka_robot_interfaces::srv::builder::Init_ClearError_Response_success();
}

}  // namespace jaka_robot_interfaces

#endif  // JAKA_ROBOT_INTERFACES__SRV__DETAIL__CLEAR_ERROR__BUILDER_HPP_
