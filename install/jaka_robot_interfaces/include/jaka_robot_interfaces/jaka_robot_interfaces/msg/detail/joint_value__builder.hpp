// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from jaka_robot_interfaces:msg/JointValue.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__MSG__DETAIL__JOINT_VALUE__BUILDER_HPP_
#define JAKA_ROBOT_INTERFACES__MSG__DETAIL__JOINT_VALUE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "jaka_robot_interfaces/msg/detail/joint_value__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace jaka_robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_JointValue_joint_values
{
public:
  Init_JointValue_joint_values()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::jaka_robot_interfaces::msg::JointValue joint_values(::jaka_robot_interfaces::msg::JointValue::_joint_values_type arg)
  {
    msg_.joint_values = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jaka_robot_interfaces::msg::JointValue msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::jaka_robot_interfaces::msg::JointValue>()
{
  return jaka_robot_interfaces::msg::builder::Init_JointValue_joint_values();
}

}  // namespace jaka_robot_interfaces

#endif  // JAKA_ROBOT_INTERFACES__MSG__DETAIL__JOINT_VALUE__BUILDER_HPP_
