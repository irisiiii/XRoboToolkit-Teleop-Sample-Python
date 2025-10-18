// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from jaka_robot_interfaces:msg/CartesianPose.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__MSG__DETAIL__CARTESIAN_POSE__BUILDER_HPP_
#define JAKA_ROBOT_INTERFACES__MSG__DETAIL__CARTESIAN_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "jaka_robot_interfaces/msg/detail/cartesian_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace jaka_robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_CartesianPose_rz
{
public:
  explicit Init_CartesianPose_rz(::jaka_robot_interfaces::msg::CartesianPose & msg)
  : msg_(msg)
  {}
  ::jaka_robot_interfaces::msg::CartesianPose rz(::jaka_robot_interfaces::msg::CartesianPose::_rz_type arg)
  {
    msg_.rz = std::move(arg);
    return std::move(msg_);
  }

private:
  ::jaka_robot_interfaces::msg::CartesianPose msg_;
};

class Init_CartesianPose_ry
{
public:
  explicit Init_CartesianPose_ry(::jaka_robot_interfaces::msg::CartesianPose & msg)
  : msg_(msg)
  {}
  Init_CartesianPose_rz ry(::jaka_robot_interfaces::msg::CartesianPose::_ry_type arg)
  {
    msg_.ry = std::move(arg);
    return Init_CartesianPose_rz(msg_);
  }

private:
  ::jaka_robot_interfaces::msg::CartesianPose msg_;
};

class Init_CartesianPose_rx
{
public:
  explicit Init_CartesianPose_rx(::jaka_robot_interfaces::msg::CartesianPose & msg)
  : msg_(msg)
  {}
  Init_CartesianPose_ry rx(::jaka_robot_interfaces::msg::CartesianPose::_rx_type arg)
  {
    msg_.rx = std::move(arg);
    return Init_CartesianPose_ry(msg_);
  }

private:
  ::jaka_robot_interfaces::msg::CartesianPose msg_;
};

class Init_CartesianPose_z
{
public:
  explicit Init_CartesianPose_z(::jaka_robot_interfaces::msg::CartesianPose & msg)
  : msg_(msg)
  {}
  Init_CartesianPose_rx z(::jaka_robot_interfaces::msg::CartesianPose::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_CartesianPose_rx(msg_);
  }

private:
  ::jaka_robot_interfaces::msg::CartesianPose msg_;
};

class Init_CartesianPose_y
{
public:
  explicit Init_CartesianPose_y(::jaka_robot_interfaces::msg::CartesianPose & msg)
  : msg_(msg)
  {}
  Init_CartesianPose_z y(::jaka_robot_interfaces::msg::CartesianPose::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_CartesianPose_z(msg_);
  }

private:
  ::jaka_robot_interfaces::msg::CartesianPose msg_;
};

class Init_CartesianPose_x
{
public:
  Init_CartesianPose_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CartesianPose_y x(::jaka_robot_interfaces::msg::CartesianPose::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_CartesianPose_y(msg_);
  }

private:
  ::jaka_robot_interfaces::msg::CartesianPose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::jaka_robot_interfaces::msg::CartesianPose>()
{
  return jaka_robot_interfaces::msg::builder::Init_CartesianPose_x();
}

}  // namespace jaka_robot_interfaces

#endif  // JAKA_ROBOT_INTERFACES__MSG__DETAIL__CARTESIAN_POSE__BUILDER_HPP_
