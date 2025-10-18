// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from jaka_robot_interfaces:msg/JointValue.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__MSG__DETAIL__JOINT_VALUE__STRUCT_HPP_
#define JAKA_ROBOT_INTERFACES__MSG__DETAIL__JOINT_VALUE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__jaka_robot_interfaces__msg__JointValue __attribute__((deprecated))
#else
# define DEPRECATED__jaka_robot_interfaces__msg__JointValue __declspec(deprecated)
#endif

namespace jaka_robot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct JointValue_
{
  using Type = JointValue_<ContainerAllocator>;

  explicit JointValue_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 7>::iterator, double>(this->joint_values.begin(), this->joint_values.end(), 0.0);
    }
  }

  explicit JointValue_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : joint_values(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 7>::iterator, double>(this->joint_values.begin(), this->joint_values.end(), 0.0);
    }
  }

  // field types and members
  using _joint_values_type =
    std::array<double, 7>;
  _joint_values_type joint_values;

  // setters for named parameter idiom
  Type & set__joint_values(
    const std::array<double, 7> & _arg)
  {
    this->joint_values = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    jaka_robot_interfaces::msg::JointValue_<ContainerAllocator> *;
  using ConstRawPtr =
    const jaka_robot_interfaces::msg::JointValue_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<jaka_robot_interfaces::msg::JointValue_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<jaka_robot_interfaces::msg::JointValue_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      jaka_robot_interfaces::msg::JointValue_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<jaka_robot_interfaces::msg::JointValue_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      jaka_robot_interfaces::msg::JointValue_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<jaka_robot_interfaces::msg::JointValue_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<jaka_robot_interfaces::msg::JointValue_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<jaka_robot_interfaces::msg::JointValue_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__jaka_robot_interfaces__msg__JointValue
    std::shared_ptr<jaka_robot_interfaces::msg::JointValue_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__jaka_robot_interfaces__msg__JointValue
    std::shared_ptr<jaka_robot_interfaces::msg::JointValue_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const JointValue_ & other) const
  {
    if (this->joint_values != other.joint_values) {
      return false;
    }
    return true;
  }
  bool operator!=(const JointValue_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct JointValue_

// alias to use template instance with default allocator
using JointValue =
  jaka_robot_interfaces::msg::JointValue_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace jaka_robot_interfaces

#endif  // JAKA_ROBOT_INTERFACES__MSG__DETAIL__JOINT_VALUE__STRUCT_HPP_
