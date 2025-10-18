// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from jaka_robot_interfaces:msg/MoveMode.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__MSG__DETAIL__MOVE_MODE__STRUCT_HPP_
#define JAKA_ROBOT_INTERFACES__MSG__DETAIL__MOVE_MODE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__jaka_robot_interfaces__msg__MoveMode __attribute__((deprecated))
#else
# define DEPRECATED__jaka_robot_interfaces__msg__MoveMode __declspec(deprecated)
#endif

namespace jaka_robot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MoveMode_
{
  using Type = MoveMode_<ContainerAllocator>;

  explicit MoveMode_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0l;
    }
  }

  explicit MoveMode_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0l;
    }
  }

  // field types and members
  using _mode_type =
    int32_t;
  _mode_type mode;

  // setters for named parameter idiom
  Type & set__mode(
    const int32_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    jaka_robot_interfaces::msg::MoveMode_<ContainerAllocator> *;
  using ConstRawPtr =
    const jaka_robot_interfaces::msg::MoveMode_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<jaka_robot_interfaces::msg::MoveMode_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<jaka_robot_interfaces::msg::MoveMode_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      jaka_robot_interfaces::msg::MoveMode_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<jaka_robot_interfaces::msg::MoveMode_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      jaka_robot_interfaces::msg::MoveMode_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<jaka_robot_interfaces::msg::MoveMode_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<jaka_robot_interfaces::msg::MoveMode_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<jaka_robot_interfaces::msg::MoveMode_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__jaka_robot_interfaces__msg__MoveMode
    std::shared_ptr<jaka_robot_interfaces::msg::MoveMode_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__jaka_robot_interfaces__msg__MoveMode
    std::shared_ptr<jaka_robot_interfaces::msg::MoveMode_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveMode_ & other) const
  {
    if (this->mode != other.mode) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveMode_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveMode_

// alias to use template instance with default allocator
using MoveMode =
  jaka_robot_interfaces::msg::MoveMode_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace jaka_robot_interfaces

#endif  // JAKA_ROBOT_INTERFACES__MSG__DETAIL__MOVE_MODE__STRUCT_HPP_
