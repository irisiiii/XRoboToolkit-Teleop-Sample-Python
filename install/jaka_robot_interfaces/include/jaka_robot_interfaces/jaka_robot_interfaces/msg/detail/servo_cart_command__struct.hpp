// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from jaka_robot_interfaces:msg/ServoCartCommand.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__MSG__DETAIL__SERVO_CART_COMMAND__STRUCT_HPP_
#define JAKA_ROBOT_INTERFACES__MSG__DETAIL__SERVO_CART_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'end_pose_left'
// Member 'end_pose_right'
#include "jaka_robot_interfaces/msg/detail/cartesian_pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__jaka_robot_interfaces__msg__ServoCartCommand __attribute__((deprecated))
#else
# define DEPRECATED__jaka_robot_interfaces__msg__ServoCartCommand __declspec(deprecated)
#endif

namespace jaka_robot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ServoCartCommand_
{
  using Type = ServoCartCommand_<ContainerAllocator>;

  explicit ServoCartCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : end_pose_left(_init),
    end_pose_right(_init)
  {
    (void)_init;
  }

  explicit ServoCartCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : end_pose_left(_alloc, _init),
    end_pose_right(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _end_pose_left_type =
    jaka_robot_interfaces::msg::CartesianPose_<ContainerAllocator>;
  _end_pose_left_type end_pose_left;
  using _end_pose_right_type =
    jaka_robot_interfaces::msg::CartesianPose_<ContainerAllocator>;
  _end_pose_right_type end_pose_right;

  // setters for named parameter idiom
  Type & set__end_pose_left(
    const jaka_robot_interfaces::msg::CartesianPose_<ContainerAllocator> & _arg)
  {
    this->end_pose_left = _arg;
    return *this;
  }
  Type & set__end_pose_right(
    const jaka_robot_interfaces::msg::CartesianPose_<ContainerAllocator> & _arg)
  {
    this->end_pose_right = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    jaka_robot_interfaces::msg::ServoCartCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const jaka_robot_interfaces::msg::ServoCartCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<jaka_robot_interfaces::msg::ServoCartCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<jaka_robot_interfaces::msg::ServoCartCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      jaka_robot_interfaces::msg::ServoCartCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<jaka_robot_interfaces::msg::ServoCartCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      jaka_robot_interfaces::msg::ServoCartCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<jaka_robot_interfaces::msg::ServoCartCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<jaka_robot_interfaces::msg::ServoCartCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<jaka_robot_interfaces::msg::ServoCartCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__jaka_robot_interfaces__msg__ServoCartCommand
    std::shared_ptr<jaka_robot_interfaces::msg::ServoCartCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__jaka_robot_interfaces__msg__ServoCartCommand
    std::shared_ptr<jaka_robot_interfaces::msg::ServoCartCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ServoCartCommand_ & other) const
  {
    if (this->end_pose_left != other.end_pose_left) {
      return false;
    }
    if (this->end_pose_right != other.end_pose_right) {
      return false;
    }
    return true;
  }
  bool operator!=(const ServoCartCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ServoCartCommand_

// alias to use template instance with default allocator
using ServoCartCommand =
  jaka_robot_interfaces::msg::ServoCartCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace jaka_robot_interfaces

#endif  // JAKA_ROBOT_INTERFACES__MSG__DETAIL__SERVO_CART_COMMAND__STRUCT_HPP_
