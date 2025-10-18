// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from jaka_robot_interfaces:srv/ServoMovJ.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__SRV__DETAIL__SERVO_MOV_J__STRUCT_HPP_
#define JAKA_ROBOT_INTERFACES__SRV__DETAIL__SERVO_MOV_J__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'joint_pos_left'
// Member 'joint_pos_right'
#include "jaka_robot_interfaces/msg/detail/joint_value__struct.hpp"
// Member 'move_mode'
#include "jaka_robot_interfaces/msg/detail/move_mode__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__jaka_robot_interfaces__srv__ServoMovJ_Request __attribute__((deprecated))
#else
# define DEPRECATED__jaka_robot_interfaces__srv__ServoMovJ_Request __declspec(deprecated)
#endif

namespace jaka_robot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ServoMovJ_Request_
{
  using Type = ServoMovJ_Request_<ContainerAllocator>;

  explicit ServoMovJ_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : joint_pos_left(_init),
    joint_pos_right(_init),
    move_mode(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->step_num = 0ul;
    }
  }

  explicit ServoMovJ_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : joint_pos_left(_alloc, _init),
    joint_pos_right(_alloc, _init),
    move_mode(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->step_num = 0ul;
    }
  }

  // field types and members
  using _joint_pos_left_type =
    jaka_robot_interfaces::msg::JointValue_<ContainerAllocator>;
  _joint_pos_left_type joint_pos_left;
  using _joint_pos_right_type =
    jaka_robot_interfaces::msg::JointValue_<ContainerAllocator>;
  _joint_pos_right_type joint_pos_right;
  using _move_mode_type =
    jaka_robot_interfaces::msg::MoveMode_<ContainerAllocator>;
  _move_mode_type move_mode;
  using _step_num_type =
    uint32_t;
  _step_num_type step_num;

  // setters for named parameter idiom
  Type & set__joint_pos_left(
    const jaka_robot_interfaces::msg::JointValue_<ContainerAllocator> & _arg)
  {
    this->joint_pos_left = _arg;
    return *this;
  }
  Type & set__joint_pos_right(
    const jaka_robot_interfaces::msg::JointValue_<ContainerAllocator> & _arg)
  {
    this->joint_pos_right = _arg;
    return *this;
  }
  Type & set__move_mode(
    const jaka_robot_interfaces::msg::MoveMode_<ContainerAllocator> & _arg)
  {
    this->move_mode = _arg;
    return *this;
  }
  Type & set__step_num(
    const uint32_t & _arg)
  {
    this->step_num = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    jaka_robot_interfaces::srv::ServoMovJ_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const jaka_robot_interfaces::srv::ServoMovJ_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<jaka_robot_interfaces::srv::ServoMovJ_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<jaka_robot_interfaces::srv::ServoMovJ_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      jaka_robot_interfaces::srv::ServoMovJ_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<jaka_robot_interfaces::srv::ServoMovJ_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      jaka_robot_interfaces::srv::ServoMovJ_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<jaka_robot_interfaces::srv::ServoMovJ_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<jaka_robot_interfaces::srv::ServoMovJ_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<jaka_robot_interfaces::srv::ServoMovJ_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__jaka_robot_interfaces__srv__ServoMovJ_Request
    std::shared_ptr<jaka_robot_interfaces::srv::ServoMovJ_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__jaka_robot_interfaces__srv__ServoMovJ_Request
    std::shared_ptr<jaka_robot_interfaces::srv::ServoMovJ_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ServoMovJ_Request_ & other) const
  {
    if (this->joint_pos_left != other.joint_pos_left) {
      return false;
    }
    if (this->joint_pos_right != other.joint_pos_right) {
      return false;
    }
    if (this->move_mode != other.move_mode) {
      return false;
    }
    if (this->step_num != other.step_num) {
      return false;
    }
    return true;
  }
  bool operator!=(const ServoMovJ_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ServoMovJ_Request_

// alias to use template instance with default allocator
using ServoMovJ_Request =
  jaka_robot_interfaces::srv::ServoMovJ_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace jaka_robot_interfaces


#ifndef _WIN32
# define DEPRECATED__jaka_robot_interfaces__srv__ServoMovJ_Response __attribute__((deprecated))
#else
# define DEPRECATED__jaka_robot_interfaces__srv__ServoMovJ_Response __declspec(deprecated)
#endif

namespace jaka_robot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ServoMovJ_Response_
{
  using Type = ServoMovJ_Response_<ContainerAllocator>;

  explicit ServoMovJ_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit ServoMovJ_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    jaka_robot_interfaces::srv::ServoMovJ_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const jaka_robot_interfaces::srv::ServoMovJ_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<jaka_robot_interfaces::srv::ServoMovJ_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<jaka_robot_interfaces::srv::ServoMovJ_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      jaka_robot_interfaces::srv::ServoMovJ_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<jaka_robot_interfaces::srv::ServoMovJ_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      jaka_robot_interfaces::srv::ServoMovJ_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<jaka_robot_interfaces::srv::ServoMovJ_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<jaka_robot_interfaces::srv::ServoMovJ_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<jaka_robot_interfaces::srv::ServoMovJ_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__jaka_robot_interfaces__srv__ServoMovJ_Response
    std::shared_ptr<jaka_robot_interfaces::srv::ServoMovJ_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__jaka_robot_interfaces__srv__ServoMovJ_Response
    std::shared_ptr<jaka_robot_interfaces::srv::ServoMovJ_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ServoMovJ_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const ServoMovJ_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ServoMovJ_Response_

// alias to use template instance with default allocator
using ServoMovJ_Response =
  jaka_robot_interfaces::srv::ServoMovJ_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace jaka_robot_interfaces

namespace jaka_robot_interfaces
{

namespace srv
{

struct ServoMovJ
{
  using Request = jaka_robot_interfaces::srv::ServoMovJ_Request;
  using Response = jaka_robot_interfaces::srv::ServoMovJ_Response;
};

}  // namespace srv

}  // namespace jaka_robot_interfaces

#endif  // JAKA_ROBOT_INTERFACES__SRV__DETAIL__SERVO_MOV_J__STRUCT_HPP_
