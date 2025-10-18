// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from jaka_robot_interfaces:srv/JointToCartesian.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__SRV__DETAIL__JOINT_TO_CARTESIAN__STRUCT_HPP_
#define JAKA_ROBOT_INTERFACES__SRV__DETAIL__JOINT_TO_CARTESIAN__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'joint_value'
#include "jaka_robot_interfaces/msg/detail/joint_value__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__jaka_robot_interfaces__srv__JointToCartesian_Request __attribute__((deprecated))
#else
# define DEPRECATED__jaka_robot_interfaces__srv__JointToCartesian_Request __declspec(deprecated)
#endif

namespace jaka_robot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct JointToCartesian_Request_
{
  using Type = JointToCartesian_Request_<ContainerAllocator>;

  explicit JointToCartesian_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : joint_value(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0;
    }
  }

  explicit JointToCartesian_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : joint_value(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0;
    }
  }

  // field types and members
  using _joint_value_type =
    jaka_robot_interfaces::msg::JointValue_<ContainerAllocator>;
  _joint_value_type joint_value;
  using _robot_id_type =
    uint8_t;
  _robot_id_type robot_id;

  // setters for named parameter idiom
  Type & set__joint_value(
    const jaka_robot_interfaces::msg::JointValue_<ContainerAllocator> & _arg)
  {
    this->joint_value = _arg;
    return *this;
  }
  Type & set__robot_id(
    const uint8_t & _arg)
  {
    this->robot_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    jaka_robot_interfaces::srv::JointToCartesian_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const jaka_robot_interfaces::srv::JointToCartesian_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<jaka_robot_interfaces::srv::JointToCartesian_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<jaka_robot_interfaces::srv::JointToCartesian_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      jaka_robot_interfaces::srv::JointToCartesian_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<jaka_robot_interfaces::srv::JointToCartesian_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      jaka_robot_interfaces::srv::JointToCartesian_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<jaka_robot_interfaces::srv::JointToCartesian_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<jaka_robot_interfaces::srv::JointToCartesian_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<jaka_robot_interfaces::srv::JointToCartesian_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__jaka_robot_interfaces__srv__JointToCartesian_Request
    std::shared_ptr<jaka_robot_interfaces::srv::JointToCartesian_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__jaka_robot_interfaces__srv__JointToCartesian_Request
    std::shared_ptr<jaka_robot_interfaces::srv::JointToCartesian_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const JointToCartesian_Request_ & other) const
  {
    if (this->joint_value != other.joint_value) {
      return false;
    }
    if (this->robot_id != other.robot_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const JointToCartesian_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct JointToCartesian_Request_

// alias to use template instance with default allocator
using JointToCartesian_Request =
  jaka_robot_interfaces::srv::JointToCartesian_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace jaka_robot_interfaces


// Include directives for member types
// Member 'cartesian_pose'
#include "jaka_robot_interfaces/msg/detail/cartesian_pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__jaka_robot_interfaces__srv__JointToCartesian_Response __attribute__((deprecated))
#else
# define DEPRECATED__jaka_robot_interfaces__srv__JointToCartesian_Response __declspec(deprecated)
#endif

namespace jaka_robot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct JointToCartesian_Response_
{
  using Type = JointToCartesian_Response_<ContainerAllocator>;

  explicit JointToCartesian_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : cartesian_pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit JointToCartesian_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : cartesian_pose(_alloc, _init),
    message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _cartesian_pose_type =
    jaka_robot_interfaces::msg::CartesianPose_<ContainerAllocator>;
  _cartesian_pose_type cartesian_pose;
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__cartesian_pose(
    const jaka_robot_interfaces::msg::CartesianPose_<ContainerAllocator> & _arg)
  {
    this->cartesian_pose = _arg;
    return *this;
  }
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
    jaka_robot_interfaces::srv::JointToCartesian_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const jaka_robot_interfaces::srv::JointToCartesian_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<jaka_robot_interfaces::srv::JointToCartesian_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<jaka_robot_interfaces::srv::JointToCartesian_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      jaka_robot_interfaces::srv::JointToCartesian_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<jaka_robot_interfaces::srv::JointToCartesian_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      jaka_robot_interfaces::srv::JointToCartesian_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<jaka_robot_interfaces::srv::JointToCartesian_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<jaka_robot_interfaces::srv::JointToCartesian_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<jaka_robot_interfaces::srv::JointToCartesian_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__jaka_robot_interfaces__srv__JointToCartesian_Response
    std::shared_ptr<jaka_robot_interfaces::srv::JointToCartesian_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__jaka_robot_interfaces__srv__JointToCartesian_Response
    std::shared_ptr<jaka_robot_interfaces::srv::JointToCartesian_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const JointToCartesian_Response_ & other) const
  {
    if (this->cartesian_pose != other.cartesian_pose) {
      return false;
    }
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const JointToCartesian_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct JointToCartesian_Response_

// alias to use template instance with default allocator
using JointToCartesian_Response =
  jaka_robot_interfaces::srv::JointToCartesian_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace jaka_robot_interfaces

namespace jaka_robot_interfaces
{

namespace srv
{

struct JointToCartesian
{
  using Request = jaka_robot_interfaces::srv::JointToCartesian_Request;
  using Response = jaka_robot_interfaces::srv::JointToCartesian_Response;
};

}  // namespace srv

}  // namespace jaka_robot_interfaces

#endif  // JAKA_ROBOT_INTERFACES__SRV__DETAIL__JOINT_TO_CARTESIAN__STRUCT_HPP_
