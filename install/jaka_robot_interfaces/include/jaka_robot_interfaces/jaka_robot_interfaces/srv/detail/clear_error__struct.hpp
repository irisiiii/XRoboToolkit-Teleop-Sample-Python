// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from jaka_robot_interfaces:srv/ClearError.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__SRV__DETAIL__CLEAR_ERROR__STRUCT_HPP_
#define JAKA_ROBOT_INTERFACES__SRV__DETAIL__CLEAR_ERROR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__jaka_robot_interfaces__srv__ClearError_Request __attribute__((deprecated))
#else
# define DEPRECATED__jaka_robot_interfaces__srv__ClearError_Request __declspec(deprecated)
#endif

namespace jaka_robot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ClearError_Request_
{
  using Type = ClearError_Request_<ContainerAllocator>;

  explicit ClearError_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit ClearError_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    jaka_robot_interfaces::srv::ClearError_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const jaka_robot_interfaces::srv::ClearError_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<jaka_robot_interfaces::srv::ClearError_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<jaka_robot_interfaces::srv::ClearError_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      jaka_robot_interfaces::srv::ClearError_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<jaka_robot_interfaces::srv::ClearError_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      jaka_robot_interfaces::srv::ClearError_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<jaka_robot_interfaces::srv::ClearError_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<jaka_robot_interfaces::srv::ClearError_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<jaka_robot_interfaces::srv::ClearError_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__jaka_robot_interfaces__srv__ClearError_Request
    std::shared_ptr<jaka_robot_interfaces::srv::ClearError_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__jaka_robot_interfaces__srv__ClearError_Request
    std::shared_ptr<jaka_robot_interfaces::srv::ClearError_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ClearError_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const ClearError_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ClearError_Request_

// alias to use template instance with default allocator
using ClearError_Request =
  jaka_robot_interfaces::srv::ClearError_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace jaka_robot_interfaces


#ifndef _WIN32
# define DEPRECATED__jaka_robot_interfaces__srv__ClearError_Response __attribute__((deprecated))
#else
# define DEPRECATED__jaka_robot_interfaces__srv__ClearError_Response __declspec(deprecated)
#endif

namespace jaka_robot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ClearError_Response_
{
  using Type = ClearError_Response_<ContainerAllocator>;

  explicit ClearError_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit ClearError_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    jaka_robot_interfaces::srv::ClearError_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const jaka_robot_interfaces::srv::ClearError_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<jaka_robot_interfaces::srv::ClearError_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<jaka_robot_interfaces::srv::ClearError_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      jaka_robot_interfaces::srv::ClearError_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<jaka_robot_interfaces::srv::ClearError_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      jaka_robot_interfaces::srv::ClearError_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<jaka_robot_interfaces::srv::ClearError_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<jaka_robot_interfaces::srv::ClearError_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<jaka_robot_interfaces::srv::ClearError_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__jaka_robot_interfaces__srv__ClearError_Response
    std::shared_ptr<jaka_robot_interfaces::srv::ClearError_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__jaka_robot_interfaces__srv__ClearError_Response
    std::shared_ptr<jaka_robot_interfaces::srv::ClearError_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ClearError_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const ClearError_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ClearError_Response_

// alias to use template instance with default allocator
using ClearError_Response =
  jaka_robot_interfaces::srv::ClearError_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace jaka_robot_interfaces

namespace jaka_robot_interfaces
{

namespace srv
{

struct ClearError
{
  using Request = jaka_robot_interfaces::srv::ClearError_Request;
  using Response = jaka_robot_interfaces::srv::ClearError_Response;
};

}  // namespace srv

}  // namespace jaka_robot_interfaces

#endif  // JAKA_ROBOT_INTERFACES__SRV__DETAIL__CLEAR_ERROR__STRUCT_HPP_
