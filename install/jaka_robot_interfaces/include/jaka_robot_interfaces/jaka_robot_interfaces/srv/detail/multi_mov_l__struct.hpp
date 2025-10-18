// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from jaka_robot_interfaces:srv/MultiMovL.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__SRV__DETAIL__MULTI_MOV_L__STRUCT_HPP_
#define JAKA_ROBOT_INTERFACES__SRV__DETAIL__MULTI_MOV_L__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'left_move_mode'
// Member 'right_move_mode'
#include "jaka_robot_interfaces/msg/detail/move_mode__struct.hpp"
// Member 'end_pos_left'
// Member 'end_pos_right'
#include "jaka_robot_interfaces/msg/detail/cartesian_pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__jaka_robot_interfaces__srv__MultiMovL_Request __attribute__((deprecated))
#else
# define DEPRECATED__jaka_robot_interfaces__srv__MultiMovL_Request __declspec(deprecated)
#endif

namespace jaka_robot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MultiMovL_Request_
{
  using Type = MultiMovL_Request_<ContainerAllocator>;

  explicit MultiMovL_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : left_move_mode(_init),
    right_move_mode(_init),
    end_pos_left(_init),
    end_pos_right(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0l;
      this->is_block = false;
    }
  }

  explicit MultiMovL_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : left_move_mode(_alloc, _init),
    right_move_mode(_alloc, _init),
    end_pos_left(_alloc, _init),
    end_pos_right(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0l;
      this->is_block = false;
    }
  }

  // field types and members
  using _robot_id_type =
    int32_t;
  _robot_id_type robot_id;
  using _left_move_mode_type =
    jaka_robot_interfaces::msg::MoveMode_<ContainerAllocator>;
  _left_move_mode_type left_move_mode;
  using _right_move_mode_type =
    jaka_robot_interfaces::msg::MoveMode_<ContainerAllocator>;
  _right_move_mode_type right_move_mode;
  using _is_block_type =
    bool;
  _is_block_type is_block;
  using _end_pos_left_type =
    jaka_robot_interfaces::msg::CartesianPose_<ContainerAllocator>;
  _end_pos_left_type end_pos_left;
  using _end_pos_right_type =
    jaka_robot_interfaces::msg::CartesianPose_<ContainerAllocator>;
  _end_pos_right_type end_pos_right;
  using _vel_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _vel_type vel;
  using _acc_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _acc_type acc;

  // setters for named parameter idiom
  Type & set__robot_id(
    const int32_t & _arg)
  {
    this->robot_id = _arg;
    return *this;
  }
  Type & set__left_move_mode(
    const jaka_robot_interfaces::msg::MoveMode_<ContainerAllocator> & _arg)
  {
    this->left_move_mode = _arg;
    return *this;
  }
  Type & set__right_move_mode(
    const jaka_robot_interfaces::msg::MoveMode_<ContainerAllocator> & _arg)
  {
    this->right_move_mode = _arg;
    return *this;
  }
  Type & set__is_block(
    const bool & _arg)
  {
    this->is_block = _arg;
    return *this;
  }
  Type & set__end_pos_left(
    const jaka_robot_interfaces::msg::CartesianPose_<ContainerAllocator> & _arg)
  {
    this->end_pos_left = _arg;
    return *this;
  }
  Type & set__end_pos_right(
    const jaka_robot_interfaces::msg::CartesianPose_<ContainerAllocator> & _arg)
  {
    this->end_pos_right = _arg;
    return *this;
  }
  Type & set__vel(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->vel = _arg;
    return *this;
  }
  Type & set__acc(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->acc = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    jaka_robot_interfaces::srv::MultiMovL_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const jaka_robot_interfaces::srv::MultiMovL_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<jaka_robot_interfaces::srv::MultiMovL_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<jaka_robot_interfaces::srv::MultiMovL_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      jaka_robot_interfaces::srv::MultiMovL_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<jaka_robot_interfaces::srv::MultiMovL_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      jaka_robot_interfaces::srv::MultiMovL_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<jaka_robot_interfaces::srv::MultiMovL_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<jaka_robot_interfaces::srv::MultiMovL_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<jaka_robot_interfaces::srv::MultiMovL_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__jaka_robot_interfaces__srv__MultiMovL_Request
    std::shared_ptr<jaka_robot_interfaces::srv::MultiMovL_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__jaka_robot_interfaces__srv__MultiMovL_Request
    std::shared_ptr<jaka_robot_interfaces::srv::MultiMovL_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MultiMovL_Request_ & other) const
  {
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->left_move_mode != other.left_move_mode) {
      return false;
    }
    if (this->right_move_mode != other.right_move_mode) {
      return false;
    }
    if (this->is_block != other.is_block) {
      return false;
    }
    if (this->end_pos_left != other.end_pos_left) {
      return false;
    }
    if (this->end_pos_right != other.end_pos_right) {
      return false;
    }
    if (this->vel != other.vel) {
      return false;
    }
    if (this->acc != other.acc) {
      return false;
    }
    return true;
  }
  bool operator!=(const MultiMovL_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MultiMovL_Request_

// alias to use template instance with default allocator
using MultiMovL_Request =
  jaka_robot_interfaces::srv::MultiMovL_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace jaka_robot_interfaces


#ifndef _WIN32
# define DEPRECATED__jaka_robot_interfaces__srv__MultiMovL_Response __attribute__((deprecated))
#else
# define DEPRECATED__jaka_robot_interfaces__srv__MultiMovL_Response __declspec(deprecated)
#endif

namespace jaka_robot_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MultiMovL_Response_
{
  using Type = MultiMovL_Response_<ContainerAllocator>;

  explicit MultiMovL_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ret_code = 0l;
      this->message = "";
    }
  }

  explicit MultiMovL_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ret_code = 0l;
      this->message = "";
    }
  }

  // field types and members
  using _ret_code_type =
    int32_t;
  _ret_code_type ret_code;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__ret_code(
    const int32_t & _arg)
  {
    this->ret_code = _arg;
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
    jaka_robot_interfaces::srv::MultiMovL_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const jaka_robot_interfaces::srv::MultiMovL_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<jaka_robot_interfaces::srv::MultiMovL_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<jaka_robot_interfaces::srv::MultiMovL_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      jaka_robot_interfaces::srv::MultiMovL_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<jaka_robot_interfaces::srv::MultiMovL_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      jaka_robot_interfaces::srv::MultiMovL_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<jaka_robot_interfaces::srv::MultiMovL_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<jaka_robot_interfaces::srv::MultiMovL_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<jaka_robot_interfaces::srv::MultiMovL_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__jaka_robot_interfaces__srv__MultiMovL_Response
    std::shared_ptr<jaka_robot_interfaces::srv::MultiMovL_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__jaka_robot_interfaces__srv__MultiMovL_Response
    std::shared_ptr<jaka_robot_interfaces::srv::MultiMovL_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MultiMovL_Response_ & other) const
  {
    if (this->ret_code != other.ret_code) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const MultiMovL_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MultiMovL_Response_

// alias to use template instance with default allocator
using MultiMovL_Response =
  jaka_robot_interfaces::srv::MultiMovL_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace jaka_robot_interfaces

namespace jaka_robot_interfaces
{

namespace srv
{

struct MultiMovL
{
  using Request = jaka_robot_interfaces::srv::MultiMovL_Request;
  using Response = jaka_robot_interfaces::srv::MultiMovL_Response;
};

}  // namespace srv

}  // namespace jaka_robot_interfaces

#endif  // JAKA_ROBOT_INTERFACES__SRV__DETAIL__MULTI_MOV_L__STRUCT_HPP_
