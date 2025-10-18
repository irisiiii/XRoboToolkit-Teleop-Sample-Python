// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from jaka_robot_interfaces:msg/JointValue.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "jaka_robot_interfaces/msg/detail/joint_value__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace jaka_robot_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void JointValue_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) jaka_robot_interfaces::msg::JointValue(_init);
}

void JointValue_fini_function(void * message_memory)
{
  auto typed_message = static_cast<jaka_robot_interfaces::msg::JointValue *>(message_memory);
  typed_message->~JointValue();
}

size_t size_function__JointValue__joint_values(const void * untyped_member)
{
  (void)untyped_member;
  return 7;
}

const void * get_const_function__JointValue__joint_values(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 7> *>(untyped_member);
  return &member[index];
}

void * get_function__JointValue__joint_values(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 7> *>(untyped_member);
  return &member[index];
}

void fetch_function__JointValue__joint_values(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__JointValue__joint_values(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__JointValue__joint_values(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__JointValue__joint_values(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember JointValue_message_member_array[1] = {
  {
    "joint_values",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    7,  // array size
    false,  // is upper bound
    offsetof(jaka_robot_interfaces::msg::JointValue, joint_values),  // bytes offset in struct
    nullptr,  // default value
    size_function__JointValue__joint_values,  // size() function pointer
    get_const_function__JointValue__joint_values,  // get_const(index) function pointer
    get_function__JointValue__joint_values,  // get(index) function pointer
    fetch_function__JointValue__joint_values,  // fetch(index, &value) function pointer
    assign_function__JointValue__joint_values,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers JointValue_message_members = {
  "jaka_robot_interfaces::msg",  // message namespace
  "JointValue",  // message name
  1,  // number of fields
  sizeof(jaka_robot_interfaces::msg::JointValue),
  JointValue_message_member_array,  // message members
  JointValue_init_function,  // function to initialize message memory (memory has to be allocated)
  JointValue_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t JointValue_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &JointValue_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace jaka_robot_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<jaka_robot_interfaces::msg::JointValue>()
{
  return &::jaka_robot_interfaces::msg::rosidl_typesupport_introspection_cpp::JointValue_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, jaka_robot_interfaces, msg, JointValue)() {
  return &::jaka_robot_interfaces::msg::rosidl_typesupport_introspection_cpp::JointValue_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
