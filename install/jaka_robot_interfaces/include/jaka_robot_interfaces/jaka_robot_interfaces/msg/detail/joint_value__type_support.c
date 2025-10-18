// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from jaka_robot_interfaces:msg/JointValue.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "jaka_robot_interfaces/msg/detail/joint_value__rosidl_typesupport_introspection_c.h"
#include "jaka_robot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "jaka_robot_interfaces/msg/detail/joint_value__functions.h"
#include "jaka_robot_interfaces/msg/detail/joint_value__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void jaka_robot_interfaces__msg__JointValue__rosidl_typesupport_introspection_c__JointValue_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  jaka_robot_interfaces__msg__JointValue__init(message_memory);
}

void jaka_robot_interfaces__msg__JointValue__rosidl_typesupport_introspection_c__JointValue_fini_function(void * message_memory)
{
  jaka_robot_interfaces__msg__JointValue__fini(message_memory);
}

size_t jaka_robot_interfaces__msg__JointValue__rosidl_typesupport_introspection_c__size_function__JointValue__joint_values(
  const void * untyped_member)
{
  (void)untyped_member;
  return 7;
}

const void * jaka_robot_interfaces__msg__JointValue__rosidl_typesupport_introspection_c__get_const_function__JointValue__joint_values(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * jaka_robot_interfaces__msg__JointValue__rosidl_typesupport_introspection_c__get_function__JointValue__joint_values(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void jaka_robot_interfaces__msg__JointValue__rosidl_typesupport_introspection_c__fetch_function__JointValue__joint_values(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    jaka_robot_interfaces__msg__JointValue__rosidl_typesupport_introspection_c__get_const_function__JointValue__joint_values(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void jaka_robot_interfaces__msg__JointValue__rosidl_typesupport_introspection_c__assign_function__JointValue__joint_values(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    jaka_robot_interfaces__msg__JointValue__rosidl_typesupport_introspection_c__get_function__JointValue__joint_values(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember jaka_robot_interfaces__msg__JointValue__rosidl_typesupport_introspection_c__JointValue_message_member_array[1] = {
  {
    "joint_values",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    7,  // array size
    false,  // is upper bound
    offsetof(jaka_robot_interfaces__msg__JointValue, joint_values),  // bytes offset in struct
    NULL,  // default value
    jaka_robot_interfaces__msg__JointValue__rosidl_typesupport_introspection_c__size_function__JointValue__joint_values,  // size() function pointer
    jaka_robot_interfaces__msg__JointValue__rosidl_typesupport_introspection_c__get_const_function__JointValue__joint_values,  // get_const(index) function pointer
    jaka_robot_interfaces__msg__JointValue__rosidl_typesupport_introspection_c__get_function__JointValue__joint_values,  // get(index) function pointer
    jaka_robot_interfaces__msg__JointValue__rosidl_typesupport_introspection_c__fetch_function__JointValue__joint_values,  // fetch(index, &value) function pointer
    jaka_robot_interfaces__msg__JointValue__rosidl_typesupport_introspection_c__assign_function__JointValue__joint_values,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers jaka_robot_interfaces__msg__JointValue__rosidl_typesupport_introspection_c__JointValue_message_members = {
  "jaka_robot_interfaces__msg",  // message namespace
  "JointValue",  // message name
  1,  // number of fields
  sizeof(jaka_robot_interfaces__msg__JointValue),
  jaka_robot_interfaces__msg__JointValue__rosidl_typesupport_introspection_c__JointValue_message_member_array,  // message members
  jaka_robot_interfaces__msg__JointValue__rosidl_typesupport_introspection_c__JointValue_init_function,  // function to initialize message memory (memory has to be allocated)
  jaka_robot_interfaces__msg__JointValue__rosidl_typesupport_introspection_c__JointValue_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t jaka_robot_interfaces__msg__JointValue__rosidl_typesupport_introspection_c__JointValue_message_type_support_handle = {
  0,
  &jaka_robot_interfaces__msg__JointValue__rosidl_typesupport_introspection_c__JointValue_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_jaka_robot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, jaka_robot_interfaces, msg, JointValue)() {
  if (!jaka_robot_interfaces__msg__JointValue__rosidl_typesupport_introspection_c__JointValue_message_type_support_handle.typesupport_identifier) {
    jaka_robot_interfaces__msg__JointValue__rosidl_typesupport_introspection_c__JointValue_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &jaka_robot_interfaces__msg__JointValue__rosidl_typesupport_introspection_c__JointValue_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
