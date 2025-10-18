// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from jaka_robot_interfaces:msg/ServoCartCommand.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "jaka_robot_interfaces/msg/detail/servo_cart_command__rosidl_typesupport_introspection_c.h"
#include "jaka_robot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "jaka_robot_interfaces/msg/detail/servo_cart_command__functions.h"
#include "jaka_robot_interfaces/msg/detail/servo_cart_command__struct.h"


// Include directives for member types
// Member `end_pose_left`
// Member `end_pose_right`
#include "jaka_robot_interfaces/msg/cartesian_pose.h"
// Member `end_pose_left`
// Member `end_pose_right`
#include "jaka_robot_interfaces/msg/detail/cartesian_pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void jaka_robot_interfaces__msg__ServoCartCommand__rosidl_typesupport_introspection_c__ServoCartCommand_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  jaka_robot_interfaces__msg__ServoCartCommand__init(message_memory);
}

void jaka_robot_interfaces__msg__ServoCartCommand__rosidl_typesupport_introspection_c__ServoCartCommand_fini_function(void * message_memory)
{
  jaka_robot_interfaces__msg__ServoCartCommand__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember jaka_robot_interfaces__msg__ServoCartCommand__rosidl_typesupport_introspection_c__ServoCartCommand_message_member_array[2] = {
  {
    "end_pose_left",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jaka_robot_interfaces__msg__ServoCartCommand, end_pose_left),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "end_pose_right",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jaka_robot_interfaces__msg__ServoCartCommand, end_pose_right),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers jaka_robot_interfaces__msg__ServoCartCommand__rosidl_typesupport_introspection_c__ServoCartCommand_message_members = {
  "jaka_robot_interfaces__msg",  // message namespace
  "ServoCartCommand",  // message name
  2,  // number of fields
  sizeof(jaka_robot_interfaces__msg__ServoCartCommand),
  jaka_robot_interfaces__msg__ServoCartCommand__rosidl_typesupport_introspection_c__ServoCartCommand_message_member_array,  // message members
  jaka_robot_interfaces__msg__ServoCartCommand__rosidl_typesupport_introspection_c__ServoCartCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  jaka_robot_interfaces__msg__ServoCartCommand__rosidl_typesupport_introspection_c__ServoCartCommand_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t jaka_robot_interfaces__msg__ServoCartCommand__rosidl_typesupport_introspection_c__ServoCartCommand_message_type_support_handle = {
  0,
  &jaka_robot_interfaces__msg__ServoCartCommand__rosidl_typesupport_introspection_c__ServoCartCommand_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_jaka_robot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, jaka_robot_interfaces, msg, ServoCartCommand)() {
  jaka_robot_interfaces__msg__ServoCartCommand__rosidl_typesupport_introspection_c__ServoCartCommand_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, jaka_robot_interfaces, msg, CartesianPose)();
  jaka_robot_interfaces__msg__ServoCartCommand__rosidl_typesupport_introspection_c__ServoCartCommand_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, jaka_robot_interfaces, msg, CartesianPose)();
  if (!jaka_robot_interfaces__msg__ServoCartCommand__rosidl_typesupport_introspection_c__ServoCartCommand_message_type_support_handle.typesupport_identifier) {
    jaka_robot_interfaces__msg__ServoCartCommand__rosidl_typesupport_introspection_c__ServoCartCommand_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &jaka_robot_interfaces__msg__ServoCartCommand__rosidl_typesupport_introspection_c__ServoCartCommand_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
