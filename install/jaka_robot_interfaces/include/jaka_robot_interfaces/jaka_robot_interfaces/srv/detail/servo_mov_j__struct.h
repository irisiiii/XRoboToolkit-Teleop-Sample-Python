// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from jaka_robot_interfaces:srv/ServoMovJ.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__SRV__DETAIL__SERVO_MOV_J__STRUCT_H_
#define JAKA_ROBOT_INTERFACES__SRV__DETAIL__SERVO_MOV_J__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'joint_pos_left'
// Member 'joint_pos_right'
#include "jaka_robot_interfaces/msg/detail/joint_value__struct.h"
// Member 'move_mode'
#include "jaka_robot_interfaces/msg/detail/move_mode__struct.h"

/// Struct defined in srv/ServoMovJ in the package jaka_robot_interfaces.
typedef struct jaka_robot_interfaces__srv__ServoMovJ_Request
{
  jaka_robot_interfaces__msg__JointValue joint_pos_left;
  jaka_robot_interfaces__msg__JointValue joint_pos_right;
  jaka_robot_interfaces__msg__MoveMode move_mode;
  uint32_t step_num;
} jaka_robot_interfaces__srv__ServoMovJ_Request;

// Struct for a sequence of jaka_robot_interfaces__srv__ServoMovJ_Request.
typedef struct jaka_robot_interfaces__srv__ServoMovJ_Request__Sequence
{
  jaka_robot_interfaces__srv__ServoMovJ_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jaka_robot_interfaces__srv__ServoMovJ_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ServoMovJ in the package jaka_robot_interfaces.
typedef struct jaka_robot_interfaces__srv__ServoMovJ_Response
{
  bool success;
  rosidl_runtime_c__String message;
} jaka_robot_interfaces__srv__ServoMovJ_Response;

// Struct for a sequence of jaka_robot_interfaces__srv__ServoMovJ_Response.
typedef struct jaka_robot_interfaces__srv__ServoMovJ_Response__Sequence
{
  jaka_robot_interfaces__srv__ServoMovJ_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jaka_robot_interfaces__srv__ServoMovJ_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // JAKA_ROBOT_INTERFACES__SRV__DETAIL__SERVO_MOV_J__STRUCT_H_
