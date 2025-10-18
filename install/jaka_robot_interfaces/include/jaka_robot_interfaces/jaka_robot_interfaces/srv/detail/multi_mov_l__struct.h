// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from jaka_robot_interfaces:srv/MultiMovL.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__SRV__DETAIL__MULTI_MOV_L__STRUCT_H_
#define JAKA_ROBOT_INTERFACES__SRV__DETAIL__MULTI_MOV_L__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'left_move_mode'
// Member 'right_move_mode'
#include "jaka_robot_interfaces/msg/detail/move_mode__struct.h"
// Member 'end_pos_left'
// Member 'end_pos_right'
#include "jaka_robot_interfaces/msg/detail/cartesian_pose__struct.h"
// Member 'vel'
// Member 'acc'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/MultiMovL in the package jaka_robot_interfaces.
typedef struct jaka_robot_interfaces__srv__MultiMovL_Request
{
  int32_t robot_id;
  jaka_robot_interfaces__msg__MoveMode left_move_mode;
  jaka_robot_interfaces__msg__MoveMode right_move_mode;
  bool is_block;
  jaka_robot_interfaces__msg__CartesianPose end_pos_left;
  jaka_robot_interfaces__msg__CartesianPose end_pos_right;
  /// 长度为2
  rosidl_runtime_c__double__Sequence vel;
  /// 长度为2
  rosidl_runtime_c__double__Sequence acc;
} jaka_robot_interfaces__srv__MultiMovL_Request;

// Struct for a sequence of jaka_robot_interfaces__srv__MultiMovL_Request.
typedef struct jaka_robot_interfaces__srv__MultiMovL_Request__Sequence
{
  jaka_robot_interfaces__srv__MultiMovL_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jaka_robot_interfaces__srv__MultiMovL_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/MultiMovL in the package jaka_robot_interfaces.
typedef struct jaka_robot_interfaces__srv__MultiMovL_Response
{
  int32_t ret_code;
  rosidl_runtime_c__String message;
} jaka_robot_interfaces__srv__MultiMovL_Response;

// Struct for a sequence of jaka_robot_interfaces__srv__MultiMovL_Response.
typedef struct jaka_robot_interfaces__srv__MultiMovL_Response__Sequence
{
  jaka_robot_interfaces__srv__MultiMovL_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jaka_robot_interfaces__srv__MultiMovL_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // JAKA_ROBOT_INTERFACES__SRV__DETAIL__MULTI_MOV_L__STRUCT_H_
