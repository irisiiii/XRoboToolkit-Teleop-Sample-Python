// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from jaka_robot_interfaces:srv/MultiMovJ.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__SRV__DETAIL__MULTI_MOV_J__STRUCT_H_
#define JAKA_ROBOT_INTERFACES__SRV__DETAIL__MULTI_MOV_J__STRUCT_H_

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
// Member 'joint_pos_left'
// Member 'joint_pos_right'
#include "jaka_robot_interfaces/msg/detail/joint_value__struct.h"
// Member 'vel'
// Member 'acc'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/MultiMovJ in the package jaka_robot_interfaces.
typedef struct jaka_robot_interfaces__srv__MultiMovJ_Request
{
  /// LEFT(0), RIGHT(1), DUAL(-1)
  int32_t robot_id;
  /// MoveMode move_mode
  /// 左臂运动模式
  jaka_robot_interfaces__msg__MoveMode left_move_mode;
  /// 右臂运动模式
  jaka_robot_interfaces__msg__MoveMode right_move_mode;
  /// TRUE(阻塞) 或 FALSE(非阻塞)
  bool is_block;
  jaka_robot_interfaces__msg__JointValue joint_pos_left;
  jaka_robot_interfaces__msg__JointValue joint_pos_right;
  /// 2个机器人速度指令
  rosidl_runtime_c__double__Sequence vel;
  /// 2个机器人加速度指令
  rosidl_runtime_c__double__Sequence acc;
} jaka_robot_interfaces__srv__MultiMovJ_Request;

// Struct for a sequence of jaka_robot_interfaces__srv__MultiMovJ_Request.
typedef struct jaka_robot_interfaces__srv__MultiMovJ_Request__Sequence
{
  jaka_robot_interfaces__srv__MultiMovJ_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jaka_robot_interfaces__srv__MultiMovJ_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/MultiMovJ in the package jaka_robot_interfaces.
typedef struct jaka_robot_interfaces__srv__MultiMovJ_Response
{
  /// 返回的错误码
  int32_t ret_code;
  /// 错误信息或成功提示
  rosidl_runtime_c__String message;
  bool success;
} jaka_robot_interfaces__srv__MultiMovJ_Response;

// Struct for a sequence of jaka_robot_interfaces__srv__MultiMovJ_Response.
typedef struct jaka_robot_interfaces__srv__MultiMovJ_Response__Sequence
{
  jaka_robot_interfaces__srv__MultiMovJ_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jaka_robot_interfaces__srv__MultiMovJ_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // JAKA_ROBOT_INTERFACES__SRV__DETAIL__MULTI_MOV_J__STRUCT_H_
