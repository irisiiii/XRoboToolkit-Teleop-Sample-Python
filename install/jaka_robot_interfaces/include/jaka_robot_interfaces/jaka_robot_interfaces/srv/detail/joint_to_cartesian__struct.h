// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from jaka_robot_interfaces:srv/JointToCartesian.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__SRV__DETAIL__JOINT_TO_CARTESIAN__STRUCT_H_
#define JAKA_ROBOT_INTERFACES__SRV__DETAIL__JOINT_TO_CARTESIAN__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'joint_value'
#include "jaka_robot_interfaces/msg/detail/joint_value__struct.h"

/// Struct defined in srv/JointToCartesian in the package jaka_robot_interfaces.
typedef struct jaka_robot_interfaces__srv__JointToCartesian_Request
{
  jaka_robot_interfaces__msg__JointValue joint_value;
  /// 0: 左臂, 1: 右臂
  uint8_t robot_id;
} jaka_robot_interfaces__srv__JointToCartesian_Request;

// Struct for a sequence of jaka_robot_interfaces__srv__JointToCartesian_Request.
typedef struct jaka_robot_interfaces__srv__JointToCartesian_Request__Sequence
{
  jaka_robot_interfaces__srv__JointToCartesian_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jaka_robot_interfaces__srv__JointToCartesian_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'cartesian_pose'
#include "jaka_robot_interfaces/msg/detail/cartesian_pose__struct.h"
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/JointToCartesian in the package jaka_robot_interfaces.
typedef struct jaka_robot_interfaces__srv__JointToCartesian_Response
{
  jaka_robot_interfaces__msg__CartesianPose cartesian_pose;
  bool success;
  rosidl_runtime_c__String message;
} jaka_robot_interfaces__srv__JointToCartesian_Response;

// Struct for a sequence of jaka_robot_interfaces__srv__JointToCartesian_Response.
typedef struct jaka_robot_interfaces__srv__JointToCartesian_Response__Sequence
{
  jaka_robot_interfaces__srv__JointToCartesian_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jaka_robot_interfaces__srv__JointToCartesian_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // JAKA_ROBOT_INTERFACES__SRV__DETAIL__JOINT_TO_CARTESIAN__STRUCT_H_
