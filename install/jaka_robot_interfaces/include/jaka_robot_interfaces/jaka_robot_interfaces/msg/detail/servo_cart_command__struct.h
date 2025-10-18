// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from jaka_robot_interfaces:msg/ServoCartCommand.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__MSG__DETAIL__SERVO_CART_COMMAND__STRUCT_H_
#define JAKA_ROBOT_INTERFACES__MSG__DETAIL__SERVO_CART_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'end_pose_left'
// Member 'end_pose_right'
#include "jaka_robot_interfaces/msg/detail/cartesian_pose__struct.h"

/// Struct defined in msg/ServoCartCommand in the package jaka_robot_interfaces.
typedef struct jaka_robot_interfaces__msg__ServoCartCommand
{
  jaka_robot_interfaces__msg__CartesianPose end_pose_left;
  jaka_robot_interfaces__msg__CartesianPose end_pose_right;
} jaka_robot_interfaces__msg__ServoCartCommand;

// Struct for a sequence of jaka_robot_interfaces__msg__ServoCartCommand.
typedef struct jaka_robot_interfaces__msg__ServoCartCommand__Sequence
{
  jaka_robot_interfaces__msg__ServoCartCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jaka_robot_interfaces__msg__ServoCartCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // JAKA_ROBOT_INTERFACES__MSG__DETAIL__SERVO_CART_COMMAND__STRUCT_H_
