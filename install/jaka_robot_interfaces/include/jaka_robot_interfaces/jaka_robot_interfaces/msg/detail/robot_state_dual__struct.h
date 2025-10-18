// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from jaka_robot_interfaces:msg/RobotStateDual.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__MSG__DETAIL__ROBOT_STATE_DUAL__STRUCT_H_
#define JAKA_ROBOT_INTERFACES__MSG__DETAIL__ROBOT_STATE_DUAL__STRUCT_H_

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
// Member 'end_pose_left'
// Member 'end_pose_right'
#include "jaka_robot_interfaces/msg/detail/cartesian_pose__struct.h"

/// Struct defined in msg/RobotStateDual in the package jaka_robot_interfaces.
typedef struct jaka_robot_interfaces__msg__RobotStateDual
{
  jaka_robot_interfaces__msg__JointValue joint_pos_left;
  jaka_robot_interfaces__msg__JointValue joint_pos_right;
  jaka_robot_interfaces__msg__CartesianPose end_pose_left;
  jaka_robot_interfaces__msg__CartesianPose end_pose_right;
} jaka_robot_interfaces__msg__RobotStateDual;

// Struct for a sequence of jaka_robot_interfaces__msg__RobotStateDual.
typedef struct jaka_robot_interfaces__msg__RobotStateDual__Sequence
{
  jaka_robot_interfaces__msg__RobotStateDual * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jaka_robot_interfaces__msg__RobotStateDual__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // JAKA_ROBOT_INTERFACES__MSG__DETAIL__ROBOT_STATE_DUAL__STRUCT_H_
