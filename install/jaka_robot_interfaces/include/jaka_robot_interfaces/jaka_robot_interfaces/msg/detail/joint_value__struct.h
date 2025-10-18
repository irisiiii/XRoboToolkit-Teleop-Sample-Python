// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from jaka_robot_interfaces:msg/JointValue.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__MSG__DETAIL__JOINT_VALUE__STRUCT_H_
#define JAKA_ROBOT_INTERFACES__MSG__DETAIL__JOINT_VALUE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/JointValue in the package jaka_robot_interfaces.
/**
  * JointValue.msg
 */
typedef struct jaka_robot_interfaces__msg__JointValue
{
  /// 7轴×2臂
  double joint_values[7];
} jaka_robot_interfaces__msg__JointValue;

// Struct for a sequence of jaka_robot_interfaces__msg__JointValue.
typedef struct jaka_robot_interfaces__msg__JointValue__Sequence
{
  jaka_robot_interfaces__msg__JointValue * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jaka_robot_interfaces__msg__JointValue__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // JAKA_ROBOT_INTERFACES__MSG__DETAIL__JOINT_VALUE__STRUCT_H_
