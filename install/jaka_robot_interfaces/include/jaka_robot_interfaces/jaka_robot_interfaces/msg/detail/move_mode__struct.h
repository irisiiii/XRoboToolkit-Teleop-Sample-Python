// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from jaka_robot_interfaces:msg/MoveMode.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__MSG__DETAIL__MOVE_MODE__STRUCT_H_
#define JAKA_ROBOT_INTERFACES__MSG__DETAIL__MOVE_MODE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/MoveMode in the package jaka_robot_interfaces.
/**
  * MoveMode.msg
  * 定义一个整型字段，表示运动模式
 */
typedef struct jaka_robot_interfaces__msg__MoveMode
{
  /// 0 = ABS, 1 = INCR, 2 = CONTINUE
  int32_t mode;
} jaka_robot_interfaces__msg__MoveMode;

// Struct for a sequence of jaka_robot_interfaces__msg__MoveMode.
typedef struct jaka_robot_interfaces__msg__MoveMode__Sequence
{
  jaka_robot_interfaces__msg__MoveMode * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jaka_robot_interfaces__msg__MoveMode__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // JAKA_ROBOT_INTERFACES__MSG__DETAIL__MOVE_MODE__STRUCT_H_
