// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from jaka_robot_interfaces:msg/CartesianPose.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__MSG__DETAIL__CARTESIAN_POSE__STRUCT_H_
#define JAKA_ROBOT_INTERFACES__MSG__DETAIL__CARTESIAN_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/CartesianPose in the package jaka_robot_interfaces.
/**
  * CartesianPose.msg
 */
typedef struct jaka_robot_interfaces__msg__CartesianPose
{
  double x;
  double y;
  double z;
  double rx;
  double ry;
  double rz;
} jaka_robot_interfaces__msg__CartesianPose;

// Struct for a sequence of jaka_robot_interfaces__msg__CartesianPose.
typedef struct jaka_robot_interfaces__msg__CartesianPose__Sequence
{
  jaka_robot_interfaces__msg__CartesianPose * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jaka_robot_interfaces__msg__CartesianPose__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // JAKA_ROBOT_INTERFACES__MSG__DETAIL__CARTESIAN_POSE__STRUCT_H_
