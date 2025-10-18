// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from jaka_robot_interfaces:srv/ClearError.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__SRV__DETAIL__CLEAR_ERROR__STRUCT_H_
#define JAKA_ROBOT_INTERFACES__SRV__DETAIL__CLEAR_ERROR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/ClearError in the package jaka_robot_interfaces.
typedef struct jaka_robot_interfaces__srv__ClearError_Request
{
  uint8_t structure_needs_at_least_one_member;
} jaka_robot_interfaces__srv__ClearError_Request;

// Struct for a sequence of jaka_robot_interfaces__srv__ClearError_Request.
typedef struct jaka_robot_interfaces__srv__ClearError_Request__Sequence
{
  jaka_robot_interfaces__srv__ClearError_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jaka_robot_interfaces__srv__ClearError_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/ClearError in the package jaka_robot_interfaces.
typedef struct jaka_robot_interfaces__srv__ClearError_Response
{
  bool success;
  rosidl_runtime_c__String message;
} jaka_robot_interfaces__srv__ClearError_Response;

// Struct for a sequence of jaka_robot_interfaces__srv__ClearError_Response.
typedef struct jaka_robot_interfaces__srv__ClearError_Response__Sequence
{
  jaka_robot_interfaces__srv__ClearError_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} jaka_robot_interfaces__srv__ClearError_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // JAKA_ROBOT_INTERFACES__SRV__DETAIL__CLEAR_ERROR__STRUCT_H_
