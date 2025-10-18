// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from jaka_robot_interfaces:msg/ServoCartCommand.idl
// generated code does not contain a copyright notice

#ifndef JAKA_ROBOT_INTERFACES__MSG__DETAIL__SERVO_CART_COMMAND__FUNCTIONS_H_
#define JAKA_ROBOT_INTERFACES__MSG__DETAIL__SERVO_CART_COMMAND__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "jaka_robot_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "jaka_robot_interfaces/msg/detail/servo_cart_command__struct.h"

/// Initialize msg/ServoCartCommand message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * jaka_robot_interfaces__msg__ServoCartCommand
 * )) before or use
 * jaka_robot_interfaces__msg__ServoCartCommand__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_jaka_robot_interfaces
bool
jaka_robot_interfaces__msg__ServoCartCommand__init(jaka_robot_interfaces__msg__ServoCartCommand * msg);

/// Finalize msg/ServoCartCommand message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jaka_robot_interfaces
void
jaka_robot_interfaces__msg__ServoCartCommand__fini(jaka_robot_interfaces__msg__ServoCartCommand * msg);

/// Create msg/ServoCartCommand message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * jaka_robot_interfaces__msg__ServoCartCommand__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_jaka_robot_interfaces
jaka_robot_interfaces__msg__ServoCartCommand *
jaka_robot_interfaces__msg__ServoCartCommand__create();

/// Destroy msg/ServoCartCommand message.
/**
 * It calls
 * jaka_robot_interfaces__msg__ServoCartCommand__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jaka_robot_interfaces
void
jaka_robot_interfaces__msg__ServoCartCommand__destroy(jaka_robot_interfaces__msg__ServoCartCommand * msg);

/// Check for msg/ServoCartCommand message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_jaka_robot_interfaces
bool
jaka_robot_interfaces__msg__ServoCartCommand__are_equal(const jaka_robot_interfaces__msg__ServoCartCommand * lhs, const jaka_robot_interfaces__msg__ServoCartCommand * rhs);

/// Copy a msg/ServoCartCommand message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_jaka_robot_interfaces
bool
jaka_robot_interfaces__msg__ServoCartCommand__copy(
  const jaka_robot_interfaces__msg__ServoCartCommand * input,
  jaka_robot_interfaces__msg__ServoCartCommand * output);

/// Initialize array of msg/ServoCartCommand messages.
/**
 * It allocates the memory for the number of elements and calls
 * jaka_robot_interfaces__msg__ServoCartCommand__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_jaka_robot_interfaces
bool
jaka_robot_interfaces__msg__ServoCartCommand__Sequence__init(jaka_robot_interfaces__msg__ServoCartCommand__Sequence * array, size_t size);

/// Finalize array of msg/ServoCartCommand messages.
/**
 * It calls
 * jaka_robot_interfaces__msg__ServoCartCommand__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jaka_robot_interfaces
void
jaka_robot_interfaces__msg__ServoCartCommand__Sequence__fini(jaka_robot_interfaces__msg__ServoCartCommand__Sequence * array);

/// Create array of msg/ServoCartCommand messages.
/**
 * It allocates the memory for the array and calls
 * jaka_robot_interfaces__msg__ServoCartCommand__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_jaka_robot_interfaces
jaka_robot_interfaces__msg__ServoCartCommand__Sequence *
jaka_robot_interfaces__msg__ServoCartCommand__Sequence__create(size_t size);

/// Destroy array of msg/ServoCartCommand messages.
/**
 * It calls
 * jaka_robot_interfaces__msg__ServoCartCommand__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_jaka_robot_interfaces
void
jaka_robot_interfaces__msg__ServoCartCommand__Sequence__destroy(jaka_robot_interfaces__msg__ServoCartCommand__Sequence * array);

/// Check for msg/ServoCartCommand message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_jaka_robot_interfaces
bool
jaka_robot_interfaces__msg__ServoCartCommand__Sequence__are_equal(const jaka_robot_interfaces__msg__ServoCartCommand__Sequence * lhs, const jaka_robot_interfaces__msg__ServoCartCommand__Sequence * rhs);

/// Copy an array of msg/ServoCartCommand messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_jaka_robot_interfaces
bool
jaka_robot_interfaces__msg__ServoCartCommand__Sequence__copy(
  const jaka_robot_interfaces__msg__ServoCartCommand__Sequence * input,
  jaka_robot_interfaces__msg__ServoCartCommand__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // JAKA_ROBOT_INTERFACES__MSG__DETAIL__SERVO_CART_COMMAND__FUNCTIONS_H_
