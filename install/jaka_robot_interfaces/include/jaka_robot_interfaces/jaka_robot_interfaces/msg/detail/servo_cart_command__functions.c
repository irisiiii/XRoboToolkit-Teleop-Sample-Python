// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from jaka_robot_interfaces:msg/ServoCartCommand.idl
// generated code does not contain a copyright notice
#include "jaka_robot_interfaces/msg/detail/servo_cart_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `end_pose_left`
// Member `end_pose_right`
#include "jaka_robot_interfaces/msg/detail/cartesian_pose__functions.h"

bool
jaka_robot_interfaces__msg__ServoCartCommand__init(jaka_robot_interfaces__msg__ServoCartCommand * msg)
{
  if (!msg) {
    return false;
  }
  // end_pose_left
  if (!jaka_robot_interfaces__msg__CartesianPose__init(&msg->end_pose_left)) {
    jaka_robot_interfaces__msg__ServoCartCommand__fini(msg);
    return false;
  }
  // end_pose_right
  if (!jaka_robot_interfaces__msg__CartesianPose__init(&msg->end_pose_right)) {
    jaka_robot_interfaces__msg__ServoCartCommand__fini(msg);
    return false;
  }
  return true;
}

void
jaka_robot_interfaces__msg__ServoCartCommand__fini(jaka_robot_interfaces__msg__ServoCartCommand * msg)
{
  if (!msg) {
    return;
  }
  // end_pose_left
  jaka_robot_interfaces__msg__CartesianPose__fini(&msg->end_pose_left);
  // end_pose_right
  jaka_robot_interfaces__msg__CartesianPose__fini(&msg->end_pose_right);
}

bool
jaka_robot_interfaces__msg__ServoCartCommand__are_equal(const jaka_robot_interfaces__msg__ServoCartCommand * lhs, const jaka_robot_interfaces__msg__ServoCartCommand * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // end_pose_left
  if (!jaka_robot_interfaces__msg__CartesianPose__are_equal(
      &(lhs->end_pose_left), &(rhs->end_pose_left)))
  {
    return false;
  }
  // end_pose_right
  if (!jaka_robot_interfaces__msg__CartesianPose__are_equal(
      &(lhs->end_pose_right), &(rhs->end_pose_right)))
  {
    return false;
  }
  return true;
}

bool
jaka_robot_interfaces__msg__ServoCartCommand__copy(
  const jaka_robot_interfaces__msg__ServoCartCommand * input,
  jaka_robot_interfaces__msg__ServoCartCommand * output)
{
  if (!input || !output) {
    return false;
  }
  // end_pose_left
  if (!jaka_robot_interfaces__msg__CartesianPose__copy(
      &(input->end_pose_left), &(output->end_pose_left)))
  {
    return false;
  }
  // end_pose_right
  if (!jaka_robot_interfaces__msg__CartesianPose__copy(
      &(input->end_pose_right), &(output->end_pose_right)))
  {
    return false;
  }
  return true;
}

jaka_robot_interfaces__msg__ServoCartCommand *
jaka_robot_interfaces__msg__ServoCartCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jaka_robot_interfaces__msg__ServoCartCommand * msg = (jaka_robot_interfaces__msg__ServoCartCommand *)allocator.allocate(sizeof(jaka_robot_interfaces__msg__ServoCartCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jaka_robot_interfaces__msg__ServoCartCommand));
  bool success = jaka_robot_interfaces__msg__ServoCartCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
jaka_robot_interfaces__msg__ServoCartCommand__destroy(jaka_robot_interfaces__msg__ServoCartCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    jaka_robot_interfaces__msg__ServoCartCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
jaka_robot_interfaces__msg__ServoCartCommand__Sequence__init(jaka_robot_interfaces__msg__ServoCartCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jaka_robot_interfaces__msg__ServoCartCommand * data = NULL;

  if (size) {
    data = (jaka_robot_interfaces__msg__ServoCartCommand *)allocator.zero_allocate(size, sizeof(jaka_robot_interfaces__msg__ServoCartCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jaka_robot_interfaces__msg__ServoCartCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jaka_robot_interfaces__msg__ServoCartCommand__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
jaka_robot_interfaces__msg__ServoCartCommand__Sequence__fini(jaka_robot_interfaces__msg__ServoCartCommand__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      jaka_robot_interfaces__msg__ServoCartCommand__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

jaka_robot_interfaces__msg__ServoCartCommand__Sequence *
jaka_robot_interfaces__msg__ServoCartCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jaka_robot_interfaces__msg__ServoCartCommand__Sequence * array = (jaka_robot_interfaces__msg__ServoCartCommand__Sequence *)allocator.allocate(sizeof(jaka_robot_interfaces__msg__ServoCartCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = jaka_robot_interfaces__msg__ServoCartCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
jaka_robot_interfaces__msg__ServoCartCommand__Sequence__destroy(jaka_robot_interfaces__msg__ServoCartCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    jaka_robot_interfaces__msg__ServoCartCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
jaka_robot_interfaces__msg__ServoCartCommand__Sequence__are_equal(const jaka_robot_interfaces__msg__ServoCartCommand__Sequence * lhs, const jaka_robot_interfaces__msg__ServoCartCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!jaka_robot_interfaces__msg__ServoCartCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
jaka_robot_interfaces__msg__ServoCartCommand__Sequence__copy(
  const jaka_robot_interfaces__msg__ServoCartCommand__Sequence * input,
  jaka_robot_interfaces__msg__ServoCartCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(jaka_robot_interfaces__msg__ServoCartCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    jaka_robot_interfaces__msg__ServoCartCommand * data =
      (jaka_robot_interfaces__msg__ServoCartCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!jaka_robot_interfaces__msg__ServoCartCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          jaka_robot_interfaces__msg__ServoCartCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!jaka_robot_interfaces__msg__ServoCartCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
