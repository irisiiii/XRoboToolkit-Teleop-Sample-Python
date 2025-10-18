// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from jaka_robot_interfaces:msg/MoveMode.idl
// generated code does not contain a copyright notice
#include "jaka_robot_interfaces/msg/detail/move_mode__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
jaka_robot_interfaces__msg__MoveMode__init(jaka_robot_interfaces__msg__MoveMode * msg)
{
  if (!msg) {
    return false;
  }
  // mode
  return true;
}

void
jaka_robot_interfaces__msg__MoveMode__fini(jaka_robot_interfaces__msg__MoveMode * msg)
{
  if (!msg) {
    return;
  }
  // mode
}

bool
jaka_robot_interfaces__msg__MoveMode__are_equal(const jaka_robot_interfaces__msg__MoveMode * lhs, const jaka_robot_interfaces__msg__MoveMode * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // mode
  if (lhs->mode != rhs->mode) {
    return false;
  }
  return true;
}

bool
jaka_robot_interfaces__msg__MoveMode__copy(
  const jaka_robot_interfaces__msg__MoveMode * input,
  jaka_robot_interfaces__msg__MoveMode * output)
{
  if (!input || !output) {
    return false;
  }
  // mode
  output->mode = input->mode;
  return true;
}

jaka_robot_interfaces__msg__MoveMode *
jaka_robot_interfaces__msg__MoveMode__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jaka_robot_interfaces__msg__MoveMode * msg = (jaka_robot_interfaces__msg__MoveMode *)allocator.allocate(sizeof(jaka_robot_interfaces__msg__MoveMode), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jaka_robot_interfaces__msg__MoveMode));
  bool success = jaka_robot_interfaces__msg__MoveMode__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
jaka_robot_interfaces__msg__MoveMode__destroy(jaka_robot_interfaces__msg__MoveMode * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    jaka_robot_interfaces__msg__MoveMode__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
jaka_robot_interfaces__msg__MoveMode__Sequence__init(jaka_robot_interfaces__msg__MoveMode__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jaka_robot_interfaces__msg__MoveMode * data = NULL;

  if (size) {
    data = (jaka_robot_interfaces__msg__MoveMode *)allocator.zero_allocate(size, sizeof(jaka_robot_interfaces__msg__MoveMode), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jaka_robot_interfaces__msg__MoveMode__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jaka_robot_interfaces__msg__MoveMode__fini(&data[i - 1]);
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
jaka_robot_interfaces__msg__MoveMode__Sequence__fini(jaka_robot_interfaces__msg__MoveMode__Sequence * array)
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
      jaka_robot_interfaces__msg__MoveMode__fini(&array->data[i]);
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

jaka_robot_interfaces__msg__MoveMode__Sequence *
jaka_robot_interfaces__msg__MoveMode__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jaka_robot_interfaces__msg__MoveMode__Sequence * array = (jaka_robot_interfaces__msg__MoveMode__Sequence *)allocator.allocate(sizeof(jaka_robot_interfaces__msg__MoveMode__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = jaka_robot_interfaces__msg__MoveMode__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
jaka_robot_interfaces__msg__MoveMode__Sequence__destroy(jaka_robot_interfaces__msg__MoveMode__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    jaka_robot_interfaces__msg__MoveMode__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
jaka_robot_interfaces__msg__MoveMode__Sequence__are_equal(const jaka_robot_interfaces__msg__MoveMode__Sequence * lhs, const jaka_robot_interfaces__msg__MoveMode__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!jaka_robot_interfaces__msg__MoveMode__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
jaka_robot_interfaces__msg__MoveMode__Sequence__copy(
  const jaka_robot_interfaces__msg__MoveMode__Sequence * input,
  jaka_robot_interfaces__msg__MoveMode__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(jaka_robot_interfaces__msg__MoveMode);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    jaka_robot_interfaces__msg__MoveMode * data =
      (jaka_robot_interfaces__msg__MoveMode *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!jaka_robot_interfaces__msg__MoveMode__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          jaka_robot_interfaces__msg__MoveMode__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!jaka_robot_interfaces__msg__MoveMode__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
