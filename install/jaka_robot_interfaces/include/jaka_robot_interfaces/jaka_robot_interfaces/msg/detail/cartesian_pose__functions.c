// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from jaka_robot_interfaces:msg/CartesianPose.idl
// generated code does not contain a copyright notice
#include "jaka_robot_interfaces/msg/detail/cartesian_pose__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
jaka_robot_interfaces__msg__CartesianPose__init(jaka_robot_interfaces__msg__CartesianPose * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  // z
  // rx
  // ry
  // rz
  return true;
}

void
jaka_robot_interfaces__msg__CartesianPose__fini(jaka_robot_interfaces__msg__CartesianPose * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
  // z
  // rx
  // ry
  // rz
}

bool
jaka_robot_interfaces__msg__CartesianPose__are_equal(const jaka_robot_interfaces__msg__CartesianPose * lhs, const jaka_robot_interfaces__msg__CartesianPose * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // z
  if (lhs->z != rhs->z) {
    return false;
  }
  // rx
  if (lhs->rx != rhs->rx) {
    return false;
  }
  // ry
  if (lhs->ry != rhs->ry) {
    return false;
  }
  // rz
  if (lhs->rz != rhs->rz) {
    return false;
  }
  return true;
}

bool
jaka_robot_interfaces__msg__CartesianPose__copy(
  const jaka_robot_interfaces__msg__CartesianPose * input,
  jaka_robot_interfaces__msg__CartesianPose * output)
{
  if (!input || !output) {
    return false;
  }
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // z
  output->z = input->z;
  // rx
  output->rx = input->rx;
  // ry
  output->ry = input->ry;
  // rz
  output->rz = input->rz;
  return true;
}

jaka_robot_interfaces__msg__CartesianPose *
jaka_robot_interfaces__msg__CartesianPose__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jaka_robot_interfaces__msg__CartesianPose * msg = (jaka_robot_interfaces__msg__CartesianPose *)allocator.allocate(sizeof(jaka_robot_interfaces__msg__CartesianPose), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jaka_robot_interfaces__msg__CartesianPose));
  bool success = jaka_robot_interfaces__msg__CartesianPose__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
jaka_robot_interfaces__msg__CartesianPose__destroy(jaka_robot_interfaces__msg__CartesianPose * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    jaka_robot_interfaces__msg__CartesianPose__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
jaka_robot_interfaces__msg__CartesianPose__Sequence__init(jaka_robot_interfaces__msg__CartesianPose__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jaka_robot_interfaces__msg__CartesianPose * data = NULL;

  if (size) {
    data = (jaka_robot_interfaces__msg__CartesianPose *)allocator.zero_allocate(size, sizeof(jaka_robot_interfaces__msg__CartesianPose), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jaka_robot_interfaces__msg__CartesianPose__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jaka_robot_interfaces__msg__CartesianPose__fini(&data[i - 1]);
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
jaka_robot_interfaces__msg__CartesianPose__Sequence__fini(jaka_robot_interfaces__msg__CartesianPose__Sequence * array)
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
      jaka_robot_interfaces__msg__CartesianPose__fini(&array->data[i]);
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

jaka_robot_interfaces__msg__CartesianPose__Sequence *
jaka_robot_interfaces__msg__CartesianPose__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jaka_robot_interfaces__msg__CartesianPose__Sequence * array = (jaka_robot_interfaces__msg__CartesianPose__Sequence *)allocator.allocate(sizeof(jaka_robot_interfaces__msg__CartesianPose__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = jaka_robot_interfaces__msg__CartesianPose__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
jaka_robot_interfaces__msg__CartesianPose__Sequence__destroy(jaka_robot_interfaces__msg__CartesianPose__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    jaka_robot_interfaces__msg__CartesianPose__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
jaka_robot_interfaces__msg__CartesianPose__Sequence__are_equal(const jaka_robot_interfaces__msg__CartesianPose__Sequence * lhs, const jaka_robot_interfaces__msg__CartesianPose__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!jaka_robot_interfaces__msg__CartesianPose__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
jaka_robot_interfaces__msg__CartesianPose__Sequence__copy(
  const jaka_robot_interfaces__msg__CartesianPose__Sequence * input,
  jaka_robot_interfaces__msg__CartesianPose__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(jaka_robot_interfaces__msg__CartesianPose);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    jaka_robot_interfaces__msg__CartesianPose * data =
      (jaka_robot_interfaces__msg__CartesianPose *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!jaka_robot_interfaces__msg__CartesianPose__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          jaka_robot_interfaces__msg__CartesianPose__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!jaka_robot_interfaces__msg__CartesianPose__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
