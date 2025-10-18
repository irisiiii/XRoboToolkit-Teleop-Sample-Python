// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from jaka_robot_interfaces:srv/ClearError.idl
// generated code does not contain a copyright notice
#include "jaka_robot_interfaces/srv/detail/clear_error__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
jaka_robot_interfaces__srv__ClearError_Request__init(jaka_robot_interfaces__srv__ClearError_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
jaka_robot_interfaces__srv__ClearError_Request__fini(jaka_robot_interfaces__srv__ClearError_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
jaka_robot_interfaces__srv__ClearError_Request__are_equal(const jaka_robot_interfaces__srv__ClearError_Request * lhs, const jaka_robot_interfaces__srv__ClearError_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
jaka_robot_interfaces__srv__ClearError_Request__copy(
  const jaka_robot_interfaces__srv__ClearError_Request * input,
  jaka_robot_interfaces__srv__ClearError_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

jaka_robot_interfaces__srv__ClearError_Request *
jaka_robot_interfaces__srv__ClearError_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jaka_robot_interfaces__srv__ClearError_Request * msg = (jaka_robot_interfaces__srv__ClearError_Request *)allocator.allocate(sizeof(jaka_robot_interfaces__srv__ClearError_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jaka_robot_interfaces__srv__ClearError_Request));
  bool success = jaka_robot_interfaces__srv__ClearError_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
jaka_robot_interfaces__srv__ClearError_Request__destroy(jaka_robot_interfaces__srv__ClearError_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    jaka_robot_interfaces__srv__ClearError_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
jaka_robot_interfaces__srv__ClearError_Request__Sequence__init(jaka_robot_interfaces__srv__ClearError_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jaka_robot_interfaces__srv__ClearError_Request * data = NULL;

  if (size) {
    data = (jaka_robot_interfaces__srv__ClearError_Request *)allocator.zero_allocate(size, sizeof(jaka_robot_interfaces__srv__ClearError_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jaka_robot_interfaces__srv__ClearError_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jaka_robot_interfaces__srv__ClearError_Request__fini(&data[i - 1]);
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
jaka_robot_interfaces__srv__ClearError_Request__Sequence__fini(jaka_robot_interfaces__srv__ClearError_Request__Sequence * array)
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
      jaka_robot_interfaces__srv__ClearError_Request__fini(&array->data[i]);
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

jaka_robot_interfaces__srv__ClearError_Request__Sequence *
jaka_robot_interfaces__srv__ClearError_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jaka_robot_interfaces__srv__ClearError_Request__Sequence * array = (jaka_robot_interfaces__srv__ClearError_Request__Sequence *)allocator.allocate(sizeof(jaka_robot_interfaces__srv__ClearError_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = jaka_robot_interfaces__srv__ClearError_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
jaka_robot_interfaces__srv__ClearError_Request__Sequence__destroy(jaka_robot_interfaces__srv__ClearError_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    jaka_robot_interfaces__srv__ClearError_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
jaka_robot_interfaces__srv__ClearError_Request__Sequence__are_equal(const jaka_robot_interfaces__srv__ClearError_Request__Sequence * lhs, const jaka_robot_interfaces__srv__ClearError_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!jaka_robot_interfaces__srv__ClearError_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
jaka_robot_interfaces__srv__ClearError_Request__Sequence__copy(
  const jaka_robot_interfaces__srv__ClearError_Request__Sequence * input,
  jaka_robot_interfaces__srv__ClearError_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(jaka_robot_interfaces__srv__ClearError_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    jaka_robot_interfaces__srv__ClearError_Request * data =
      (jaka_robot_interfaces__srv__ClearError_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!jaka_robot_interfaces__srv__ClearError_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          jaka_robot_interfaces__srv__ClearError_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!jaka_robot_interfaces__srv__ClearError_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

bool
jaka_robot_interfaces__srv__ClearError_Response__init(jaka_robot_interfaces__srv__ClearError_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    jaka_robot_interfaces__srv__ClearError_Response__fini(msg);
    return false;
  }
  return true;
}

void
jaka_robot_interfaces__srv__ClearError_Response__fini(jaka_robot_interfaces__srv__ClearError_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
jaka_robot_interfaces__srv__ClearError_Response__are_equal(const jaka_robot_interfaces__srv__ClearError_Response * lhs, const jaka_robot_interfaces__srv__ClearError_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
jaka_robot_interfaces__srv__ClearError_Response__copy(
  const jaka_robot_interfaces__srv__ClearError_Response * input,
  jaka_robot_interfaces__srv__ClearError_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

jaka_robot_interfaces__srv__ClearError_Response *
jaka_robot_interfaces__srv__ClearError_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jaka_robot_interfaces__srv__ClearError_Response * msg = (jaka_robot_interfaces__srv__ClearError_Response *)allocator.allocate(sizeof(jaka_robot_interfaces__srv__ClearError_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jaka_robot_interfaces__srv__ClearError_Response));
  bool success = jaka_robot_interfaces__srv__ClearError_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
jaka_robot_interfaces__srv__ClearError_Response__destroy(jaka_robot_interfaces__srv__ClearError_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    jaka_robot_interfaces__srv__ClearError_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
jaka_robot_interfaces__srv__ClearError_Response__Sequence__init(jaka_robot_interfaces__srv__ClearError_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jaka_robot_interfaces__srv__ClearError_Response * data = NULL;

  if (size) {
    data = (jaka_robot_interfaces__srv__ClearError_Response *)allocator.zero_allocate(size, sizeof(jaka_robot_interfaces__srv__ClearError_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jaka_robot_interfaces__srv__ClearError_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jaka_robot_interfaces__srv__ClearError_Response__fini(&data[i - 1]);
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
jaka_robot_interfaces__srv__ClearError_Response__Sequence__fini(jaka_robot_interfaces__srv__ClearError_Response__Sequence * array)
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
      jaka_robot_interfaces__srv__ClearError_Response__fini(&array->data[i]);
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

jaka_robot_interfaces__srv__ClearError_Response__Sequence *
jaka_robot_interfaces__srv__ClearError_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jaka_robot_interfaces__srv__ClearError_Response__Sequence * array = (jaka_robot_interfaces__srv__ClearError_Response__Sequence *)allocator.allocate(sizeof(jaka_robot_interfaces__srv__ClearError_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = jaka_robot_interfaces__srv__ClearError_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
jaka_robot_interfaces__srv__ClearError_Response__Sequence__destroy(jaka_robot_interfaces__srv__ClearError_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    jaka_robot_interfaces__srv__ClearError_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
jaka_robot_interfaces__srv__ClearError_Response__Sequence__are_equal(const jaka_robot_interfaces__srv__ClearError_Response__Sequence * lhs, const jaka_robot_interfaces__srv__ClearError_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!jaka_robot_interfaces__srv__ClearError_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
jaka_robot_interfaces__srv__ClearError_Response__Sequence__copy(
  const jaka_robot_interfaces__srv__ClearError_Response__Sequence * input,
  jaka_robot_interfaces__srv__ClearError_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(jaka_robot_interfaces__srv__ClearError_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    jaka_robot_interfaces__srv__ClearError_Response * data =
      (jaka_robot_interfaces__srv__ClearError_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!jaka_robot_interfaces__srv__ClearError_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          jaka_robot_interfaces__srv__ClearError_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!jaka_robot_interfaces__srv__ClearError_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
