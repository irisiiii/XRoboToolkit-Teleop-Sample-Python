// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from jaka_robot_interfaces:srv/MultiMovJ.idl
// generated code does not contain a copyright notice
#include "jaka_robot_interfaces/srv/detail/multi_mov_j__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `left_move_mode`
// Member `right_move_mode`
#include "jaka_robot_interfaces/msg/detail/move_mode__functions.h"
// Member `joint_pos_left`
// Member `joint_pos_right`
#include "jaka_robot_interfaces/msg/detail/joint_value__functions.h"
// Member `vel`
// Member `acc`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
jaka_robot_interfaces__srv__MultiMovJ_Request__init(jaka_robot_interfaces__srv__MultiMovJ_Request * msg)
{
  if (!msg) {
    return false;
  }
  // robot_id
  // left_move_mode
  if (!jaka_robot_interfaces__msg__MoveMode__init(&msg->left_move_mode)) {
    jaka_robot_interfaces__srv__MultiMovJ_Request__fini(msg);
    return false;
  }
  // right_move_mode
  if (!jaka_robot_interfaces__msg__MoveMode__init(&msg->right_move_mode)) {
    jaka_robot_interfaces__srv__MultiMovJ_Request__fini(msg);
    return false;
  }
  // is_block
  // joint_pos_left
  if (!jaka_robot_interfaces__msg__JointValue__init(&msg->joint_pos_left)) {
    jaka_robot_interfaces__srv__MultiMovJ_Request__fini(msg);
    return false;
  }
  // joint_pos_right
  if (!jaka_robot_interfaces__msg__JointValue__init(&msg->joint_pos_right)) {
    jaka_robot_interfaces__srv__MultiMovJ_Request__fini(msg);
    return false;
  }
  // vel
  if (!rosidl_runtime_c__double__Sequence__init(&msg->vel, 0)) {
    jaka_robot_interfaces__srv__MultiMovJ_Request__fini(msg);
    return false;
  }
  // acc
  if (!rosidl_runtime_c__double__Sequence__init(&msg->acc, 0)) {
    jaka_robot_interfaces__srv__MultiMovJ_Request__fini(msg);
    return false;
  }
  return true;
}

void
jaka_robot_interfaces__srv__MultiMovJ_Request__fini(jaka_robot_interfaces__srv__MultiMovJ_Request * msg)
{
  if (!msg) {
    return;
  }
  // robot_id
  // left_move_mode
  jaka_robot_interfaces__msg__MoveMode__fini(&msg->left_move_mode);
  // right_move_mode
  jaka_robot_interfaces__msg__MoveMode__fini(&msg->right_move_mode);
  // is_block
  // joint_pos_left
  jaka_robot_interfaces__msg__JointValue__fini(&msg->joint_pos_left);
  // joint_pos_right
  jaka_robot_interfaces__msg__JointValue__fini(&msg->joint_pos_right);
  // vel
  rosidl_runtime_c__double__Sequence__fini(&msg->vel);
  // acc
  rosidl_runtime_c__double__Sequence__fini(&msg->acc);
}

bool
jaka_robot_interfaces__srv__MultiMovJ_Request__are_equal(const jaka_robot_interfaces__srv__MultiMovJ_Request * lhs, const jaka_robot_interfaces__srv__MultiMovJ_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // robot_id
  if (lhs->robot_id != rhs->robot_id) {
    return false;
  }
  // left_move_mode
  if (!jaka_robot_interfaces__msg__MoveMode__are_equal(
      &(lhs->left_move_mode), &(rhs->left_move_mode)))
  {
    return false;
  }
  // right_move_mode
  if (!jaka_robot_interfaces__msg__MoveMode__are_equal(
      &(lhs->right_move_mode), &(rhs->right_move_mode)))
  {
    return false;
  }
  // is_block
  if (lhs->is_block != rhs->is_block) {
    return false;
  }
  // joint_pos_left
  if (!jaka_robot_interfaces__msg__JointValue__are_equal(
      &(lhs->joint_pos_left), &(rhs->joint_pos_left)))
  {
    return false;
  }
  // joint_pos_right
  if (!jaka_robot_interfaces__msg__JointValue__are_equal(
      &(lhs->joint_pos_right), &(rhs->joint_pos_right)))
  {
    return false;
  }
  // vel
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->vel), &(rhs->vel)))
  {
    return false;
  }
  // acc
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->acc), &(rhs->acc)))
  {
    return false;
  }
  return true;
}

bool
jaka_robot_interfaces__srv__MultiMovJ_Request__copy(
  const jaka_robot_interfaces__srv__MultiMovJ_Request * input,
  jaka_robot_interfaces__srv__MultiMovJ_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // robot_id
  output->robot_id = input->robot_id;
  // left_move_mode
  if (!jaka_robot_interfaces__msg__MoveMode__copy(
      &(input->left_move_mode), &(output->left_move_mode)))
  {
    return false;
  }
  // right_move_mode
  if (!jaka_robot_interfaces__msg__MoveMode__copy(
      &(input->right_move_mode), &(output->right_move_mode)))
  {
    return false;
  }
  // is_block
  output->is_block = input->is_block;
  // joint_pos_left
  if (!jaka_robot_interfaces__msg__JointValue__copy(
      &(input->joint_pos_left), &(output->joint_pos_left)))
  {
    return false;
  }
  // joint_pos_right
  if (!jaka_robot_interfaces__msg__JointValue__copy(
      &(input->joint_pos_right), &(output->joint_pos_right)))
  {
    return false;
  }
  // vel
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->vel), &(output->vel)))
  {
    return false;
  }
  // acc
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->acc), &(output->acc)))
  {
    return false;
  }
  return true;
}

jaka_robot_interfaces__srv__MultiMovJ_Request *
jaka_robot_interfaces__srv__MultiMovJ_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jaka_robot_interfaces__srv__MultiMovJ_Request * msg = (jaka_robot_interfaces__srv__MultiMovJ_Request *)allocator.allocate(sizeof(jaka_robot_interfaces__srv__MultiMovJ_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jaka_robot_interfaces__srv__MultiMovJ_Request));
  bool success = jaka_robot_interfaces__srv__MultiMovJ_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
jaka_robot_interfaces__srv__MultiMovJ_Request__destroy(jaka_robot_interfaces__srv__MultiMovJ_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    jaka_robot_interfaces__srv__MultiMovJ_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
jaka_robot_interfaces__srv__MultiMovJ_Request__Sequence__init(jaka_robot_interfaces__srv__MultiMovJ_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jaka_robot_interfaces__srv__MultiMovJ_Request * data = NULL;

  if (size) {
    data = (jaka_robot_interfaces__srv__MultiMovJ_Request *)allocator.zero_allocate(size, sizeof(jaka_robot_interfaces__srv__MultiMovJ_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jaka_robot_interfaces__srv__MultiMovJ_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jaka_robot_interfaces__srv__MultiMovJ_Request__fini(&data[i - 1]);
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
jaka_robot_interfaces__srv__MultiMovJ_Request__Sequence__fini(jaka_robot_interfaces__srv__MultiMovJ_Request__Sequence * array)
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
      jaka_robot_interfaces__srv__MultiMovJ_Request__fini(&array->data[i]);
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

jaka_robot_interfaces__srv__MultiMovJ_Request__Sequence *
jaka_robot_interfaces__srv__MultiMovJ_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jaka_robot_interfaces__srv__MultiMovJ_Request__Sequence * array = (jaka_robot_interfaces__srv__MultiMovJ_Request__Sequence *)allocator.allocate(sizeof(jaka_robot_interfaces__srv__MultiMovJ_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = jaka_robot_interfaces__srv__MultiMovJ_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
jaka_robot_interfaces__srv__MultiMovJ_Request__Sequence__destroy(jaka_robot_interfaces__srv__MultiMovJ_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    jaka_robot_interfaces__srv__MultiMovJ_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
jaka_robot_interfaces__srv__MultiMovJ_Request__Sequence__are_equal(const jaka_robot_interfaces__srv__MultiMovJ_Request__Sequence * lhs, const jaka_robot_interfaces__srv__MultiMovJ_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!jaka_robot_interfaces__srv__MultiMovJ_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
jaka_robot_interfaces__srv__MultiMovJ_Request__Sequence__copy(
  const jaka_robot_interfaces__srv__MultiMovJ_Request__Sequence * input,
  jaka_robot_interfaces__srv__MultiMovJ_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(jaka_robot_interfaces__srv__MultiMovJ_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    jaka_robot_interfaces__srv__MultiMovJ_Request * data =
      (jaka_robot_interfaces__srv__MultiMovJ_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!jaka_robot_interfaces__srv__MultiMovJ_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          jaka_robot_interfaces__srv__MultiMovJ_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!jaka_robot_interfaces__srv__MultiMovJ_Request__copy(
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
jaka_robot_interfaces__srv__MultiMovJ_Response__init(jaka_robot_interfaces__srv__MultiMovJ_Response * msg)
{
  if (!msg) {
    return false;
  }
  // ret_code
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    jaka_robot_interfaces__srv__MultiMovJ_Response__fini(msg);
    return false;
  }
  // success
  return true;
}

void
jaka_robot_interfaces__srv__MultiMovJ_Response__fini(jaka_robot_interfaces__srv__MultiMovJ_Response * msg)
{
  if (!msg) {
    return;
  }
  // ret_code
  // message
  rosidl_runtime_c__String__fini(&msg->message);
  // success
}

bool
jaka_robot_interfaces__srv__MultiMovJ_Response__are_equal(const jaka_robot_interfaces__srv__MultiMovJ_Response * lhs, const jaka_robot_interfaces__srv__MultiMovJ_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // ret_code
  if (lhs->ret_code != rhs->ret_code) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
jaka_robot_interfaces__srv__MultiMovJ_Response__copy(
  const jaka_robot_interfaces__srv__MultiMovJ_Response * input,
  jaka_robot_interfaces__srv__MultiMovJ_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // ret_code
  output->ret_code = input->ret_code;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

jaka_robot_interfaces__srv__MultiMovJ_Response *
jaka_robot_interfaces__srv__MultiMovJ_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jaka_robot_interfaces__srv__MultiMovJ_Response * msg = (jaka_robot_interfaces__srv__MultiMovJ_Response *)allocator.allocate(sizeof(jaka_robot_interfaces__srv__MultiMovJ_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(jaka_robot_interfaces__srv__MultiMovJ_Response));
  bool success = jaka_robot_interfaces__srv__MultiMovJ_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
jaka_robot_interfaces__srv__MultiMovJ_Response__destroy(jaka_robot_interfaces__srv__MultiMovJ_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    jaka_robot_interfaces__srv__MultiMovJ_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
jaka_robot_interfaces__srv__MultiMovJ_Response__Sequence__init(jaka_robot_interfaces__srv__MultiMovJ_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jaka_robot_interfaces__srv__MultiMovJ_Response * data = NULL;

  if (size) {
    data = (jaka_robot_interfaces__srv__MultiMovJ_Response *)allocator.zero_allocate(size, sizeof(jaka_robot_interfaces__srv__MultiMovJ_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = jaka_robot_interfaces__srv__MultiMovJ_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        jaka_robot_interfaces__srv__MultiMovJ_Response__fini(&data[i - 1]);
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
jaka_robot_interfaces__srv__MultiMovJ_Response__Sequence__fini(jaka_robot_interfaces__srv__MultiMovJ_Response__Sequence * array)
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
      jaka_robot_interfaces__srv__MultiMovJ_Response__fini(&array->data[i]);
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

jaka_robot_interfaces__srv__MultiMovJ_Response__Sequence *
jaka_robot_interfaces__srv__MultiMovJ_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  jaka_robot_interfaces__srv__MultiMovJ_Response__Sequence * array = (jaka_robot_interfaces__srv__MultiMovJ_Response__Sequence *)allocator.allocate(sizeof(jaka_robot_interfaces__srv__MultiMovJ_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = jaka_robot_interfaces__srv__MultiMovJ_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
jaka_robot_interfaces__srv__MultiMovJ_Response__Sequence__destroy(jaka_robot_interfaces__srv__MultiMovJ_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    jaka_robot_interfaces__srv__MultiMovJ_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
jaka_robot_interfaces__srv__MultiMovJ_Response__Sequence__are_equal(const jaka_robot_interfaces__srv__MultiMovJ_Response__Sequence * lhs, const jaka_robot_interfaces__srv__MultiMovJ_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!jaka_robot_interfaces__srv__MultiMovJ_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
jaka_robot_interfaces__srv__MultiMovJ_Response__Sequence__copy(
  const jaka_robot_interfaces__srv__MultiMovJ_Response__Sequence * input,
  jaka_robot_interfaces__srv__MultiMovJ_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(jaka_robot_interfaces__srv__MultiMovJ_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    jaka_robot_interfaces__srv__MultiMovJ_Response * data =
      (jaka_robot_interfaces__srv__MultiMovJ_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!jaka_robot_interfaces__srv__MultiMovJ_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          jaka_robot_interfaces__srv__MultiMovJ_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!jaka_robot_interfaces__srv__MultiMovJ_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
