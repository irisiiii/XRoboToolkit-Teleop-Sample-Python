// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from jaka_robot_interfaces:srv/MultiMovL.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "jaka_robot_interfaces/srv/detail/multi_mov_l__rosidl_typesupport_introspection_c.h"
#include "jaka_robot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "jaka_robot_interfaces/srv/detail/multi_mov_l__functions.h"
#include "jaka_robot_interfaces/srv/detail/multi_mov_l__struct.h"


// Include directives for member types
// Member `left_move_mode`
// Member `right_move_mode`
#include "jaka_robot_interfaces/msg/move_mode.h"
// Member `left_move_mode`
// Member `right_move_mode`
#include "jaka_robot_interfaces/msg/detail/move_mode__rosidl_typesupport_introspection_c.h"
// Member `end_pos_left`
// Member `end_pos_right`
#include "jaka_robot_interfaces/msg/cartesian_pose.h"
// Member `end_pos_left`
// Member `end_pos_right`
#include "jaka_robot_interfaces/msg/detail/cartesian_pose__rosidl_typesupport_introspection_c.h"
// Member `vel`
// Member `acc`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__MultiMovL_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  jaka_robot_interfaces__srv__MultiMovL_Request__init(message_memory);
}

void jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__MultiMovL_Request_fini_function(void * message_memory)
{
  jaka_robot_interfaces__srv__MultiMovL_Request__fini(message_memory);
}

size_t jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__size_function__MultiMovL_Request__vel(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__get_const_function__MultiMovL_Request__vel(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__get_function__MultiMovL_Request__vel(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__fetch_function__MultiMovL_Request__vel(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__get_const_function__MultiMovL_Request__vel(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__assign_function__MultiMovL_Request__vel(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__get_function__MultiMovL_Request__vel(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__resize_function__MultiMovL_Request__vel(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__size_function__MultiMovL_Request__acc(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__get_const_function__MultiMovL_Request__acc(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__get_function__MultiMovL_Request__acc(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__fetch_function__MultiMovL_Request__acc(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__get_const_function__MultiMovL_Request__acc(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__assign_function__MultiMovL_Request__acc(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__get_function__MultiMovL_Request__acc(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__resize_function__MultiMovL_Request__acc(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__MultiMovL_Request_message_member_array[8] = {
  {
    "robot_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jaka_robot_interfaces__srv__MultiMovL_Request, robot_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "left_move_mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jaka_robot_interfaces__srv__MultiMovL_Request, left_move_mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "right_move_mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jaka_robot_interfaces__srv__MultiMovL_Request, right_move_mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_block",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jaka_robot_interfaces__srv__MultiMovL_Request, is_block),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "end_pos_left",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jaka_robot_interfaces__srv__MultiMovL_Request, end_pos_left),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "end_pos_right",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jaka_robot_interfaces__srv__MultiMovL_Request, end_pos_right),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "vel",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jaka_robot_interfaces__srv__MultiMovL_Request, vel),  // bytes offset in struct
    NULL,  // default value
    jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__size_function__MultiMovL_Request__vel,  // size() function pointer
    jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__get_const_function__MultiMovL_Request__vel,  // get_const(index) function pointer
    jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__get_function__MultiMovL_Request__vel,  // get(index) function pointer
    jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__fetch_function__MultiMovL_Request__vel,  // fetch(index, &value) function pointer
    jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__assign_function__MultiMovL_Request__vel,  // assign(index, value) function pointer
    jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__resize_function__MultiMovL_Request__vel  // resize(index) function pointer
  },
  {
    "acc",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jaka_robot_interfaces__srv__MultiMovL_Request, acc),  // bytes offset in struct
    NULL,  // default value
    jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__size_function__MultiMovL_Request__acc,  // size() function pointer
    jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__get_const_function__MultiMovL_Request__acc,  // get_const(index) function pointer
    jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__get_function__MultiMovL_Request__acc,  // get(index) function pointer
    jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__fetch_function__MultiMovL_Request__acc,  // fetch(index, &value) function pointer
    jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__assign_function__MultiMovL_Request__acc,  // assign(index, value) function pointer
    jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__resize_function__MultiMovL_Request__acc  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__MultiMovL_Request_message_members = {
  "jaka_robot_interfaces__srv",  // message namespace
  "MultiMovL_Request",  // message name
  8,  // number of fields
  sizeof(jaka_robot_interfaces__srv__MultiMovL_Request),
  jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__MultiMovL_Request_message_member_array,  // message members
  jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__MultiMovL_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__MultiMovL_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__MultiMovL_Request_message_type_support_handle = {
  0,
  &jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__MultiMovL_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_jaka_robot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, jaka_robot_interfaces, srv, MultiMovL_Request)() {
  jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__MultiMovL_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, jaka_robot_interfaces, msg, MoveMode)();
  jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__MultiMovL_Request_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, jaka_robot_interfaces, msg, MoveMode)();
  jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__MultiMovL_Request_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, jaka_robot_interfaces, msg, CartesianPose)();
  jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__MultiMovL_Request_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, jaka_robot_interfaces, msg, CartesianPose)();
  if (!jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__MultiMovL_Request_message_type_support_handle.typesupport_identifier) {
    jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__MultiMovL_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &jaka_robot_interfaces__srv__MultiMovL_Request__rosidl_typesupport_introspection_c__MultiMovL_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "jaka_robot_interfaces/srv/detail/multi_mov_l__rosidl_typesupport_introspection_c.h"
// already included above
// #include "jaka_robot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "jaka_robot_interfaces/srv/detail/multi_mov_l__functions.h"
// already included above
// #include "jaka_robot_interfaces/srv/detail/multi_mov_l__struct.h"


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void jaka_robot_interfaces__srv__MultiMovL_Response__rosidl_typesupport_introspection_c__MultiMovL_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  jaka_robot_interfaces__srv__MultiMovL_Response__init(message_memory);
}

void jaka_robot_interfaces__srv__MultiMovL_Response__rosidl_typesupport_introspection_c__MultiMovL_Response_fini_function(void * message_memory)
{
  jaka_robot_interfaces__srv__MultiMovL_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember jaka_robot_interfaces__srv__MultiMovL_Response__rosidl_typesupport_introspection_c__MultiMovL_Response_message_member_array[2] = {
  {
    "ret_code",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jaka_robot_interfaces__srv__MultiMovL_Response, ret_code),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(jaka_robot_interfaces__srv__MultiMovL_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers jaka_robot_interfaces__srv__MultiMovL_Response__rosidl_typesupport_introspection_c__MultiMovL_Response_message_members = {
  "jaka_robot_interfaces__srv",  // message namespace
  "MultiMovL_Response",  // message name
  2,  // number of fields
  sizeof(jaka_robot_interfaces__srv__MultiMovL_Response),
  jaka_robot_interfaces__srv__MultiMovL_Response__rosidl_typesupport_introspection_c__MultiMovL_Response_message_member_array,  // message members
  jaka_robot_interfaces__srv__MultiMovL_Response__rosidl_typesupport_introspection_c__MultiMovL_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  jaka_robot_interfaces__srv__MultiMovL_Response__rosidl_typesupport_introspection_c__MultiMovL_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t jaka_robot_interfaces__srv__MultiMovL_Response__rosidl_typesupport_introspection_c__MultiMovL_Response_message_type_support_handle = {
  0,
  &jaka_robot_interfaces__srv__MultiMovL_Response__rosidl_typesupport_introspection_c__MultiMovL_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_jaka_robot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, jaka_robot_interfaces, srv, MultiMovL_Response)() {
  if (!jaka_robot_interfaces__srv__MultiMovL_Response__rosidl_typesupport_introspection_c__MultiMovL_Response_message_type_support_handle.typesupport_identifier) {
    jaka_robot_interfaces__srv__MultiMovL_Response__rosidl_typesupport_introspection_c__MultiMovL_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &jaka_robot_interfaces__srv__MultiMovL_Response__rosidl_typesupport_introspection_c__MultiMovL_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "jaka_robot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "jaka_robot_interfaces/srv/detail/multi_mov_l__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers jaka_robot_interfaces__srv__detail__multi_mov_l__rosidl_typesupport_introspection_c__MultiMovL_service_members = {
  "jaka_robot_interfaces__srv",  // service namespace
  "MultiMovL",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // jaka_robot_interfaces__srv__detail__multi_mov_l__rosidl_typesupport_introspection_c__MultiMovL_Request_message_type_support_handle,
  NULL  // response message
  // jaka_robot_interfaces__srv__detail__multi_mov_l__rosidl_typesupport_introspection_c__MultiMovL_Response_message_type_support_handle
};

static rosidl_service_type_support_t jaka_robot_interfaces__srv__detail__multi_mov_l__rosidl_typesupport_introspection_c__MultiMovL_service_type_support_handle = {
  0,
  &jaka_robot_interfaces__srv__detail__multi_mov_l__rosidl_typesupport_introspection_c__MultiMovL_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, jaka_robot_interfaces, srv, MultiMovL_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, jaka_robot_interfaces, srv, MultiMovL_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_jaka_robot_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, jaka_robot_interfaces, srv, MultiMovL)() {
  if (!jaka_robot_interfaces__srv__detail__multi_mov_l__rosidl_typesupport_introspection_c__MultiMovL_service_type_support_handle.typesupport_identifier) {
    jaka_robot_interfaces__srv__detail__multi_mov_l__rosidl_typesupport_introspection_c__MultiMovL_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)jaka_robot_interfaces__srv__detail__multi_mov_l__rosidl_typesupport_introspection_c__MultiMovL_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, jaka_robot_interfaces, srv, MultiMovL_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, jaka_robot_interfaces, srv, MultiMovL_Response)()->data;
  }

  return &jaka_robot_interfaces__srv__detail__multi_mov_l__rosidl_typesupport_introspection_c__MultiMovL_service_type_support_handle;
}
