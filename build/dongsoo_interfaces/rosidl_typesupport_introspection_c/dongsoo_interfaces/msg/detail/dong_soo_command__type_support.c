// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dongsoo_interfaces:msg/DongSooCommand.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dongsoo_interfaces/msg/detail/dong_soo_command__rosidl_typesupport_introspection_c.h"
#include "dongsoo_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dongsoo_interfaces/msg/detail/dong_soo_command__functions.h"
#include "dongsoo_interfaces/msg/detail/dong_soo_command__struct.h"


// Include directives for member types
// Member `look`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void dongsoo_interfaces__msg__DongSooCommand__rosidl_typesupport_introspection_c__DongSooCommand_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dongsoo_interfaces__msg__DongSooCommand__init(message_memory);
}

void dongsoo_interfaces__msg__DongSooCommand__rosidl_typesupport_introspection_c__DongSooCommand_fini_function(void * message_memory)
{
  dongsoo_interfaces__msg__DongSooCommand__fini(message_memory);
}

size_t dongsoo_interfaces__msg__DongSooCommand__rosidl_typesupport_introspection_c__size_function__DongSooCommand__position(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * dongsoo_interfaces__msg__DongSooCommand__rosidl_typesupport_introspection_c__get_const_function__DongSooCommand__position(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * dongsoo_interfaces__msg__DongSooCommand__rosidl_typesupport_introspection_c__get_function__DongSooCommand__position(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void dongsoo_interfaces__msg__DongSooCommand__rosidl_typesupport_introspection_c__fetch_function__DongSooCommand__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    dongsoo_interfaces__msg__DongSooCommand__rosidl_typesupport_introspection_c__get_const_function__DongSooCommand__position(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void dongsoo_interfaces__msg__DongSooCommand__rosidl_typesupport_introspection_c__assign_function__DongSooCommand__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    dongsoo_interfaces__msg__DongSooCommand__rosidl_typesupport_introspection_c__get_function__DongSooCommand__position(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember dongsoo_interfaces__msg__DongSooCommand__rosidl_typesupport_introspection_c__DongSooCommand_message_member_array[3] = {
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(dongsoo_interfaces__msg__DongSooCommand, position),  // bytes offset in struct
    NULL,  // default value
    dongsoo_interfaces__msg__DongSooCommand__rosidl_typesupport_introspection_c__size_function__DongSooCommand__position,  // size() function pointer
    dongsoo_interfaces__msg__DongSooCommand__rosidl_typesupport_introspection_c__get_const_function__DongSooCommand__position,  // get_const(index) function pointer
    dongsoo_interfaces__msg__DongSooCommand__rosidl_typesupport_introspection_c__get_function__DongSooCommand__position,  // get(index) function pointer
    dongsoo_interfaces__msg__DongSooCommand__rosidl_typesupport_introspection_c__fetch_function__DongSooCommand__position,  // fetch(index, &value) function pointer
    dongsoo_interfaces__msg__DongSooCommand__rosidl_typesupport_introspection_c__assign_function__DongSooCommand__position,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "look",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dongsoo_interfaces__msg__DongSooCommand, look),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dongsoo_interfaces__msg__DongSooCommand, time),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers dongsoo_interfaces__msg__DongSooCommand__rosidl_typesupport_introspection_c__DongSooCommand_message_members = {
  "dongsoo_interfaces__msg",  // message namespace
  "DongSooCommand",  // message name
  3,  // number of fields
  sizeof(dongsoo_interfaces__msg__DongSooCommand),
  dongsoo_interfaces__msg__DongSooCommand__rosidl_typesupport_introspection_c__DongSooCommand_message_member_array,  // message members
  dongsoo_interfaces__msg__DongSooCommand__rosidl_typesupport_introspection_c__DongSooCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  dongsoo_interfaces__msg__DongSooCommand__rosidl_typesupport_introspection_c__DongSooCommand_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t dongsoo_interfaces__msg__DongSooCommand__rosidl_typesupport_introspection_c__DongSooCommand_message_type_support_handle = {
  0,
  &dongsoo_interfaces__msg__DongSooCommand__rosidl_typesupport_introspection_c__DongSooCommand_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dongsoo_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dongsoo_interfaces, msg, DongSooCommand)() {
  if (!dongsoo_interfaces__msg__DongSooCommand__rosidl_typesupport_introspection_c__DongSooCommand_message_type_support_handle.typesupport_identifier) {
    dongsoo_interfaces__msg__DongSooCommand__rosidl_typesupport_introspection_c__DongSooCommand_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &dongsoo_interfaces__msg__DongSooCommand__rosidl_typesupport_introspection_c__DongSooCommand_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
