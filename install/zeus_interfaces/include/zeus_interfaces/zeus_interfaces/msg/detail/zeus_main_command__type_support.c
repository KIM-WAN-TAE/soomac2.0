// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from zeus_interfaces:msg/ZeusMainCommand.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "zeus_interfaces/msg/detail/zeus_main_command__rosidl_typesupport_introspection_c.h"
#include "zeus_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "zeus_interfaces/msg/detail/zeus_main_command__functions.h"
#include "zeus_interfaces/msg/detail/zeus_main_command__struct.h"


// Include directives for member types
// Member `frame`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void zeus_interfaces__msg__ZeusMainCommand__rosidl_typesupport_introspection_c__ZeusMainCommand_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  zeus_interfaces__msg__ZeusMainCommand__init(message_memory);
}

void zeus_interfaces__msg__ZeusMainCommand__rosidl_typesupport_introspection_c__ZeusMainCommand_fini_function(void * message_memory)
{
  zeus_interfaces__msg__ZeusMainCommand__fini(message_memory);
}

size_t zeus_interfaces__msg__ZeusMainCommand__rosidl_typesupport_introspection_c__size_function__ZeusMainCommand__position(
  const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * zeus_interfaces__msg__ZeusMainCommand__rosidl_typesupport_introspection_c__get_const_function__ZeusMainCommand__position(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * zeus_interfaces__msg__ZeusMainCommand__rosidl_typesupport_introspection_c__get_function__ZeusMainCommand__position(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void zeus_interfaces__msg__ZeusMainCommand__rosidl_typesupport_introspection_c__fetch_function__ZeusMainCommand__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    zeus_interfaces__msg__ZeusMainCommand__rosidl_typesupport_introspection_c__get_const_function__ZeusMainCommand__position(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void zeus_interfaces__msg__ZeusMainCommand__rosidl_typesupport_introspection_c__assign_function__ZeusMainCommand__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    zeus_interfaces__msg__ZeusMainCommand__rosidl_typesupport_introspection_c__get_function__ZeusMainCommand__position(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember zeus_interfaces__msg__ZeusMainCommand__rosidl_typesupport_introspection_c__ZeusMainCommand_message_member_array[2] = {
  {
    "frame",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zeus_interfaces__msg__ZeusMainCommand, frame),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(zeus_interfaces__msg__ZeusMainCommand, position),  // bytes offset in struct
    NULL,  // default value
    zeus_interfaces__msg__ZeusMainCommand__rosidl_typesupport_introspection_c__size_function__ZeusMainCommand__position,  // size() function pointer
    zeus_interfaces__msg__ZeusMainCommand__rosidl_typesupport_introspection_c__get_const_function__ZeusMainCommand__position,  // get_const(index) function pointer
    zeus_interfaces__msg__ZeusMainCommand__rosidl_typesupport_introspection_c__get_function__ZeusMainCommand__position,  // get(index) function pointer
    zeus_interfaces__msg__ZeusMainCommand__rosidl_typesupport_introspection_c__fetch_function__ZeusMainCommand__position,  // fetch(index, &value) function pointer
    zeus_interfaces__msg__ZeusMainCommand__rosidl_typesupport_introspection_c__assign_function__ZeusMainCommand__position,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers zeus_interfaces__msg__ZeusMainCommand__rosidl_typesupport_introspection_c__ZeusMainCommand_message_members = {
  "zeus_interfaces__msg",  // message namespace
  "ZeusMainCommand",  // message name
  2,  // number of fields
  sizeof(zeus_interfaces__msg__ZeusMainCommand),
  zeus_interfaces__msg__ZeusMainCommand__rosidl_typesupport_introspection_c__ZeusMainCommand_message_member_array,  // message members
  zeus_interfaces__msg__ZeusMainCommand__rosidl_typesupport_introspection_c__ZeusMainCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  zeus_interfaces__msg__ZeusMainCommand__rosidl_typesupport_introspection_c__ZeusMainCommand_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t zeus_interfaces__msg__ZeusMainCommand__rosidl_typesupport_introspection_c__ZeusMainCommand_message_type_support_handle = {
  0,
  &zeus_interfaces__msg__ZeusMainCommand__rosidl_typesupport_introspection_c__ZeusMainCommand_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_zeus_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zeus_interfaces, msg, ZeusMainCommand)() {
  if (!zeus_interfaces__msg__ZeusMainCommand__rosidl_typesupport_introspection_c__ZeusMainCommand_message_type_support_handle.typesupport_identifier) {
    zeus_interfaces__msg__ZeusMainCommand__rosidl_typesupport_introspection_c__ZeusMainCommand_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &zeus_interfaces__msg__ZeusMainCommand__rosidl_typesupport_introspection_c__ZeusMainCommand_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
