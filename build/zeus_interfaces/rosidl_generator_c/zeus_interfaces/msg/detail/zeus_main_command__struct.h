// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from zeus_interfaces:msg/ZeusMainCommand.idl
// generated code does not contain a copyright notice

#ifndef ZEUS_INTERFACES__MSG__DETAIL__ZEUS_MAIN_COMMAND__STRUCT_H_
#define ZEUS_INTERFACES__MSG__DETAIL__ZEUS_MAIN_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'frame'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/ZeusMainCommand in the package zeus_interfaces.
/**
  * zeus_interfaces/msg/ZeusMainCommand.msg
 */
typedef struct zeus_interfaces__msg__ZeusMainCommand
{
  rosidl_runtime_c__String frame;
  float position[6];
  float speed;
} zeus_interfaces__msg__ZeusMainCommand;

// Struct for a sequence of zeus_interfaces__msg__ZeusMainCommand.
typedef struct zeus_interfaces__msg__ZeusMainCommand__Sequence
{
  zeus_interfaces__msg__ZeusMainCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} zeus_interfaces__msg__ZeusMainCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ZEUS_INTERFACES__MSG__DETAIL__ZEUS_MAIN_COMMAND__STRUCT_H_
