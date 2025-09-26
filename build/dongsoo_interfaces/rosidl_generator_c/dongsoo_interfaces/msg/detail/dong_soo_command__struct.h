// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dongsoo_interfaces:msg/DongSooCommand.idl
// generated code does not contain a copyright notice

#ifndef DONGSOO_INTERFACES__MSG__DETAIL__DONG_SOO_COMMAND__STRUCT_H_
#define DONGSOO_INTERFACES__MSG__DETAIL__DONG_SOO_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'look'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/DongSooCommand in the package dongsoo_interfaces.
/**
  * dongsoo_interfaces/msg/DongSooCommand.msg
 */
typedef struct dongsoo_interfaces__msg__DongSooCommand
{
  float position[3];
  rosidl_runtime_c__String look;
  float time;
  float wrist;
} dongsoo_interfaces__msg__DongSooCommand;

// Struct for a sequence of dongsoo_interfaces__msg__DongSooCommand.
typedef struct dongsoo_interfaces__msg__DongSooCommand__Sequence
{
  dongsoo_interfaces__msg__DongSooCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dongsoo_interfaces__msg__DongSooCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DONGSOO_INTERFACES__MSG__DETAIL__DONG_SOO_COMMAND__STRUCT_H_
