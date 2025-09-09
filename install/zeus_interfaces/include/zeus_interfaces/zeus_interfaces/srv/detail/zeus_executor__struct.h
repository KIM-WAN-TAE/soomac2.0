// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from zeus_interfaces:srv/ZeusExecutor.idl
// generated code does not contain a copyright notice

#ifndef ZEUS_INTERFACES__SRV__DETAIL__ZEUS_EXECUTOR__STRUCT_H_
#define ZEUS_INTERFACES__SRV__DETAIL__ZEUS_EXECUTOR__STRUCT_H_

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

/// Struct defined in srv/ZeusExecutor in the package zeus_interfaces.
typedef struct zeus_interfaces__srv__ZeusExecutor_Request
{
  rosidl_runtime_c__String frame;
  float coordinate[6];
} zeus_interfaces__srv__ZeusExecutor_Request;

// Struct for a sequence of zeus_interfaces__srv__ZeusExecutor_Request.
typedef struct zeus_interfaces__srv__ZeusExecutor_Request__Sequence
{
  zeus_interfaces__srv__ZeusExecutor_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} zeus_interfaces__srv__ZeusExecutor_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/ZeusExecutor in the package zeus_interfaces.
typedef struct zeus_interfaces__srv__ZeusExecutor_Response
{
  bool success;
} zeus_interfaces__srv__ZeusExecutor_Response;

// Struct for a sequence of zeus_interfaces__srv__ZeusExecutor_Response.
typedef struct zeus_interfaces__srv__ZeusExecutor_Response__Sequence
{
  zeus_interfaces__srv__ZeusExecutor_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} zeus_interfaces__srv__ZeusExecutor_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ZEUS_INTERFACES__SRV__DETAIL__ZEUS_EXECUTOR__STRUCT_H_
