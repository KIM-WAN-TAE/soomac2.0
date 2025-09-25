// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dongsoo_interfaces:srv/DongSooExecutor.idl
// generated code does not contain a copyright notice

#ifndef DONGSOO_INTERFACES__SRV__DETAIL__DONG_SOO_EXECUTOR__STRUCT_H_
#define DONGSOO_INTERFACES__SRV__DETAIL__DONG_SOO_EXECUTOR__STRUCT_H_

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

/// Struct defined in srv/DongSooExecutor in the package dongsoo_interfaces.
typedef struct dongsoo_interfaces__srv__DongSooExecutor_Request
{
  float position[3];
  rosidl_runtime_c__String look;
  float time;
} dongsoo_interfaces__srv__DongSooExecutor_Request;

// Struct for a sequence of dongsoo_interfaces__srv__DongSooExecutor_Request.
typedef struct dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence
{
  dongsoo_interfaces__srv__DongSooExecutor_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/DongSooExecutor in the package dongsoo_interfaces.
typedef struct dongsoo_interfaces__srv__DongSooExecutor_Response
{
  bool success;
} dongsoo_interfaces__srv__DongSooExecutor_Response;

// Struct for a sequence of dongsoo_interfaces__srv__DongSooExecutor_Response.
typedef struct dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence
{
  dongsoo_interfaces__srv__DongSooExecutor_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DONGSOO_INTERFACES__SRV__DETAIL__DONG_SOO_EXECUTOR__STRUCT_H_
