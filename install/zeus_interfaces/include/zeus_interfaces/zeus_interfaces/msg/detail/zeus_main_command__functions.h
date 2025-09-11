// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from zeus_interfaces:msg/ZeusMainCommand.idl
// generated code does not contain a copyright notice

#ifndef ZEUS_INTERFACES__MSG__DETAIL__ZEUS_MAIN_COMMAND__FUNCTIONS_H_
#define ZEUS_INTERFACES__MSG__DETAIL__ZEUS_MAIN_COMMAND__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "zeus_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "zeus_interfaces/msg/detail/zeus_main_command__struct.h"

/// Initialize msg/ZeusMainCommand message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * zeus_interfaces__msg__ZeusMainCommand
 * )) before or use
 * zeus_interfaces__msg__ZeusMainCommand__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_zeus_interfaces
bool
zeus_interfaces__msg__ZeusMainCommand__init(zeus_interfaces__msg__ZeusMainCommand * msg);

/// Finalize msg/ZeusMainCommand message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_zeus_interfaces
void
zeus_interfaces__msg__ZeusMainCommand__fini(zeus_interfaces__msg__ZeusMainCommand * msg);

/// Create msg/ZeusMainCommand message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * zeus_interfaces__msg__ZeusMainCommand__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_zeus_interfaces
zeus_interfaces__msg__ZeusMainCommand *
zeus_interfaces__msg__ZeusMainCommand__create();

/// Destroy msg/ZeusMainCommand message.
/**
 * It calls
 * zeus_interfaces__msg__ZeusMainCommand__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_zeus_interfaces
void
zeus_interfaces__msg__ZeusMainCommand__destroy(zeus_interfaces__msg__ZeusMainCommand * msg);

/// Check for msg/ZeusMainCommand message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_zeus_interfaces
bool
zeus_interfaces__msg__ZeusMainCommand__are_equal(const zeus_interfaces__msg__ZeusMainCommand * lhs, const zeus_interfaces__msg__ZeusMainCommand * rhs);

/// Copy a msg/ZeusMainCommand message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_zeus_interfaces
bool
zeus_interfaces__msg__ZeusMainCommand__copy(
  const zeus_interfaces__msg__ZeusMainCommand * input,
  zeus_interfaces__msg__ZeusMainCommand * output);

/// Initialize array of msg/ZeusMainCommand messages.
/**
 * It allocates the memory for the number of elements and calls
 * zeus_interfaces__msg__ZeusMainCommand__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_zeus_interfaces
bool
zeus_interfaces__msg__ZeusMainCommand__Sequence__init(zeus_interfaces__msg__ZeusMainCommand__Sequence * array, size_t size);

/// Finalize array of msg/ZeusMainCommand messages.
/**
 * It calls
 * zeus_interfaces__msg__ZeusMainCommand__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_zeus_interfaces
void
zeus_interfaces__msg__ZeusMainCommand__Sequence__fini(zeus_interfaces__msg__ZeusMainCommand__Sequence * array);

/// Create array of msg/ZeusMainCommand messages.
/**
 * It allocates the memory for the array and calls
 * zeus_interfaces__msg__ZeusMainCommand__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_zeus_interfaces
zeus_interfaces__msg__ZeusMainCommand__Sequence *
zeus_interfaces__msg__ZeusMainCommand__Sequence__create(size_t size);

/// Destroy array of msg/ZeusMainCommand messages.
/**
 * It calls
 * zeus_interfaces__msg__ZeusMainCommand__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_zeus_interfaces
void
zeus_interfaces__msg__ZeusMainCommand__Sequence__destroy(zeus_interfaces__msg__ZeusMainCommand__Sequence * array);

/// Check for msg/ZeusMainCommand message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_zeus_interfaces
bool
zeus_interfaces__msg__ZeusMainCommand__Sequence__are_equal(const zeus_interfaces__msg__ZeusMainCommand__Sequence * lhs, const zeus_interfaces__msg__ZeusMainCommand__Sequence * rhs);

/// Copy an array of msg/ZeusMainCommand messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_zeus_interfaces
bool
zeus_interfaces__msg__ZeusMainCommand__Sequence__copy(
  const zeus_interfaces__msg__ZeusMainCommand__Sequence * input,
  zeus_interfaces__msg__ZeusMainCommand__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ZEUS_INTERFACES__MSG__DETAIL__ZEUS_MAIN_COMMAND__FUNCTIONS_H_
