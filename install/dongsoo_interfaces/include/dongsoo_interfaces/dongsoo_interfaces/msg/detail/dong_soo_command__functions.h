// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from dongsoo_interfaces:msg/DongSooCommand.idl
// generated code does not contain a copyright notice

#ifndef DONGSOO_INTERFACES__MSG__DETAIL__DONG_SOO_COMMAND__FUNCTIONS_H_
#define DONGSOO_INTERFACES__MSG__DETAIL__DONG_SOO_COMMAND__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "dongsoo_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "dongsoo_interfaces/msg/detail/dong_soo_command__struct.h"

/// Initialize msg/DongSooCommand message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * dongsoo_interfaces__msg__DongSooCommand
 * )) before or use
 * dongsoo_interfaces__msg__DongSooCommand__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_dongsoo_interfaces
bool
dongsoo_interfaces__msg__DongSooCommand__init(dongsoo_interfaces__msg__DongSooCommand * msg);

/// Finalize msg/DongSooCommand message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dongsoo_interfaces
void
dongsoo_interfaces__msg__DongSooCommand__fini(dongsoo_interfaces__msg__DongSooCommand * msg);

/// Create msg/DongSooCommand message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * dongsoo_interfaces__msg__DongSooCommand__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dongsoo_interfaces
dongsoo_interfaces__msg__DongSooCommand *
dongsoo_interfaces__msg__DongSooCommand__create();

/// Destroy msg/DongSooCommand message.
/**
 * It calls
 * dongsoo_interfaces__msg__DongSooCommand__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dongsoo_interfaces
void
dongsoo_interfaces__msg__DongSooCommand__destroy(dongsoo_interfaces__msg__DongSooCommand * msg);

/// Check for msg/DongSooCommand message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dongsoo_interfaces
bool
dongsoo_interfaces__msg__DongSooCommand__are_equal(const dongsoo_interfaces__msg__DongSooCommand * lhs, const dongsoo_interfaces__msg__DongSooCommand * rhs);

/// Copy a msg/DongSooCommand message.
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
ROSIDL_GENERATOR_C_PUBLIC_dongsoo_interfaces
bool
dongsoo_interfaces__msg__DongSooCommand__copy(
  const dongsoo_interfaces__msg__DongSooCommand * input,
  dongsoo_interfaces__msg__DongSooCommand * output);

/// Initialize array of msg/DongSooCommand messages.
/**
 * It allocates the memory for the number of elements and calls
 * dongsoo_interfaces__msg__DongSooCommand__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_dongsoo_interfaces
bool
dongsoo_interfaces__msg__DongSooCommand__Sequence__init(dongsoo_interfaces__msg__DongSooCommand__Sequence * array, size_t size);

/// Finalize array of msg/DongSooCommand messages.
/**
 * It calls
 * dongsoo_interfaces__msg__DongSooCommand__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dongsoo_interfaces
void
dongsoo_interfaces__msg__DongSooCommand__Sequence__fini(dongsoo_interfaces__msg__DongSooCommand__Sequence * array);

/// Create array of msg/DongSooCommand messages.
/**
 * It allocates the memory for the array and calls
 * dongsoo_interfaces__msg__DongSooCommand__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dongsoo_interfaces
dongsoo_interfaces__msg__DongSooCommand__Sequence *
dongsoo_interfaces__msg__DongSooCommand__Sequence__create(size_t size);

/// Destroy array of msg/DongSooCommand messages.
/**
 * It calls
 * dongsoo_interfaces__msg__DongSooCommand__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dongsoo_interfaces
void
dongsoo_interfaces__msg__DongSooCommand__Sequence__destroy(dongsoo_interfaces__msg__DongSooCommand__Sequence * array);

/// Check for msg/DongSooCommand message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dongsoo_interfaces
bool
dongsoo_interfaces__msg__DongSooCommand__Sequence__are_equal(const dongsoo_interfaces__msg__DongSooCommand__Sequence * lhs, const dongsoo_interfaces__msg__DongSooCommand__Sequence * rhs);

/// Copy an array of msg/DongSooCommand messages.
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
ROSIDL_GENERATOR_C_PUBLIC_dongsoo_interfaces
bool
dongsoo_interfaces__msg__DongSooCommand__Sequence__copy(
  const dongsoo_interfaces__msg__DongSooCommand__Sequence * input,
  dongsoo_interfaces__msg__DongSooCommand__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // DONGSOO_INTERFACES__MSG__DETAIL__DONG_SOO_COMMAND__FUNCTIONS_H_
