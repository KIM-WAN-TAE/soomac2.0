// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from dongsoo_interfaces:srv/DongSooExecutor.idl
// generated code does not contain a copyright notice

#ifndef DONGSOO_INTERFACES__SRV__DETAIL__DONG_SOO_EXECUTOR__FUNCTIONS_H_
#define DONGSOO_INTERFACES__SRV__DETAIL__DONG_SOO_EXECUTOR__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "dongsoo_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "dongsoo_interfaces/srv/detail/dong_soo_executor__struct.h"

/// Initialize srv/DongSooExecutor message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * dongsoo_interfaces__srv__DongSooExecutor_Request
 * )) before or use
 * dongsoo_interfaces__srv__DongSooExecutor_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_dongsoo_interfaces
bool
dongsoo_interfaces__srv__DongSooExecutor_Request__init(dongsoo_interfaces__srv__DongSooExecutor_Request * msg);

/// Finalize srv/DongSooExecutor message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dongsoo_interfaces
void
dongsoo_interfaces__srv__DongSooExecutor_Request__fini(dongsoo_interfaces__srv__DongSooExecutor_Request * msg);

/// Create srv/DongSooExecutor message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * dongsoo_interfaces__srv__DongSooExecutor_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dongsoo_interfaces
dongsoo_interfaces__srv__DongSooExecutor_Request *
dongsoo_interfaces__srv__DongSooExecutor_Request__create();

/// Destroy srv/DongSooExecutor message.
/**
 * It calls
 * dongsoo_interfaces__srv__DongSooExecutor_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dongsoo_interfaces
void
dongsoo_interfaces__srv__DongSooExecutor_Request__destroy(dongsoo_interfaces__srv__DongSooExecutor_Request * msg);

/// Check for srv/DongSooExecutor message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dongsoo_interfaces
bool
dongsoo_interfaces__srv__DongSooExecutor_Request__are_equal(const dongsoo_interfaces__srv__DongSooExecutor_Request * lhs, const dongsoo_interfaces__srv__DongSooExecutor_Request * rhs);

/// Copy a srv/DongSooExecutor message.
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
dongsoo_interfaces__srv__DongSooExecutor_Request__copy(
  const dongsoo_interfaces__srv__DongSooExecutor_Request * input,
  dongsoo_interfaces__srv__DongSooExecutor_Request * output);

/// Initialize array of srv/DongSooExecutor messages.
/**
 * It allocates the memory for the number of elements and calls
 * dongsoo_interfaces__srv__DongSooExecutor_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_dongsoo_interfaces
bool
dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence__init(dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence * array, size_t size);

/// Finalize array of srv/DongSooExecutor messages.
/**
 * It calls
 * dongsoo_interfaces__srv__DongSooExecutor_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dongsoo_interfaces
void
dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence__fini(dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence * array);

/// Create array of srv/DongSooExecutor messages.
/**
 * It allocates the memory for the array and calls
 * dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dongsoo_interfaces
dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence *
dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence__create(size_t size);

/// Destroy array of srv/DongSooExecutor messages.
/**
 * It calls
 * dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dongsoo_interfaces
void
dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence__destroy(dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence * array);

/// Check for srv/DongSooExecutor message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dongsoo_interfaces
bool
dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence__are_equal(const dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence * lhs, const dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence * rhs);

/// Copy an array of srv/DongSooExecutor messages.
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
dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence__copy(
  const dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence * input,
  dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence * output);

/// Initialize srv/DongSooExecutor message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * dongsoo_interfaces__srv__DongSooExecutor_Response
 * )) before or use
 * dongsoo_interfaces__srv__DongSooExecutor_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_dongsoo_interfaces
bool
dongsoo_interfaces__srv__DongSooExecutor_Response__init(dongsoo_interfaces__srv__DongSooExecutor_Response * msg);

/// Finalize srv/DongSooExecutor message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dongsoo_interfaces
void
dongsoo_interfaces__srv__DongSooExecutor_Response__fini(dongsoo_interfaces__srv__DongSooExecutor_Response * msg);

/// Create srv/DongSooExecutor message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * dongsoo_interfaces__srv__DongSooExecutor_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dongsoo_interfaces
dongsoo_interfaces__srv__DongSooExecutor_Response *
dongsoo_interfaces__srv__DongSooExecutor_Response__create();

/// Destroy srv/DongSooExecutor message.
/**
 * It calls
 * dongsoo_interfaces__srv__DongSooExecutor_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dongsoo_interfaces
void
dongsoo_interfaces__srv__DongSooExecutor_Response__destroy(dongsoo_interfaces__srv__DongSooExecutor_Response * msg);

/// Check for srv/DongSooExecutor message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dongsoo_interfaces
bool
dongsoo_interfaces__srv__DongSooExecutor_Response__are_equal(const dongsoo_interfaces__srv__DongSooExecutor_Response * lhs, const dongsoo_interfaces__srv__DongSooExecutor_Response * rhs);

/// Copy a srv/DongSooExecutor message.
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
dongsoo_interfaces__srv__DongSooExecutor_Response__copy(
  const dongsoo_interfaces__srv__DongSooExecutor_Response * input,
  dongsoo_interfaces__srv__DongSooExecutor_Response * output);

/// Initialize array of srv/DongSooExecutor messages.
/**
 * It allocates the memory for the number of elements and calls
 * dongsoo_interfaces__srv__DongSooExecutor_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_dongsoo_interfaces
bool
dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence__init(dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence * array, size_t size);

/// Finalize array of srv/DongSooExecutor messages.
/**
 * It calls
 * dongsoo_interfaces__srv__DongSooExecutor_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dongsoo_interfaces
void
dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence__fini(dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence * array);

/// Create array of srv/DongSooExecutor messages.
/**
 * It allocates the memory for the array and calls
 * dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_dongsoo_interfaces
dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence *
dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence__create(size_t size);

/// Destroy array of srv/DongSooExecutor messages.
/**
 * It calls
 * dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_dongsoo_interfaces
void
dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence__destroy(dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence * array);

/// Check for srv/DongSooExecutor message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_dongsoo_interfaces
bool
dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence__are_equal(const dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence * lhs, const dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence * rhs);

/// Copy an array of srv/DongSooExecutor messages.
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
dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence__copy(
  const dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence * input,
  dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // DONGSOO_INTERFACES__SRV__DETAIL__DONG_SOO_EXECUTOR__FUNCTIONS_H_
