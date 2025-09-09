// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from zeus_interfaces:srv/ZeusExecutor.idl
// generated code does not contain a copyright notice

#ifndef ZEUS_INTERFACES__SRV__DETAIL__ZEUS_EXECUTOR__FUNCTIONS_H_
#define ZEUS_INTERFACES__SRV__DETAIL__ZEUS_EXECUTOR__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "zeus_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "zeus_interfaces/srv/detail/zeus_executor__struct.h"

/// Initialize srv/ZeusExecutor message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * zeus_interfaces__srv__ZeusExecutor_Request
 * )) before or use
 * zeus_interfaces__srv__ZeusExecutor_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_zeus_interfaces
bool
zeus_interfaces__srv__ZeusExecutor_Request__init(zeus_interfaces__srv__ZeusExecutor_Request * msg);

/// Finalize srv/ZeusExecutor message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_zeus_interfaces
void
zeus_interfaces__srv__ZeusExecutor_Request__fini(zeus_interfaces__srv__ZeusExecutor_Request * msg);

/// Create srv/ZeusExecutor message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * zeus_interfaces__srv__ZeusExecutor_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_zeus_interfaces
zeus_interfaces__srv__ZeusExecutor_Request *
zeus_interfaces__srv__ZeusExecutor_Request__create();

/// Destroy srv/ZeusExecutor message.
/**
 * It calls
 * zeus_interfaces__srv__ZeusExecutor_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_zeus_interfaces
void
zeus_interfaces__srv__ZeusExecutor_Request__destroy(zeus_interfaces__srv__ZeusExecutor_Request * msg);

/// Check for srv/ZeusExecutor message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_zeus_interfaces
bool
zeus_interfaces__srv__ZeusExecutor_Request__are_equal(const zeus_interfaces__srv__ZeusExecutor_Request * lhs, const zeus_interfaces__srv__ZeusExecutor_Request * rhs);

/// Copy a srv/ZeusExecutor message.
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
zeus_interfaces__srv__ZeusExecutor_Request__copy(
  const zeus_interfaces__srv__ZeusExecutor_Request * input,
  zeus_interfaces__srv__ZeusExecutor_Request * output);

/// Initialize array of srv/ZeusExecutor messages.
/**
 * It allocates the memory for the number of elements and calls
 * zeus_interfaces__srv__ZeusExecutor_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_zeus_interfaces
bool
zeus_interfaces__srv__ZeusExecutor_Request__Sequence__init(zeus_interfaces__srv__ZeusExecutor_Request__Sequence * array, size_t size);

/// Finalize array of srv/ZeusExecutor messages.
/**
 * It calls
 * zeus_interfaces__srv__ZeusExecutor_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_zeus_interfaces
void
zeus_interfaces__srv__ZeusExecutor_Request__Sequence__fini(zeus_interfaces__srv__ZeusExecutor_Request__Sequence * array);

/// Create array of srv/ZeusExecutor messages.
/**
 * It allocates the memory for the array and calls
 * zeus_interfaces__srv__ZeusExecutor_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_zeus_interfaces
zeus_interfaces__srv__ZeusExecutor_Request__Sequence *
zeus_interfaces__srv__ZeusExecutor_Request__Sequence__create(size_t size);

/// Destroy array of srv/ZeusExecutor messages.
/**
 * It calls
 * zeus_interfaces__srv__ZeusExecutor_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_zeus_interfaces
void
zeus_interfaces__srv__ZeusExecutor_Request__Sequence__destroy(zeus_interfaces__srv__ZeusExecutor_Request__Sequence * array);

/// Check for srv/ZeusExecutor message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_zeus_interfaces
bool
zeus_interfaces__srv__ZeusExecutor_Request__Sequence__are_equal(const zeus_interfaces__srv__ZeusExecutor_Request__Sequence * lhs, const zeus_interfaces__srv__ZeusExecutor_Request__Sequence * rhs);

/// Copy an array of srv/ZeusExecutor messages.
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
zeus_interfaces__srv__ZeusExecutor_Request__Sequence__copy(
  const zeus_interfaces__srv__ZeusExecutor_Request__Sequence * input,
  zeus_interfaces__srv__ZeusExecutor_Request__Sequence * output);

/// Initialize srv/ZeusExecutor message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * zeus_interfaces__srv__ZeusExecutor_Response
 * )) before or use
 * zeus_interfaces__srv__ZeusExecutor_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_zeus_interfaces
bool
zeus_interfaces__srv__ZeusExecutor_Response__init(zeus_interfaces__srv__ZeusExecutor_Response * msg);

/// Finalize srv/ZeusExecutor message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_zeus_interfaces
void
zeus_interfaces__srv__ZeusExecutor_Response__fini(zeus_interfaces__srv__ZeusExecutor_Response * msg);

/// Create srv/ZeusExecutor message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * zeus_interfaces__srv__ZeusExecutor_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_zeus_interfaces
zeus_interfaces__srv__ZeusExecutor_Response *
zeus_interfaces__srv__ZeusExecutor_Response__create();

/// Destroy srv/ZeusExecutor message.
/**
 * It calls
 * zeus_interfaces__srv__ZeusExecutor_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_zeus_interfaces
void
zeus_interfaces__srv__ZeusExecutor_Response__destroy(zeus_interfaces__srv__ZeusExecutor_Response * msg);

/// Check for srv/ZeusExecutor message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_zeus_interfaces
bool
zeus_interfaces__srv__ZeusExecutor_Response__are_equal(const zeus_interfaces__srv__ZeusExecutor_Response * lhs, const zeus_interfaces__srv__ZeusExecutor_Response * rhs);

/// Copy a srv/ZeusExecutor message.
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
zeus_interfaces__srv__ZeusExecutor_Response__copy(
  const zeus_interfaces__srv__ZeusExecutor_Response * input,
  zeus_interfaces__srv__ZeusExecutor_Response * output);

/// Initialize array of srv/ZeusExecutor messages.
/**
 * It allocates the memory for the number of elements and calls
 * zeus_interfaces__srv__ZeusExecutor_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_zeus_interfaces
bool
zeus_interfaces__srv__ZeusExecutor_Response__Sequence__init(zeus_interfaces__srv__ZeusExecutor_Response__Sequence * array, size_t size);

/// Finalize array of srv/ZeusExecutor messages.
/**
 * It calls
 * zeus_interfaces__srv__ZeusExecutor_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_zeus_interfaces
void
zeus_interfaces__srv__ZeusExecutor_Response__Sequence__fini(zeus_interfaces__srv__ZeusExecutor_Response__Sequence * array);

/// Create array of srv/ZeusExecutor messages.
/**
 * It allocates the memory for the array and calls
 * zeus_interfaces__srv__ZeusExecutor_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_zeus_interfaces
zeus_interfaces__srv__ZeusExecutor_Response__Sequence *
zeus_interfaces__srv__ZeusExecutor_Response__Sequence__create(size_t size);

/// Destroy array of srv/ZeusExecutor messages.
/**
 * It calls
 * zeus_interfaces__srv__ZeusExecutor_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_zeus_interfaces
void
zeus_interfaces__srv__ZeusExecutor_Response__Sequence__destroy(zeus_interfaces__srv__ZeusExecutor_Response__Sequence * array);

/// Check for srv/ZeusExecutor message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_zeus_interfaces
bool
zeus_interfaces__srv__ZeusExecutor_Response__Sequence__are_equal(const zeus_interfaces__srv__ZeusExecutor_Response__Sequence * lhs, const zeus_interfaces__srv__ZeusExecutor_Response__Sequence * rhs);

/// Copy an array of srv/ZeusExecutor messages.
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
zeus_interfaces__srv__ZeusExecutor_Response__Sequence__copy(
  const zeus_interfaces__srv__ZeusExecutor_Response__Sequence * input,
  zeus_interfaces__srv__ZeusExecutor_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ZEUS_INTERFACES__SRV__DETAIL__ZEUS_EXECUTOR__FUNCTIONS_H_
