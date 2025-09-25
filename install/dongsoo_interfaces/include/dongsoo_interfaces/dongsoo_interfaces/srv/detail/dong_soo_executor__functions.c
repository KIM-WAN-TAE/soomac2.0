// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dongsoo_interfaces:srv/DongSooExecutor.idl
// generated code does not contain a copyright notice
#include "dongsoo_interfaces/srv/detail/dong_soo_executor__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `look`
#include "rosidl_runtime_c/string_functions.h"

bool
dongsoo_interfaces__srv__DongSooExecutor_Request__init(dongsoo_interfaces__srv__DongSooExecutor_Request * msg)
{
  if (!msg) {
    return false;
  }
  // position
  // look
  if (!rosidl_runtime_c__String__init(&msg->look)) {
    dongsoo_interfaces__srv__DongSooExecutor_Request__fini(msg);
    return false;
  }
  // time
  return true;
}

void
dongsoo_interfaces__srv__DongSooExecutor_Request__fini(dongsoo_interfaces__srv__DongSooExecutor_Request * msg)
{
  if (!msg) {
    return;
  }
  // position
  // look
  rosidl_runtime_c__String__fini(&msg->look);
  // time
}

bool
dongsoo_interfaces__srv__DongSooExecutor_Request__are_equal(const dongsoo_interfaces__srv__DongSooExecutor_Request * lhs, const dongsoo_interfaces__srv__DongSooExecutor_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // position
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->position[i] != rhs->position[i]) {
      return false;
    }
  }
  // look
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->look), &(rhs->look)))
  {
    return false;
  }
  // time
  if (lhs->time != rhs->time) {
    return false;
  }
  return true;
}

bool
dongsoo_interfaces__srv__DongSooExecutor_Request__copy(
  const dongsoo_interfaces__srv__DongSooExecutor_Request * input,
  dongsoo_interfaces__srv__DongSooExecutor_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // position
  for (size_t i = 0; i < 3; ++i) {
    output->position[i] = input->position[i];
  }
  // look
  if (!rosidl_runtime_c__String__copy(
      &(input->look), &(output->look)))
  {
    return false;
  }
  // time
  output->time = input->time;
  return true;
}

dongsoo_interfaces__srv__DongSooExecutor_Request *
dongsoo_interfaces__srv__DongSooExecutor_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dongsoo_interfaces__srv__DongSooExecutor_Request * msg = (dongsoo_interfaces__srv__DongSooExecutor_Request *)allocator.allocate(sizeof(dongsoo_interfaces__srv__DongSooExecutor_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dongsoo_interfaces__srv__DongSooExecutor_Request));
  bool success = dongsoo_interfaces__srv__DongSooExecutor_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dongsoo_interfaces__srv__DongSooExecutor_Request__destroy(dongsoo_interfaces__srv__DongSooExecutor_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dongsoo_interfaces__srv__DongSooExecutor_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence__init(dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dongsoo_interfaces__srv__DongSooExecutor_Request * data = NULL;

  if (size) {
    data = (dongsoo_interfaces__srv__DongSooExecutor_Request *)allocator.zero_allocate(size, sizeof(dongsoo_interfaces__srv__DongSooExecutor_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dongsoo_interfaces__srv__DongSooExecutor_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dongsoo_interfaces__srv__DongSooExecutor_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence__fini(dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      dongsoo_interfaces__srv__DongSooExecutor_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence *
dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence * array = (dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence *)allocator.allocate(sizeof(dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence__destroy(dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence__are_equal(const dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence * lhs, const dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dongsoo_interfaces__srv__DongSooExecutor_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence__copy(
  const dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence * input,
  dongsoo_interfaces__srv__DongSooExecutor_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dongsoo_interfaces__srv__DongSooExecutor_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dongsoo_interfaces__srv__DongSooExecutor_Request * data =
      (dongsoo_interfaces__srv__DongSooExecutor_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dongsoo_interfaces__srv__DongSooExecutor_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dongsoo_interfaces__srv__DongSooExecutor_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dongsoo_interfaces__srv__DongSooExecutor_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
dongsoo_interfaces__srv__DongSooExecutor_Response__init(dongsoo_interfaces__srv__DongSooExecutor_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
dongsoo_interfaces__srv__DongSooExecutor_Response__fini(dongsoo_interfaces__srv__DongSooExecutor_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
dongsoo_interfaces__srv__DongSooExecutor_Response__are_equal(const dongsoo_interfaces__srv__DongSooExecutor_Response * lhs, const dongsoo_interfaces__srv__DongSooExecutor_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
dongsoo_interfaces__srv__DongSooExecutor_Response__copy(
  const dongsoo_interfaces__srv__DongSooExecutor_Response * input,
  dongsoo_interfaces__srv__DongSooExecutor_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

dongsoo_interfaces__srv__DongSooExecutor_Response *
dongsoo_interfaces__srv__DongSooExecutor_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dongsoo_interfaces__srv__DongSooExecutor_Response * msg = (dongsoo_interfaces__srv__DongSooExecutor_Response *)allocator.allocate(sizeof(dongsoo_interfaces__srv__DongSooExecutor_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dongsoo_interfaces__srv__DongSooExecutor_Response));
  bool success = dongsoo_interfaces__srv__DongSooExecutor_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dongsoo_interfaces__srv__DongSooExecutor_Response__destroy(dongsoo_interfaces__srv__DongSooExecutor_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dongsoo_interfaces__srv__DongSooExecutor_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence__init(dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dongsoo_interfaces__srv__DongSooExecutor_Response * data = NULL;

  if (size) {
    data = (dongsoo_interfaces__srv__DongSooExecutor_Response *)allocator.zero_allocate(size, sizeof(dongsoo_interfaces__srv__DongSooExecutor_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dongsoo_interfaces__srv__DongSooExecutor_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dongsoo_interfaces__srv__DongSooExecutor_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence__fini(dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      dongsoo_interfaces__srv__DongSooExecutor_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence *
dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence * array = (dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence *)allocator.allocate(sizeof(dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence__destroy(dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence__are_equal(const dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence * lhs, const dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dongsoo_interfaces__srv__DongSooExecutor_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence__copy(
  const dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence * input,
  dongsoo_interfaces__srv__DongSooExecutor_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dongsoo_interfaces__srv__DongSooExecutor_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dongsoo_interfaces__srv__DongSooExecutor_Response * data =
      (dongsoo_interfaces__srv__DongSooExecutor_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dongsoo_interfaces__srv__DongSooExecutor_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dongsoo_interfaces__srv__DongSooExecutor_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dongsoo_interfaces__srv__DongSooExecutor_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
