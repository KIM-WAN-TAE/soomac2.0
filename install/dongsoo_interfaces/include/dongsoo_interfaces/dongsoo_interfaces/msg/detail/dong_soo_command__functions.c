// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dongsoo_interfaces:msg/DongSooCommand.idl
// generated code does not contain a copyright notice
#include "dongsoo_interfaces/msg/detail/dong_soo_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `look`
#include "rosidl_runtime_c/string_functions.h"

bool
dongsoo_interfaces__msg__DongSooCommand__init(dongsoo_interfaces__msg__DongSooCommand * msg)
{
  if (!msg) {
    return false;
  }
  // position
  // look
  if (!rosidl_runtime_c__String__init(&msg->look)) {
    dongsoo_interfaces__msg__DongSooCommand__fini(msg);
    return false;
  }
  return true;
}

void
dongsoo_interfaces__msg__DongSooCommand__fini(dongsoo_interfaces__msg__DongSooCommand * msg)
{
  if (!msg) {
    return;
  }
  // position
  // look
  rosidl_runtime_c__String__fini(&msg->look);
}

bool
dongsoo_interfaces__msg__DongSooCommand__are_equal(const dongsoo_interfaces__msg__DongSooCommand * lhs, const dongsoo_interfaces__msg__DongSooCommand * rhs)
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
  return true;
}

bool
dongsoo_interfaces__msg__DongSooCommand__copy(
  const dongsoo_interfaces__msg__DongSooCommand * input,
  dongsoo_interfaces__msg__DongSooCommand * output)
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
  return true;
}

dongsoo_interfaces__msg__DongSooCommand *
dongsoo_interfaces__msg__DongSooCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dongsoo_interfaces__msg__DongSooCommand * msg = (dongsoo_interfaces__msg__DongSooCommand *)allocator.allocate(sizeof(dongsoo_interfaces__msg__DongSooCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dongsoo_interfaces__msg__DongSooCommand));
  bool success = dongsoo_interfaces__msg__DongSooCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dongsoo_interfaces__msg__DongSooCommand__destroy(dongsoo_interfaces__msg__DongSooCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dongsoo_interfaces__msg__DongSooCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dongsoo_interfaces__msg__DongSooCommand__Sequence__init(dongsoo_interfaces__msg__DongSooCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dongsoo_interfaces__msg__DongSooCommand * data = NULL;

  if (size) {
    data = (dongsoo_interfaces__msg__DongSooCommand *)allocator.zero_allocate(size, sizeof(dongsoo_interfaces__msg__DongSooCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dongsoo_interfaces__msg__DongSooCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dongsoo_interfaces__msg__DongSooCommand__fini(&data[i - 1]);
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
dongsoo_interfaces__msg__DongSooCommand__Sequence__fini(dongsoo_interfaces__msg__DongSooCommand__Sequence * array)
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
      dongsoo_interfaces__msg__DongSooCommand__fini(&array->data[i]);
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

dongsoo_interfaces__msg__DongSooCommand__Sequence *
dongsoo_interfaces__msg__DongSooCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dongsoo_interfaces__msg__DongSooCommand__Sequence * array = (dongsoo_interfaces__msg__DongSooCommand__Sequence *)allocator.allocate(sizeof(dongsoo_interfaces__msg__DongSooCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dongsoo_interfaces__msg__DongSooCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dongsoo_interfaces__msg__DongSooCommand__Sequence__destroy(dongsoo_interfaces__msg__DongSooCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dongsoo_interfaces__msg__DongSooCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dongsoo_interfaces__msg__DongSooCommand__Sequence__are_equal(const dongsoo_interfaces__msg__DongSooCommand__Sequence * lhs, const dongsoo_interfaces__msg__DongSooCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dongsoo_interfaces__msg__DongSooCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dongsoo_interfaces__msg__DongSooCommand__Sequence__copy(
  const dongsoo_interfaces__msg__DongSooCommand__Sequence * input,
  dongsoo_interfaces__msg__DongSooCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dongsoo_interfaces__msg__DongSooCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dongsoo_interfaces__msg__DongSooCommand * data =
      (dongsoo_interfaces__msg__DongSooCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dongsoo_interfaces__msg__DongSooCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dongsoo_interfaces__msg__DongSooCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dongsoo_interfaces__msg__DongSooCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
