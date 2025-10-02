// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from zeus_interfaces:msg/ZeusMainCommand.idl
// generated code does not contain a copyright notice
#include "zeus_interfaces/msg/detail/zeus_main_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `frame`
#include "rosidl_runtime_c/string_functions.h"

bool
zeus_interfaces__msg__ZeusMainCommand__init(zeus_interfaces__msg__ZeusMainCommand * msg)
{
  if (!msg) {
    return false;
  }
  // frame
  if (!rosidl_runtime_c__String__init(&msg->frame)) {
    zeus_interfaces__msg__ZeusMainCommand__fini(msg);
    return false;
  }
  // position
  // speed
  return true;
}

void
zeus_interfaces__msg__ZeusMainCommand__fini(zeus_interfaces__msg__ZeusMainCommand * msg)
{
  if (!msg) {
    return;
  }
  // frame
  rosidl_runtime_c__String__fini(&msg->frame);
  // position
  // speed
}

bool
zeus_interfaces__msg__ZeusMainCommand__are_equal(const zeus_interfaces__msg__ZeusMainCommand * lhs, const zeus_interfaces__msg__ZeusMainCommand * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // frame
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->frame), &(rhs->frame)))
  {
    return false;
  }
  // position
  for (size_t i = 0; i < 6; ++i) {
    if (lhs->position[i] != rhs->position[i]) {
      return false;
    }
  }
  // speed
  if (lhs->speed != rhs->speed) {
    return false;
  }
  return true;
}

bool
zeus_interfaces__msg__ZeusMainCommand__copy(
  const zeus_interfaces__msg__ZeusMainCommand * input,
  zeus_interfaces__msg__ZeusMainCommand * output)
{
  if (!input || !output) {
    return false;
  }
  // frame
  if (!rosidl_runtime_c__String__copy(
      &(input->frame), &(output->frame)))
  {
    return false;
  }
  // position
  for (size_t i = 0; i < 6; ++i) {
    output->position[i] = input->position[i];
  }
  // speed
  output->speed = input->speed;
  return true;
}

zeus_interfaces__msg__ZeusMainCommand *
zeus_interfaces__msg__ZeusMainCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zeus_interfaces__msg__ZeusMainCommand * msg = (zeus_interfaces__msg__ZeusMainCommand *)allocator.allocate(sizeof(zeus_interfaces__msg__ZeusMainCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(zeus_interfaces__msg__ZeusMainCommand));
  bool success = zeus_interfaces__msg__ZeusMainCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
zeus_interfaces__msg__ZeusMainCommand__destroy(zeus_interfaces__msg__ZeusMainCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    zeus_interfaces__msg__ZeusMainCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
zeus_interfaces__msg__ZeusMainCommand__Sequence__init(zeus_interfaces__msg__ZeusMainCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zeus_interfaces__msg__ZeusMainCommand * data = NULL;

  if (size) {
    data = (zeus_interfaces__msg__ZeusMainCommand *)allocator.zero_allocate(size, sizeof(zeus_interfaces__msg__ZeusMainCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = zeus_interfaces__msg__ZeusMainCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        zeus_interfaces__msg__ZeusMainCommand__fini(&data[i - 1]);
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
zeus_interfaces__msg__ZeusMainCommand__Sequence__fini(zeus_interfaces__msg__ZeusMainCommand__Sequence * array)
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
      zeus_interfaces__msg__ZeusMainCommand__fini(&array->data[i]);
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

zeus_interfaces__msg__ZeusMainCommand__Sequence *
zeus_interfaces__msg__ZeusMainCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  zeus_interfaces__msg__ZeusMainCommand__Sequence * array = (zeus_interfaces__msg__ZeusMainCommand__Sequence *)allocator.allocate(sizeof(zeus_interfaces__msg__ZeusMainCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = zeus_interfaces__msg__ZeusMainCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
zeus_interfaces__msg__ZeusMainCommand__Sequence__destroy(zeus_interfaces__msg__ZeusMainCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    zeus_interfaces__msg__ZeusMainCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
zeus_interfaces__msg__ZeusMainCommand__Sequence__are_equal(const zeus_interfaces__msg__ZeusMainCommand__Sequence * lhs, const zeus_interfaces__msg__ZeusMainCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!zeus_interfaces__msg__ZeusMainCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
zeus_interfaces__msg__ZeusMainCommand__Sequence__copy(
  const zeus_interfaces__msg__ZeusMainCommand__Sequence * input,
  zeus_interfaces__msg__ZeusMainCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(zeus_interfaces__msg__ZeusMainCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    zeus_interfaces__msg__ZeusMainCommand * data =
      (zeus_interfaces__msg__ZeusMainCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!zeus_interfaces__msg__ZeusMainCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          zeus_interfaces__msg__ZeusMainCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!zeus_interfaces__msg__ZeusMainCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
