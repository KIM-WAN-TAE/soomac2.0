// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from dongsoo_interfaces:msg/DongSooCommand.idl
// generated code does not contain a copyright notice
#include "dongsoo_interfaces/msg/detail/dong_soo_command__rosidl_typesupport_fastrtps_cpp.hpp"
#include "dongsoo_interfaces/msg/detail/dong_soo_command__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace dongsoo_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dongsoo_interfaces
cdr_serialize(
  const dongsoo_interfaces::msg::DongSooCommand & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: frame
  cdr << ros_message.frame;
  // Member: position
  {
    cdr << ros_message.position;
  }
  // Member: look
  cdr << ros_message.look;
  // Member: time
  cdr << ros_message.time;
  // Member: wrist
  cdr << ros_message.wrist;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dongsoo_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  dongsoo_interfaces::msg::DongSooCommand & ros_message)
{
  // Member: frame
  cdr >> ros_message.frame;

  // Member: position
  {
    cdr >> ros_message.position;
  }

  // Member: look
  cdr >> ros_message.look;

  // Member: time
  cdr >> ros_message.time;

  // Member: wrist
  cdr >> ros_message.wrist;

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dongsoo_interfaces
get_serialized_size(
  const dongsoo_interfaces::msg::DongSooCommand & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: frame
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.frame.size() + 1);
  // Member: position
  {
    size_t array_size = 4;
    size_t item_size = sizeof(ros_message.position[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: look
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.look.size() + 1);
  // Member: time
  {
    size_t item_size = sizeof(ros_message.time);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: wrist
  {
    size_t item_size = sizeof(ros_message.wrist);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dongsoo_interfaces
max_serialized_size_DongSooCommand(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: frame
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: position
  {
    size_t array_size = 4;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: look
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: time
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: wrist
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = dongsoo_interfaces::msg::DongSooCommand;
    is_plain =
      (
      offsetof(DataType, wrist) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _DongSooCommand__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const dongsoo_interfaces::msg::DongSooCommand *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _DongSooCommand__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<dongsoo_interfaces::msg::DongSooCommand *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _DongSooCommand__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const dongsoo_interfaces::msg::DongSooCommand *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _DongSooCommand__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_DongSooCommand(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _DongSooCommand__callbacks = {
  "dongsoo_interfaces::msg",
  "DongSooCommand",
  _DongSooCommand__cdr_serialize,
  _DongSooCommand__cdr_deserialize,
  _DongSooCommand__get_serialized_size,
  _DongSooCommand__max_serialized_size
};

static rosidl_message_type_support_t _DongSooCommand__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_DongSooCommand__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace dongsoo_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_dongsoo_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<dongsoo_interfaces::msg::DongSooCommand>()
{
  return &dongsoo_interfaces::msg::typesupport_fastrtps_cpp::_DongSooCommand__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, dongsoo_interfaces, msg, DongSooCommand)() {
  return &dongsoo_interfaces::msg::typesupport_fastrtps_cpp::_DongSooCommand__handle;
}

#ifdef __cplusplus
}
#endif
