// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from zeus_interfaces:msg/ZeusMainCommand.idl
// generated code does not contain a copyright notice

#ifndef ZEUS_INTERFACES__MSG__DETAIL__ZEUS_MAIN_COMMAND__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define ZEUS_INTERFACES__MSG__DETAIL__ZEUS_MAIN_COMMAND__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "zeus_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "zeus_interfaces/msg/detail/zeus_main_command__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace zeus_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_zeus_interfaces
cdr_serialize(
  const zeus_interfaces::msg::ZeusMainCommand & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_zeus_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  zeus_interfaces::msg::ZeusMainCommand & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_zeus_interfaces
get_serialized_size(
  const zeus_interfaces::msg::ZeusMainCommand & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_zeus_interfaces
max_serialized_size_ZeusMainCommand(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace zeus_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_zeus_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, zeus_interfaces, msg, ZeusMainCommand)();

#ifdef __cplusplus
}
#endif

#endif  // ZEUS_INTERFACES__MSG__DETAIL__ZEUS_MAIN_COMMAND__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
