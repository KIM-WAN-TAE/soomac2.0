// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from dongsoo_interfaces:msg/DongSooCommand.idl
// generated code does not contain a copyright notice

#ifndef DONGSOO_INTERFACES__MSG__DETAIL__DONG_SOO_COMMAND__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define DONGSOO_INTERFACES__MSG__DETAIL__DONG_SOO_COMMAND__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "dongsoo_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "dongsoo_interfaces/msg/detail/dong_soo_command__struct.hpp"

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
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dongsoo_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  dongsoo_interfaces::msg::DongSooCommand & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dongsoo_interfaces
get_serialized_size(
  const dongsoo_interfaces::msg::DongSooCommand & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dongsoo_interfaces
max_serialized_size_DongSooCommand(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace dongsoo_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_dongsoo_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, dongsoo_interfaces, msg, DongSooCommand)();

#ifdef __cplusplus
}
#endif

#endif  // DONGSOO_INTERFACES__MSG__DETAIL__DONG_SOO_COMMAND__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
