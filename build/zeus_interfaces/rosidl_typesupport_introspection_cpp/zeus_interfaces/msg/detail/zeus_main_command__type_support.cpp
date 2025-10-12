// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from zeus_interfaces:msg/ZeusMainCommand.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "zeus_interfaces/msg/detail/zeus_main_command__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace zeus_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ZeusMainCommand_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) zeus_interfaces::msg::ZeusMainCommand(_init);
}

void ZeusMainCommand_fini_function(void * message_memory)
{
  auto typed_message = static_cast<zeus_interfaces::msg::ZeusMainCommand *>(message_memory);
  typed_message->~ZeusMainCommand();
}

size_t size_function__ZeusMainCommand__position(const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * get_const_function__ZeusMainCommand__position(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 6> *>(untyped_member);
  return &member[index];
}

void * get_function__ZeusMainCommand__position(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 6> *>(untyped_member);
  return &member[index];
}

void fetch_function__ZeusMainCommand__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__ZeusMainCommand__position(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__ZeusMainCommand__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__ZeusMainCommand__position(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ZeusMainCommand_message_member_array[3] = {
  {
    "frame",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zeus_interfaces::msg::ZeusMainCommand, frame),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "position",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(zeus_interfaces::msg::ZeusMainCommand, position),  // bytes offset in struct
    nullptr,  // default value
    size_function__ZeusMainCommand__position,  // size() function pointer
    get_const_function__ZeusMainCommand__position,  // get_const(index) function pointer
    get_function__ZeusMainCommand__position,  // get(index) function pointer
    fetch_function__ZeusMainCommand__position,  // fetch(index, &value) function pointer
    assign_function__ZeusMainCommand__position,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "speed",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zeus_interfaces::msg::ZeusMainCommand, speed),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ZeusMainCommand_message_members = {
  "zeus_interfaces::msg",  // message namespace
  "ZeusMainCommand",  // message name
  3,  // number of fields
  sizeof(zeus_interfaces::msg::ZeusMainCommand),
  ZeusMainCommand_message_member_array,  // message members
  ZeusMainCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  ZeusMainCommand_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ZeusMainCommand_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ZeusMainCommand_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace zeus_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<zeus_interfaces::msg::ZeusMainCommand>()
{
  return &::zeus_interfaces::msg::rosidl_typesupport_introspection_cpp::ZeusMainCommand_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, zeus_interfaces, msg, ZeusMainCommand)() {
  return &::zeus_interfaces::msg::rosidl_typesupport_introspection_cpp::ZeusMainCommand_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
