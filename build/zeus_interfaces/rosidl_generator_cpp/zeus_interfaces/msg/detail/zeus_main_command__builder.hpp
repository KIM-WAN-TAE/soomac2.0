// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from zeus_interfaces:msg/ZeusMainCommand.idl
// generated code does not contain a copyright notice

#ifndef ZEUS_INTERFACES__MSG__DETAIL__ZEUS_MAIN_COMMAND__BUILDER_HPP_
#define ZEUS_INTERFACES__MSG__DETAIL__ZEUS_MAIN_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "zeus_interfaces/msg/detail/zeus_main_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace zeus_interfaces
{

namespace msg
{

namespace builder
{

class Init_ZeusMainCommand_speed
{
public:
  explicit Init_ZeusMainCommand_speed(::zeus_interfaces::msg::ZeusMainCommand & msg)
  : msg_(msg)
  {}
  ::zeus_interfaces::msg::ZeusMainCommand speed(::zeus_interfaces::msg::ZeusMainCommand::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::zeus_interfaces::msg::ZeusMainCommand msg_;
};

class Init_ZeusMainCommand_position
{
public:
  explicit Init_ZeusMainCommand_position(::zeus_interfaces::msg::ZeusMainCommand & msg)
  : msg_(msg)
  {}
  Init_ZeusMainCommand_speed position(::zeus_interfaces::msg::ZeusMainCommand::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_ZeusMainCommand_speed(msg_);
  }

private:
  ::zeus_interfaces::msg::ZeusMainCommand msg_;
};

class Init_ZeusMainCommand_frame
{
public:
  Init_ZeusMainCommand_frame()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ZeusMainCommand_position frame(::zeus_interfaces::msg::ZeusMainCommand::_frame_type arg)
  {
    msg_.frame = std::move(arg);
    return Init_ZeusMainCommand_position(msg_);
  }

private:
  ::zeus_interfaces::msg::ZeusMainCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::zeus_interfaces::msg::ZeusMainCommand>()
{
  return zeus_interfaces::msg::builder::Init_ZeusMainCommand_frame();
}

}  // namespace zeus_interfaces

#endif  // ZEUS_INTERFACES__MSG__DETAIL__ZEUS_MAIN_COMMAND__BUILDER_HPP_
