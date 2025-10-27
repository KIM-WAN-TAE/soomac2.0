// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dongsoo_interfaces:msg/DongSooCommand.idl
// generated code does not contain a copyright notice

#ifndef DONGSOO_INTERFACES__MSG__DETAIL__DONG_SOO_COMMAND__BUILDER_HPP_
#define DONGSOO_INTERFACES__MSG__DETAIL__DONG_SOO_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dongsoo_interfaces/msg/detail/dong_soo_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dongsoo_interfaces
{

namespace msg
{

namespace builder
{

class Init_DongSooCommand_wrist
{
public:
  explicit Init_DongSooCommand_wrist(::dongsoo_interfaces::msg::DongSooCommand & msg)
  : msg_(msg)
  {}
  ::dongsoo_interfaces::msg::DongSooCommand wrist(::dongsoo_interfaces::msg::DongSooCommand::_wrist_type arg)
  {
    msg_.wrist = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dongsoo_interfaces::msg::DongSooCommand msg_;
};

class Init_DongSooCommand_time
{
public:
  explicit Init_DongSooCommand_time(::dongsoo_interfaces::msg::DongSooCommand & msg)
  : msg_(msg)
  {}
  Init_DongSooCommand_wrist time(::dongsoo_interfaces::msg::DongSooCommand::_time_type arg)
  {
    msg_.time = std::move(arg);
    return Init_DongSooCommand_wrist(msg_);
  }

private:
  ::dongsoo_interfaces::msg::DongSooCommand msg_;
};

class Init_DongSooCommand_look
{
public:
  explicit Init_DongSooCommand_look(::dongsoo_interfaces::msg::DongSooCommand & msg)
  : msg_(msg)
  {}
  Init_DongSooCommand_time look(::dongsoo_interfaces::msg::DongSooCommand::_look_type arg)
  {
    msg_.look = std::move(arg);
    return Init_DongSooCommand_time(msg_);
  }

private:
  ::dongsoo_interfaces::msg::DongSooCommand msg_;
};

class Init_DongSooCommand_position
{
public:
  explicit Init_DongSooCommand_position(::dongsoo_interfaces::msg::DongSooCommand & msg)
  : msg_(msg)
  {}
  Init_DongSooCommand_look position(::dongsoo_interfaces::msg::DongSooCommand::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_DongSooCommand_look(msg_);
  }

private:
  ::dongsoo_interfaces::msg::DongSooCommand msg_;
};

class Init_DongSooCommand_frame
{
public:
  Init_DongSooCommand_frame()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DongSooCommand_position frame(::dongsoo_interfaces::msg::DongSooCommand::_frame_type arg)
  {
    msg_.frame = std::move(arg);
    return Init_DongSooCommand_position(msg_);
  }

private:
  ::dongsoo_interfaces::msg::DongSooCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::dongsoo_interfaces::msg::DongSooCommand>()
{
  return dongsoo_interfaces::msg::builder::Init_DongSooCommand_frame();
}

}  // namespace dongsoo_interfaces

#endif  // DONGSOO_INTERFACES__MSG__DETAIL__DONG_SOO_COMMAND__BUILDER_HPP_
