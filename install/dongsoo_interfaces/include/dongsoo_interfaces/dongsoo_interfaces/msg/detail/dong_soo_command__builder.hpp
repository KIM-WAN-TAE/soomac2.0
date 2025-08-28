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

class Init_DongSooCommand_look
{
public:
  explicit Init_DongSooCommand_look(::dongsoo_interfaces::msg::DongSooCommand & msg)
  : msg_(msg)
  {}
  ::dongsoo_interfaces::msg::DongSooCommand look(::dongsoo_interfaces::msg::DongSooCommand::_look_type arg)
  {
    msg_.look = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dongsoo_interfaces::msg::DongSooCommand msg_;
};

class Init_DongSooCommand_position
{
public:
  Init_DongSooCommand_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DongSooCommand_look position(::dongsoo_interfaces::msg::DongSooCommand::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_DongSooCommand_look(msg_);
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
  return dongsoo_interfaces::msg::builder::Init_DongSooCommand_position();
}

}  // namespace dongsoo_interfaces

#endif  // DONGSOO_INTERFACES__MSG__DETAIL__DONG_SOO_COMMAND__BUILDER_HPP_
