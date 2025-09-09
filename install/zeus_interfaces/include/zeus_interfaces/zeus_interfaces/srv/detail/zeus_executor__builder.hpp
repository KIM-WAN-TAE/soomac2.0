// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from zeus_interfaces:srv/ZeusExecutor.idl
// generated code does not contain a copyright notice

#ifndef ZEUS_INTERFACES__SRV__DETAIL__ZEUS_EXECUTOR__BUILDER_HPP_
#define ZEUS_INTERFACES__SRV__DETAIL__ZEUS_EXECUTOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "zeus_interfaces/srv/detail/zeus_executor__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace zeus_interfaces
{

namespace srv
{

namespace builder
{

class Init_ZeusExecutor_Request_coordinate
{
public:
  explicit Init_ZeusExecutor_Request_coordinate(::zeus_interfaces::srv::ZeusExecutor_Request & msg)
  : msg_(msg)
  {}
  ::zeus_interfaces::srv::ZeusExecutor_Request coordinate(::zeus_interfaces::srv::ZeusExecutor_Request::_coordinate_type arg)
  {
    msg_.coordinate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::zeus_interfaces::srv::ZeusExecutor_Request msg_;
};

class Init_ZeusExecutor_Request_frame
{
public:
  Init_ZeusExecutor_Request_frame()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ZeusExecutor_Request_coordinate frame(::zeus_interfaces::srv::ZeusExecutor_Request::_frame_type arg)
  {
    msg_.frame = std::move(arg);
    return Init_ZeusExecutor_Request_coordinate(msg_);
  }

private:
  ::zeus_interfaces::srv::ZeusExecutor_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::zeus_interfaces::srv::ZeusExecutor_Request>()
{
  return zeus_interfaces::srv::builder::Init_ZeusExecutor_Request_frame();
}

}  // namespace zeus_interfaces


namespace zeus_interfaces
{

namespace srv
{

namespace builder
{

class Init_ZeusExecutor_Response_success
{
public:
  Init_ZeusExecutor_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::zeus_interfaces::srv::ZeusExecutor_Response success(::zeus_interfaces::srv::ZeusExecutor_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::zeus_interfaces::srv::ZeusExecutor_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::zeus_interfaces::srv::ZeusExecutor_Response>()
{
  return zeus_interfaces::srv::builder::Init_ZeusExecutor_Response_success();
}

}  // namespace zeus_interfaces

#endif  // ZEUS_INTERFACES__SRV__DETAIL__ZEUS_EXECUTOR__BUILDER_HPP_
