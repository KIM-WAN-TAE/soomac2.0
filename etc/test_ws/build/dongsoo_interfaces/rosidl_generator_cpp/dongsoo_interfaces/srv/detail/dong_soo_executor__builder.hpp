// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dongsoo_interfaces:srv/DongSooExecutor.idl
// generated code does not contain a copyright notice

#ifndef DONGSOO_INTERFACES__SRV__DETAIL__DONG_SOO_EXECUTOR__BUILDER_HPP_
#define DONGSOO_INTERFACES__SRV__DETAIL__DONG_SOO_EXECUTOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "dongsoo_interfaces/srv/detail/dong_soo_executor__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace dongsoo_interfaces
{

namespace srv
{

namespace builder
{

class Init_DongSooExecutor_Request_wrist
{
public:
  explicit Init_DongSooExecutor_Request_wrist(::dongsoo_interfaces::srv::DongSooExecutor_Request & msg)
  : msg_(msg)
  {}
  ::dongsoo_interfaces::srv::DongSooExecutor_Request wrist(::dongsoo_interfaces::srv::DongSooExecutor_Request::_wrist_type arg)
  {
    msg_.wrist = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dongsoo_interfaces::srv::DongSooExecutor_Request msg_;
};

class Init_DongSooExecutor_Request_time
{
public:
  explicit Init_DongSooExecutor_Request_time(::dongsoo_interfaces::srv::DongSooExecutor_Request & msg)
  : msg_(msg)
  {}
  Init_DongSooExecutor_Request_wrist time(::dongsoo_interfaces::srv::DongSooExecutor_Request::_time_type arg)
  {
    msg_.time = std::move(arg);
    return Init_DongSooExecutor_Request_wrist(msg_);
  }

private:
  ::dongsoo_interfaces::srv::DongSooExecutor_Request msg_;
};

class Init_DongSooExecutor_Request_look
{
public:
  explicit Init_DongSooExecutor_Request_look(::dongsoo_interfaces::srv::DongSooExecutor_Request & msg)
  : msg_(msg)
  {}
  Init_DongSooExecutor_Request_time look(::dongsoo_interfaces::srv::DongSooExecutor_Request::_look_type arg)
  {
    msg_.look = std::move(arg);
    return Init_DongSooExecutor_Request_time(msg_);
  }

private:
  ::dongsoo_interfaces::srv::DongSooExecutor_Request msg_;
};

class Init_DongSooExecutor_Request_position
{
public:
  Init_DongSooExecutor_Request_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DongSooExecutor_Request_look position(::dongsoo_interfaces::srv::DongSooExecutor_Request::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_DongSooExecutor_Request_look(msg_);
  }

private:
  ::dongsoo_interfaces::srv::DongSooExecutor_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dongsoo_interfaces::srv::DongSooExecutor_Request>()
{
  return dongsoo_interfaces::srv::builder::Init_DongSooExecutor_Request_position();
}

}  // namespace dongsoo_interfaces


namespace dongsoo_interfaces
{

namespace srv
{

namespace builder
{

class Init_DongSooExecutor_Response_success
{
public:
  Init_DongSooExecutor_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::dongsoo_interfaces::srv::DongSooExecutor_Response success(::dongsoo_interfaces::srv::DongSooExecutor_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dongsoo_interfaces::srv::DongSooExecutor_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dongsoo_interfaces::srv::DongSooExecutor_Response>()
{
  return dongsoo_interfaces::srv::builder::Init_DongSooExecutor_Response_success();
}

}  // namespace dongsoo_interfaces

#endif  // DONGSOO_INTERFACES__SRV__DETAIL__DONG_SOO_EXECUTOR__BUILDER_HPP_
