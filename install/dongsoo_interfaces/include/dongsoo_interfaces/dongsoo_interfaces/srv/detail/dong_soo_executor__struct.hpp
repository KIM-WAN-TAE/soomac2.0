// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dongsoo_interfaces:srv/DongSooExecutor.idl
// generated code does not contain a copyright notice

#ifndef DONGSOO_INTERFACES__SRV__DETAIL__DONG_SOO_EXECUTOR__STRUCT_HPP_
#define DONGSOO_INTERFACES__SRV__DETAIL__DONG_SOO_EXECUTOR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dongsoo_interfaces__srv__DongSooExecutor_Request __attribute__((deprecated))
#else
# define DEPRECATED__dongsoo_interfaces__srv__DongSooExecutor_Request __declspec(deprecated)
#endif

namespace dongsoo_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct DongSooExecutor_Request_
{
  using Type = DongSooExecutor_Request_<ContainerAllocator>;

  explicit DongSooExecutor_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 16>::iterator, float>(this->position.begin(), this->position.end(), 0.0f);
      this->look = "";
    }
  }

  explicit DongSooExecutor_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position(_alloc),
    look(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 16>::iterator, float>(this->position.begin(), this->position.end(), 0.0f);
      this->look = "";
    }
  }

  // field types and members
  using _position_type =
    std::array<float, 16>;
  _position_type position;
  using _look_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _look_type look;

  // setters for named parameter idiom
  Type & set__position(
    const std::array<float, 16> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__look(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->look = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dongsoo_interfaces::srv::DongSooExecutor_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const dongsoo_interfaces::srv::DongSooExecutor_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dongsoo_interfaces::srv::DongSooExecutor_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dongsoo_interfaces::srv::DongSooExecutor_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dongsoo_interfaces::srv::DongSooExecutor_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dongsoo_interfaces::srv::DongSooExecutor_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dongsoo_interfaces::srv::DongSooExecutor_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dongsoo_interfaces::srv::DongSooExecutor_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dongsoo_interfaces::srv::DongSooExecutor_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dongsoo_interfaces::srv::DongSooExecutor_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dongsoo_interfaces__srv__DongSooExecutor_Request
    std::shared_ptr<dongsoo_interfaces::srv::DongSooExecutor_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dongsoo_interfaces__srv__DongSooExecutor_Request
    std::shared_ptr<dongsoo_interfaces::srv::DongSooExecutor_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DongSooExecutor_Request_ & other) const
  {
    if (this->position != other.position) {
      return false;
    }
    if (this->look != other.look) {
      return false;
    }
    return true;
  }
  bool operator!=(const DongSooExecutor_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DongSooExecutor_Request_

// alias to use template instance with default allocator
using DongSooExecutor_Request =
  dongsoo_interfaces::srv::DongSooExecutor_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dongsoo_interfaces


#ifndef _WIN32
# define DEPRECATED__dongsoo_interfaces__srv__DongSooExecutor_Response __attribute__((deprecated))
#else
# define DEPRECATED__dongsoo_interfaces__srv__DongSooExecutor_Response __declspec(deprecated)
#endif

namespace dongsoo_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct DongSooExecutor_Response_
{
  using Type = DongSooExecutor_Response_<ContainerAllocator>;

  explicit DongSooExecutor_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit DongSooExecutor_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dongsoo_interfaces::srv::DongSooExecutor_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const dongsoo_interfaces::srv::DongSooExecutor_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dongsoo_interfaces::srv::DongSooExecutor_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dongsoo_interfaces::srv::DongSooExecutor_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dongsoo_interfaces::srv::DongSooExecutor_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dongsoo_interfaces::srv::DongSooExecutor_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dongsoo_interfaces::srv::DongSooExecutor_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dongsoo_interfaces::srv::DongSooExecutor_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dongsoo_interfaces::srv::DongSooExecutor_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dongsoo_interfaces::srv::DongSooExecutor_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dongsoo_interfaces__srv__DongSooExecutor_Response
    std::shared_ptr<dongsoo_interfaces::srv::DongSooExecutor_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dongsoo_interfaces__srv__DongSooExecutor_Response
    std::shared_ptr<dongsoo_interfaces::srv::DongSooExecutor_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DongSooExecutor_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const DongSooExecutor_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DongSooExecutor_Response_

// alias to use template instance with default allocator
using DongSooExecutor_Response =
  dongsoo_interfaces::srv::DongSooExecutor_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dongsoo_interfaces

namespace dongsoo_interfaces
{

namespace srv
{

struct DongSooExecutor
{
  using Request = dongsoo_interfaces::srv::DongSooExecutor_Request;
  using Response = dongsoo_interfaces::srv::DongSooExecutor_Response;
};

}  // namespace srv

}  // namespace dongsoo_interfaces

#endif  // DONGSOO_INTERFACES__SRV__DETAIL__DONG_SOO_EXECUTOR__STRUCT_HPP_
