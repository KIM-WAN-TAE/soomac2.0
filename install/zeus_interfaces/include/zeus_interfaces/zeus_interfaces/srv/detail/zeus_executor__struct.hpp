// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from zeus_interfaces:srv/ZeusExecutor.idl
// generated code does not contain a copyright notice

#ifndef ZEUS_INTERFACES__SRV__DETAIL__ZEUS_EXECUTOR__STRUCT_HPP_
#define ZEUS_INTERFACES__SRV__DETAIL__ZEUS_EXECUTOR__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__zeus_interfaces__srv__ZeusExecutor_Request __attribute__((deprecated))
#else
# define DEPRECATED__zeus_interfaces__srv__ZeusExecutor_Request __declspec(deprecated)
#endif

namespace zeus_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ZeusExecutor_Request_
{
  using Type = ZeusExecutor_Request_<ContainerAllocator>;

  explicit ZeusExecutor_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->frame = "";
      std::fill<typename std::array<float, 6>::iterator, float>(this->coordinate.begin(), this->coordinate.end(), 0.0f);
    }
  }

  explicit ZeusExecutor_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : frame(_alloc),
    coordinate(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->frame = "";
      std::fill<typename std::array<float, 6>::iterator, float>(this->coordinate.begin(), this->coordinate.end(), 0.0f);
    }
  }

  // field types and members
  using _frame_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _frame_type frame;
  using _coordinate_type =
    std::array<float, 6>;
  _coordinate_type coordinate;

  // setters for named parameter idiom
  Type & set__frame(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->frame = _arg;
    return *this;
  }
  Type & set__coordinate(
    const std::array<float, 6> & _arg)
  {
    this->coordinate = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    zeus_interfaces::srv::ZeusExecutor_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const zeus_interfaces::srv::ZeusExecutor_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<zeus_interfaces::srv::ZeusExecutor_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<zeus_interfaces::srv::ZeusExecutor_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      zeus_interfaces::srv::ZeusExecutor_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<zeus_interfaces::srv::ZeusExecutor_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      zeus_interfaces::srv::ZeusExecutor_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<zeus_interfaces::srv::ZeusExecutor_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<zeus_interfaces::srv::ZeusExecutor_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<zeus_interfaces::srv::ZeusExecutor_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__zeus_interfaces__srv__ZeusExecutor_Request
    std::shared_ptr<zeus_interfaces::srv::ZeusExecutor_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__zeus_interfaces__srv__ZeusExecutor_Request
    std::shared_ptr<zeus_interfaces::srv::ZeusExecutor_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ZeusExecutor_Request_ & other) const
  {
    if (this->frame != other.frame) {
      return false;
    }
    if (this->coordinate != other.coordinate) {
      return false;
    }
    return true;
  }
  bool operator!=(const ZeusExecutor_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ZeusExecutor_Request_

// alias to use template instance with default allocator
using ZeusExecutor_Request =
  zeus_interfaces::srv::ZeusExecutor_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace zeus_interfaces


#ifndef _WIN32
# define DEPRECATED__zeus_interfaces__srv__ZeusExecutor_Response __attribute__((deprecated))
#else
# define DEPRECATED__zeus_interfaces__srv__ZeusExecutor_Response __declspec(deprecated)
#endif

namespace zeus_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ZeusExecutor_Response_
{
  using Type = ZeusExecutor_Response_<ContainerAllocator>;

  explicit ZeusExecutor_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit ZeusExecutor_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    zeus_interfaces::srv::ZeusExecutor_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const zeus_interfaces::srv::ZeusExecutor_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<zeus_interfaces::srv::ZeusExecutor_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<zeus_interfaces::srv::ZeusExecutor_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      zeus_interfaces::srv::ZeusExecutor_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<zeus_interfaces::srv::ZeusExecutor_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      zeus_interfaces::srv::ZeusExecutor_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<zeus_interfaces::srv::ZeusExecutor_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<zeus_interfaces::srv::ZeusExecutor_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<zeus_interfaces::srv::ZeusExecutor_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__zeus_interfaces__srv__ZeusExecutor_Response
    std::shared_ptr<zeus_interfaces::srv::ZeusExecutor_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__zeus_interfaces__srv__ZeusExecutor_Response
    std::shared_ptr<zeus_interfaces::srv::ZeusExecutor_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ZeusExecutor_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const ZeusExecutor_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ZeusExecutor_Response_

// alias to use template instance with default allocator
using ZeusExecutor_Response =
  zeus_interfaces::srv::ZeusExecutor_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace zeus_interfaces

namespace zeus_interfaces
{

namespace srv
{

struct ZeusExecutor
{
  using Request = zeus_interfaces::srv::ZeusExecutor_Request;
  using Response = zeus_interfaces::srv::ZeusExecutor_Response;
};

}  // namespace srv

}  // namespace zeus_interfaces

#endif  // ZEUS_INTERFACES__SRV__DETAIL__ZEUS_EXECUTOR__STRUCT_HPP_
