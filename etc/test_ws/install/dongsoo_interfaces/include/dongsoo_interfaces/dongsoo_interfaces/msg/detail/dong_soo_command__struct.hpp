// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dongsoo_interfaces:msg/DongSooCommand.idl
// generated code does not contain a copyright notice

#ifndef DONGSOO_INTERFACES__MSG__DETAIL__DONG_SOO_COMMAND__STRUCT_HPP_
#define DONGSOO_INTERFACES__MSG__DETAIL__DONG_SOO_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__dongsoo_interfaces__msg__DongSooCommand __attribute__((deprecated))
#else
# define DEPRECATED__dongsoo_interfaces__msg__DongSooCommand __declspec(deprecated)
#endif

namespace dongsoo_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DongSooCommand_
{
  using Type = DongSooCommand_<ContainerAllocator>;

  explicit DongSooCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 3>::iterator, float>(this->position.begin(), this->position.end(), 0.0f);
      this->look = "";
      this->time = 0.0f;
      this->wrist = 0.0f;
    }
  }

  explicit DongSooCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : position(_alloc),
    look(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<float, 3>::iterator, float>(this->position.begin(), this->position.end(), 0.0f);
      this->look = "";
      this->time = 0.0f;
      this->wrist = 0.0f;
    }
  }

  // field types and members
  using _position_type =
    std::array<float, 3>;
  _position_type position;
  using _look_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _look_type look;
  using _time_type =
    float;
  _time_type time;
  using _wrist_type =
    float;
  _wrist_type wrist;

  // setters for named parameter idiom
  Type & set__position(
    const std::array<float, 3> & _arg)
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
  Type & set__time(
    const float & _arg)
  {
    this->time = _arg;
    return *this;
  }
  Type & set__wrist(
    const float & _arg)
  {
    this->wrist = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dongsoo_interfaces::msg::DongSooCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const dongsoo_interfaces::msg::DongSooCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dongsoo_interfaces::msg::DongSooCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dongsoo_interfaces::msg::DongSooCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dongsoo_interfaces::msg::DongSooCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dongsoo_interfaces::msg::DongSooCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dongsoo_interfaces::msg::DongSooCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dongsoo_interfaces::msg::DongSooCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dongsoo_interfaces::msg::DongSooCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dongsoo_interfaces::msg::DongSooCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dongsoo_interfaces__msg__DongSooCommand
    std::shared_ptr<dongsoo_interfaces::msg::DongSooCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dongsoo_interfaces__msg__DongSooCommand
    std::shared_ptr<dongsoo_interfaces::msg::DongSooCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DongSooCommand_ & other) const
  {
    if (this->position != other.position) {
      return false;
    }
    if (this->look != other.look) {
      return false;
    }
    if (this->time != other.time) {
      return false;
    }
    if (this->wrist != other.wrist) {
      return false;
    }
    return true;
  }
  bool operator!=(const DongSooCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DongSooCommand_

// alias to use template instance with default allocator
using DongSooCommand =
  dongsoo_interfaces::msg::DongSooCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace dongsoo_interfaces

#endif  // DONGSOO_INTERFACES__MSG__DETAIL__DONG_SOO_COMMAND__STRUCT_HPP_
