// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from zeus_interfaces:msg/ZeusMainCommand.idl
// generated code does not contain a copyright notice

#ifndef ZEUS_INTERFACES__MSG__DETAIL__ZEUS_MAIN_COMMAND__STRUCT_HPP_
#define ZEUS_INTERFACES__MSG__DETAIL__ZEUS_MAIN_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__zeus_interfaces__msg__ZeusMainCommand __attribute__((deprecated))
#else
# define DEPRECATED__zeus_interfaces__msg__ZeusMainCommand __declspec(deprecated)
#endif

namespace zeus_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ZeusMainCommand_
{
  using Type = ZeusMainCommand_<ContainerAllocator>;

  explicit ZeusMainCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->frame = "";
      std::fill<typename std::array<float, 6>::iterator, float>(this->position.begin(), this->position.end(), 0.0f);
      this->speed = 0.0f;
    }
  }

  explicit ZeusMainCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : frame(_alloc),
    position(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->frame = "";
      std::fill<typename std::array<float, 6>::iterator, float>(this->position.begin(), this->position.end(), 0.0f);
      this->speed = 0.0f;
    }
  }

  // field types and members
  using _frame_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _frame_type frame;
  using _position_type =
    std::array<float, 6>;
  _position_type position;
  using _speed_type =
    float;
  _speed_type speed;

  // setters for named parameter idiom
  Type & set__frame(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->frame = _arg;
    return *this;
  }
  Type & set__position(
    const std::array<float, 6> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__speed(
    const float & _arg)
  {
    this->speed = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    zeus_interfaces::msg::ZeusMainCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const zeus_interfaces::msg::ZeusMainCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<zeus_interfaces::msg::ZeusMainCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<zeus_interfaces::msg::ZeusMainCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      zeus_interfaces::msg::ZeusMainCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<zeus_interfaces::msg::ZeusMainCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      zeus_interfaces::msg::ZeusMainCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<zeus_interfaces::msg::ZeusMainCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<zeus_interfaces::msg::ZeusMainCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<zeus_interfaces::msg::ZeusMainCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__zeus_interfaces__msg__ZeusMainCommand
    std::shared_ptr<zeus_interfaces::msg::ZeusMainCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__zeus_interfaces__msg__ZeusMainCommand
    std::shared_ptr<zeus_interfaces::msg::ZeusMainCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ZeusMainCommand_ & other) const
  {
    if (this->frame != other.frame) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    return true;
  }
  bool operator!=(const ZeusMainCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ZeusMainCommand_

// alias to use template instance with default allocator
using ZeusMainCommand =
  zeus_interfaces::msg::ZeusMainCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace zeus_interfaces

#endif  // ZEUS_INTERFACES__MSG__DETAIL__ZEUS_MAIN_COMMAND__STRUCT_HPP_
