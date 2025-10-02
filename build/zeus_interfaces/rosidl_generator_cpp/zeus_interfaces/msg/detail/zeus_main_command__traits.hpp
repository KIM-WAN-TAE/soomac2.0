// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from zeus_interfaces:msg/ZeusMainCommand.idl
// generated code does not contain a copyright notice

#ifndef ZEUS_INTERFACES__MSG__DETAIL__ZEUS_MAIN_COMMAND__TRAITS_HPP_
#define ZEUS_INTERFACES__MSG__DETAIL__ZEUS_MAIN_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "zeus_interfaces/msg/detail/zeus_main_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace zeus_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ZeusMainCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: frame
  {
    out << "frame: ";
    rosidl_generator_traits::value_to_yaml(msg.frame, out);
    out << ", ";
  }

  // member: position
  {
    if (msg.position.size() == 0) {
      out << "position: []";
    } else {
      out << "position: [";
      size_t pending_items = msg.position.size();
      for (auto item : msg.position) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: speed
  {
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ZeusMainCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: frame
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "frame: ";
    rosidl_generator_traits::value_to_yaml(msg.frame, out);
    out << "\n";
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.position.size() == 0) {
      out << "position: []\n";
    } else {
      out << "position:\n";
      for (auto item : msg.position) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ZeusMainCommand & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace zeus_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use zeus_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const zeus_interfaces::msg::ZeusMainCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  zeus_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use zeus_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const zeus_interfaces::msg::ZeusMainCommand & msg)
{
  return zeus_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<zeus_interfaces::msg::ZeusMainCommand>()
{
  return "zeus_interfaces::msg::ZeusMainCommand";
}

template<>
inline const char * name<zeus_interfaces::msg::ZeusMainCommand>()
{
  return "zeus_interfaces/msg/ZeusMainCommand";
}

template<>
struct has_fixed_size<zeus_interfaces::msg::ZeusMainCommand>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<zeus_interfaces::msg::ZeusMainCommand>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<zeus_interfaces::msg::ZeusMainCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ZEUS_INTERFACES__MSG__DETAIL__ZEUS_MAIN_COMMAND__TRAITS_HPP_
