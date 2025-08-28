// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dongsoo_interfaces:msg/DongSooCommand.idl
// generated code does not contain a copyright notice

#ifndef DONGSOO_INTERFACES__MSG__DETAIL__DONG_SOO_COMMAND__TRAITS_HPP_
#define DONGSOO_INTERFACES__MSG__DETAIL__DONG_SOO_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dongsoo_interfaces/msg/detail/dong_soo_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dongsoo_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const DongSooCommand & msg,
  std::ostream & out)
{
  out << "{";
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

  // member: look
  {
    out << "look: ";
    rosidl_generator_traits::value_to_yaml(msg.look, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DongSooCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
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

  // member: look
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "look: ";
    rosidl_generator_traits::value_to_yaml(msg.look, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DongSooCommand & msg, bool use_flow_style = false)
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

}  // namespace dongsoo_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use dongsoo_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dongsoo_interfaces::msg::DongSooCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  dongsoo_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dongsoo_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const dongsoo_interfaces::msg::DongSooCommand & msg)
{
  return dongsoo_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<dongsoo_interfaces::msg::DongSooCommand>()
{
  return "dongsoo_interfaces::msg::DongSooCommand";
}

template<>
inline const char * name<dongsoo_interfaces::msg::DongSooCommand>()
{
  return "dongsoo_interfaces/msg/DongSooCommand";
}

template<>
struct has_fixed_size<dongsoo_interfaces::msg::DongSooCommand>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dongsoo_interfaces::msg::DongSooCommand>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dongsoo_interfaces::msg::DongSooCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DONGSOO_INTERFACES__MSG__DETAIL__DONG_SOO_COMMAND__TRAITS_HPP_
