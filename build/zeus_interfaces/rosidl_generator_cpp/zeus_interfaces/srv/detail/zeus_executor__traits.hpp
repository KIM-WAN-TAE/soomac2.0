// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from zeus_interfaces:srv/ZeusExecutor.idl
// generated code does not contain a copyright notice

#ifndef ZEUS_INTERFACES__SRV__DETAIL__ZEUS_EXECUTOR__TRAITS_HPP_
#define ZEUS_INTERFACES__SRV__DETAIL__ZEUS_EXECUTOR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "zeus_interfaces/srv/detail/zeus_executor__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace zeus_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ZeusExecutor_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: frame
  {
    out << "frame: ";
    rosidl_generator_traits::value_to_yaml(msg.frame, out);
    out << ", ";
  }

  // member: coordinate
  {
    if (msg.coordinate.size() == 0) {
      out << "coordinate: []";
    } else {
      out << "coordinate: [";
      size_t pending_items = msg.coordinate.size();
      for (auto item : msg.coordinate) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ZeusExecutor_Request & msg,
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

  // member: coordinate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.coordinate.size() == 0) {
      out << "coordinate: []\n";
    } else {
      out << "coordinate:\n";
      for (auto item : msg.coordinate) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ZeusExecutor_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace zeus_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use zeus_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const zeus_interfaces::srv::ZeusExecutor_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  zeus_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use zeus_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const zeus_interfaces::srv::ZeusExecutor_Request & msg)
{
  return zeus_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<zeus_interfaces::srv::ZeusExecutor_Request>()
{
  return "zeus_interfaces::srv::ZeusExecutor_Request";
}

template<>
inline const char * name<zeus_interfaces::srv::ZeusExecutor_Request>()
{
  return "zeus_interfaces/srv/ZeusExecutor_Request";
}

template<>
struct has_fixed_size<zeus_interfaces::srv::ZeusExecutor_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<zeus_interfaces::srv::ZeusExecutor_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<zeus_interfaces::srv::ZeusExecutor_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace zeus_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ZeusExecutor_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ZeusExecutor_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ZeusExecutor_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace zeus_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use zeus_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const zeus_interfaces::srv::ZeusExecutor_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  zeus_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use zeus_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const zeus_interfaces::srv::ZeusExecutor_Response & msg)
{
  return zeus_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<zeus_interfaces::srv::ZeusExecutor_Response>()
{
  return "zeus_interfaces::srv::ZeusExecutor_Response";
}

template<>
inline const char * name<zeus_interfaces::srv::ZeusExecutor_Response>()
{
  return "zeus_interfaces/srv/ZeusExecutor_Response";
}

template<>
struct has_fixed_size<zeus_interfaces::srv::ZeusExecutor_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<zeus_interfaces::srv::ZeusExecutor_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<zeus_interfaces::srv::ZeusExecutor_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<zeus_interfaces::srv::ZeusExecutor>()
{
  return "zeus_interfaces::srv::ZeusExecutor";
}

template<>
inline const char * name<zeus_interfaces::srv::ZeusExecutor>()
{
  return "zeus_interfaces/srv/ZeusExecutor";
}

template<>
struct has_fixed_size<zeus_interfaces::srv::ZeusExecutor>
  : std::integral_constant<
    bool,
    has_fixed_size<zeus_interfaces::srv::ZeusExecutor_Request>::value &&
    has_fixed_size<zeus_interfaces::srv::ZeusExecutor_Response>::value
  >
{
};

template<>
struct has_bounded_size<zeus_interfaces::srv::ZeusExecutor>
  : std::integral_constant<
    bool,
    has_bounded_size<zeus_interfaces::srv::ZeusExecutor_Request>::value &&
    has_bounded_size<zeus_interfaces::srv::ZeusExecutor_Response>::value
  >
{
};

template<>
struct is_service<zeus_interfaces::srv::ZeusExecutor>
  : std::true_type
{
};

template<>
struct is_service_request<zeus_interfaces::srv::ZeusExecutor_Request>
  : std::true_type
{
};

template<>
struct is_service_response<zeus_interfaces::srv::ZeusExecutor_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ZEUS_INTERFACES__SRV__DETAIL__ZEUS_EXECUTOR__TRAITS_HPP_
