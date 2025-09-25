// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dongsoo_interfaces:srv/DongSooExecutor.idl
// generated code does not contain a copyright notice

#ifndef DONGSOO_INTERFACES__SRV__DETAIL__DONG_SOO_EXECUTOR__TRAITS_HPP_
#define DONGSOO_INTERFACES__SRV__DETAIL__DONG_SOO_EXECUTOR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dongsoo_interfaces/srv/detail/dong_soo_executor__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dongsoo_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const DongSooExecutor_Request & msg,
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
    out << ", ";
  }

  // member: time
  {
    out << "time: ";
    rosidl_generator_traits::value_to_yaml(msg.time, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DongSooExecutor_Request & msg,
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

  // member: time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time: ";
    rosidl_generator_traits::value_to_yaml(msg.time, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DongSooExecutor_Request & msg, bool use_flow_style = false)
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

}  // namespace dongsoo_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use dongsoo_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dongsoo_interfaces::srv::DongSooExecutor_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  dongsoo_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dongsoo_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const dongsoo_interfaces::srv::DongSooExecutor_Request & msg)
{
  return dongsoo_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<dongsoo_interfaces::srv::DongSooExecutor_Request>()
{
  return "dongsoo_interfaces::srv::DongSooExecutor_Request";
}

template<>
inline const char * name<dongsoo_interfaces::srv::DongSooExecutor_Request>()
{
  return "dongsoo_interfaces/srv/DongSooExecutor_Request";
}

template<>
struct has_fixed_size<dongsoo_interfaces::srv::DongSooExecutor_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<dongsoo_interfaces::srv::DongSooExecutor_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<dongsoo_interfaces::srv::DongSooExecutor_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace dongsoo_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const DongSooExecutor_Response & msg,
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
  const DongSooExecutor_Response & msg,
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

inline std::string to_yaml(const DongSooExecutor_Response & msg, bool use_flow_style = false)
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

}  // namespace dongsoo_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use dongsoo_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dongsoo_interfaces::srv::DongSooExecutor_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  dongsoo_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dongsoo_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const dongsoo_interfaces::srv::DongSooExecutor_Response & msg)
{
  return dongsoo_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<dongsoo_interfaces::srv::DongSooExecutor_Response>()
{
  return "dongsoo_interfaces::srv::DongSooExecutor_Response";
}

template<>
inline const char * name<dongsoo_interfaces::srv::DongSooExecutor_Response>()
{
  return "dongsoo_interfaces/srv/DongSooExecutor_Response";
}

template<>
struct has_fixed_size<dongsoo_interfaces::srv::DongSooExecutor_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<dongsoo_interfaces::srv::DongSooExecutor_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<dongsoo_interfaces::srv::DongSooExecutor_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dongsoo_interfaces::srv::DongSooExecutor>()
{
  return "dongsoo_interfaces::srv::DongSooExecutor";
}

template<>
inline const char * name<dongsoo_interfaces::srv::DongSooExecutor>()
{
  return "dongsoo_interfaces/srv/DongSooExecutor";
}

template<>
struct has_fixed_size<dongsoo_interfaces::srv::DongSooExecutor>
  : std::integral_constant<
    bool,
    has_fixed_size<dongsoo_interfaces::srv::DongSooExecutor_Request>::value &&
    has_fixed_size<dongsoo_interfaces::srv::DongSooExecutor_Response>::value
  >
{
};

template<>
struct has_bounded_size<dongsoo_interfaces::srv::DongSooExecutor>
  : std::integral_constant<
    bool,
    has_bounded_size<dongsoo_interfaces::srv::DongSooExecutor_Request>::value &&
    has_bounded_size<dongsoo_interfaces::srv::DongSooExecutor_Response>::value
  >
{
};

template<>
struct is_service<dongsoo_interfaces::srv::DongSooExecutor>
  : std::true_type
{
};

template<>
struct is_service_request<dongsoo_interfaces::srv::DongSooExecutor_Request>
  : std::true_type
{
};

template<>
struct is_service_response<dongsoo_interfaces::srv::DongSooExecutor_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // DONGSOO_INTERFACES__SRV__DETAIL__DONG_SOO_EXECUTOR__TRAITS_HPP_
