// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from zeus_interfaces:srv/ZeusExecutor.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "zeus_interfaces/srv/detail/zeus_executor__rosidl_typesupport_introspection_c.h"
#include "zeus_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "zeus_interfaces/srv/detail/zeus_executor__functions.h"
#include "zeus_interfaces/srv/detail/zeus_executor__struct.h"


// Include directives for member types
// Member `frame`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void zeus_interfaces__srv__ZeusExecutor_Request__rosidl_typesupport_introspection_c__ZeusExecutor_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  zeus_interfaces__srv__ZeusExecutor_Request__init(message_memory);
}

void zeus_interfaces__srv__ZeusExecutor_Request__rosidl_typesupport_introspection_c__ZeusExecutor_Request_fini_function(void * message_memory)
{
  zeus_interfaces__srv__ZeusExecutor_Request__fini(message_memory);
}

size_t zeus_interfaces__srv__ZeusExecutor_Request__rosidl_typesupport_introspection_c__size_function__ZeusExecutor_Request__coordinate(
  const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * zeus_interfaces__srv__ZeusExecutor_Request__rosidl_typesupport_introspection_c__get_const_function__ZeusExecutor_Request__coordinate(
  const void * untyped_member, size_t index)
{
  const float * member =
    (const float *)(untyped_member);
  return &member[index];
}

void * zeus_interfaces__srv__ZeusExecutor_Request__rosidl_typesupport_introspection_c__get_function__ZeusExecutor_Request__coordinate(
  void * untyped_member, size_t index)
{
  float * member =
    (float *)(untyped_member);
  return &member[index];
}

void zeus_interfaces__srv__ZeusExecutor_Request__rosidl_typesupport_introspection_c__fetch_function__ZeusExecutor_Request__coordinate(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    zeus_interfaces__srv__ZeusExecutor_Request__rosidl_typesupport_introspection_c__get_const_function__ZeusExecutor_Request__coordinate(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void zeus_interfaces__srv__ZeusExecutor_Request__rosidl_typesupport_introspection_c__assign_function__ZeusExecutor_Request__coordinate(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    zeus_interfaces__srv__ZeusExecutor_Request__rosidl_typesupport_introspection_c__get_function__ZeusExecutor_Request__coordinate(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember zeus_interfaces__srv__ZeusExecutor_Request__rosidl_typesupport_introspection_c__ZeusExecutor_Request_message_member_array[2] = {
  {
    "frame",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zeus_interfaces__srv__ZeusExecutor_Request, frame),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "coordinate",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(zeus_interfaces__srv__ZeusExecutor_Request, coordinate),  // bytes offset in struct
    NULL,  // default value
    zeus_interfaces__srv__ZeusExecutor_Request__rosidl_typesupport_introspection_c__size_function__ZeusExecutor_Request__coordinate,  // size() function pointer
    zeus_interfaces__srv__ZeusExecutor_Request__rosidl_typesupport_introspection_c__get_const_function__ZeusExecutor_Request__coordinate,  // get_const(index) function pointer
    zeus_interfaces__srv__ZeusExecutor_Request__rosidl_typesupport_introspection_c__get_function__ZeusExecutor_Request__coordinate,  // get(index) function pointer
    zeus_interfaces__srv__ZeusExecutor_Request__rosidl_typesupport_introspection_c__fetch_function__ZeusExecutor_Request__coordinate,  // fetch(index, &value) function pointer
    zeus_interfaces__srv__ZeusExecutor_Request__rosidl_typesupport_introspection_c__assign_function__ZeusExecutor_Request__coordinate,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers zeus_interfaces__srv__ZeusExecutor_Request__rosidl_typesupport_introspection_c__ZeusExecutor_Request_message_members = {
  "zeus_interfaces__srv",  // message namespace
  "ZeusExecutor_Request",  // message name
  2,  // number of fields
  sizeof(zeus_interfaces__srv__ZeusExecutor_Request),
  zeus_interfaces__srv__ZeusExecutor_Request__rosidl_typesupport_introspection_c__ZeusExecutor_Request_message_member_array,  // message members
  zeus_interfaces__srv__ZeusExecutor_Request__rosidl_typesupport_introspection_c__ZeusExecutor_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  zeus_interfaces__srv__ZeusExecutor_Request__rosidl_typesupport_introspection_c__ZeusExecutor_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t zeus_interfaces__srv__ZeusExecutor_Request__rosidl_typesupport_introspection_c__ZeusExecutor_Request_message_type_support_handle = {
  0,
  &zeus_interfaces__srv__ZeusExecutor_Request__rosidl_typesupport_introspection_c__ZeusExecutor_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_zeus_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zeus_interfaces, srv, ZeusExecutor_Request)() {
  if (!zeus_interfaces__srv__ZeusExecutor_Request__rosidl_typesupport_introspection_c__ZeusExecutor_Request_message_type_support_handle.typesupport_identifier) {
    zeus_interfaces__srv__ZeusExecutor_Request__rosidl_typesupport_introspection_c__ZeusExecutor_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &zeus_interfaces__srv__ZeusExecutor_Request__rosidl_typesupport_introspection_c__ZeusExecutor_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "zeus_interfaces/srv/detail/zeus_executor__rosidl_typesupport_introspection_c.h"
// already included above
// #include "zeus_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "zeus_interfaces/srv/detail/zeus_executor__functions.h"
// already included above
// #include "zeus_interfaces/srv/detail/zeus_executor__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void zeus_interfaces__srv__ZeusExecutor_Response__rosidl_typesupport_introspection_c__ZeusExecutor_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  zeus_interfaces__srv__ZeusExecutor_Response__init(message_memory);
}

void zeus_interfaces__srv__ZeusExecutor_Response__rosidl_typesupport_introspection_c__ZeusExecutor_Response_fini_function(void * message_memory)
{
  zeus_interfaces__srv__ZeusExecutor_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember zeus_interfaces__srv__ZeusExecutor_Response__rosidl_typesupport_introspection_c__ZeusExecutor_Response_message_member_array[1] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(zeus_interfaces__srv__ZeusExecutor_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers zeus_interfaces__srv__ZeusExecutor_Response__rosidl_typesupport_introspection_c__ZeusExecutor_Response_message_members = {
  "zeus_interfaces__srv",  // message namespace
  "ZeusExecutor_Response",  // message name
  1,  // number of fields
  sizeof(zeus_interfaces__srv__ZeusExecutor_Response),
  zeus_interfaces__srv__ZeusExecutor_Response__rosidl_typesupport_introspection_c__ZeusExecutor_Response_message_member_array,  // message members
  zeus_interfaces__srv__ZeusExecutor_Response__rosidl_typesupport_introspection_c__ZeusExecutor_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  zeus_interfaces__srv__ZeusExecutor_Response__rosidl_typesupport_introspection_c__ZeusExecutor_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t zeus_interfaces__srv__ZeusExecutor_Response__rosidl_typesupport_introspection_c__ZeusExecutor_Response_message_type_support_handle = {
  0,
  &zeus_interfaces__srv__ZeusExecutor_Response__rosidl_typesupport_introspection_c__ZeusExecutor_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_zeus_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zeus_interfaces, srv, ZeusExecutor_Response)() {
  if (!zeus_interfaces__srv__ZeusExecutor_Response__rosidl_typesupport_introspection_c__ZeusExecutor_Response_message_type_support_handle.typesupport_identifier) {
    zeus_interfaces__srv__ZeusExecutor_Response__rosidl_typesupport_introspection_c__ZeusExecutor_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &zeus_interfaces__srv__ZeusExecutor_Response__rosidl_typesupport_introspection_c__ZeusExecutor_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "zeus_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "zeus_interfaces/srv/detail/zeus_executor__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers zeus_interfaces__srv__detail__zeus_executor__rosidl_typesupport_introspection_c__ZeusExecutor_service_members = {
  "zeus_interfaces__srv",  // service namespace
  "ZeusExecutor",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // zeus_interfaces__srv__detail__zeus_executor__rosidl_typesupport_introspection_c__ZeusExecutor_Request_message_type_support_handle,
  NULL  // response message
  // zeus_interfaces__srv__detail__zeus_executor__rosidl_typesupport_introspection_c__ZeusExecutor_Response_message_type_support_handle
};

static rosidl_service_type_support_t zeus_interfaces__srv__detail__zeus_executor__rosidl_typesupport_introspection_c__ZeusExecutor_service_type_support_handle = {
  0,
  &zeus_interfaces__srv__detail__zeus_executor__rosidl_typesupport_introspection_c__ZeusExecutor_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zeus_interfaces, srv, ZeusExecutor_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zeus_interfaces, srv, ZeusExecutor_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_zeus_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zeus_interfaces, srv, ZeusExecutor)() {
  if (!zeus_interfaces__srv__detail__zeus_executor__rosidl_typesupport_introspection_c__ZeusExecutor_service_type_support_handle.typesupport_identifier) {
    zeus_interfaces__srv__detail__zeus_executor__rosidl_typesupport_introspection_c__ZeusExecutor_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)zeus_interfaces__srv__detail__zeus_executor__rosidl_typesupport_introspection_c__ZeusExecutor_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zeus_interfaces, srv, ZeusExecutor_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, zeus_interfaces, srv, ZeusExecutor_Response)()->data;
  }

  return &zeus_interfaces__srv__detail__zeus_executor__rosidl_typesupport_introspection_c__ZeusExecutor_service_type_support_handle;
}
