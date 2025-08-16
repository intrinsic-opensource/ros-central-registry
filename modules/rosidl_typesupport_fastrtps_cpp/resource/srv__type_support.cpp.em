@# Included from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
@{
from rosidl_generator_c import idl_structure_type_to_c_typename
from rosidl_generator_type_description import GET_DESCRIPTION_FUNC
from rosidl_generator_type_description import GET_HASH_FUNC
from rosidl_generator_type_description import GET_SOURCES_FUNC
from rosidl_parser.definition import SERVICE_EVENT_MESSAGE_SUFFIX
from rosidl_parser.definition import SERVICE_REQUEST_MESSAGE_SUFFIX
from rosidl_parser.definition import SERVICE_RESPONSE_MESSAGE_SUFFIX
from rosidl_pycommon import convert_camel_case_to_lower_case_underscore

include_parts = [package_name] + list(interface_path.parents[0].parts) + [
    'detail', convert_camel_case_to_lower_case_underscore(interface_path.stem)]
include_base = '/'.join(include_parts)
}@
@{
TEMPLATE(
    'msg__type_support.cpp.em',
    package_name=package_name, interface_path=interface_path, message=service.request_message,
    include_directives=include_directives,
    forward_declared_types=forward_declared_types)
}@

@{
TEMPLATE(
    'msg__type_support.cpp.em',
    package_name=package_name, interface_path=interface_path, message=service.response_message,
    include_directives=include_directives,
    forward_declared_types=forward_declared_types)
}@

@{
TEMPLATE(
    'msg__type_support.cpp.em',
    package_name=package_name, interface_path=interface_path, message=service.event_message,
    include_directives=include_directives,
    forward_declared_types=forward_declared_types)
}@

@{
header_files = [
    'rmw/error_handling.h',
    'rosidl_typesupport_cpp/service_type_support.hpp',
    'rosidl_typesupport_fastrtps_cpp/identifier.hpp',
    'rosidl_typesupport_fastrtps_cpp/service_type_support.h',
    'rosidl_typesupport_fastrtps_cpp/service_type_support_decl.hpp',
]
}@
@[for header_file in header_files]@
@[    if header_file in include_directives]@
// already included above
// @
@[    else]@
@{include_directives.add(header_file)}@
@[    end if]@
#include "@(header_file)"
@[end for]@
@[  for ns in service.namespaced_type.namespaces]@

namespace @(ns)
{
@[  end for]@

namespace typesupport_fastrtps_cpp
{

static service_type_support_callbacks_t _@(service.namespaced_type.name)__callbacks = {
  "@('::'.join([package_name] + list(interface_path.parents[0].parts)))",
  "@(service.namespaced_type.name)",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, @(', '.join([package_name] + list(interface_path.parents[0].parts))), @(service.namespaced_type.name + SERVICE_REQUEST_MESSAGE_SUFFIX))(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, @(', '.join([package_name] + list(interface_path.parents[0].parts))), @(service.namespaced_type.name + SERVICE_RESPONSE_MESSAGE_SUFFIX))(),
};

#ifdef __cplusplus
extern "C"
{
#endif

static const rosidl_service_type_support_t _@(service.namespaced_type.name)__handle{
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_@(service.namespaced_type.name)__callbacks,
  get_service_typesupport_handle_function,
  ::rosidl_typesupport_fastrtps_cpp::get_message_type_support_handle<@('::'.join([package_name, *interface_path.parents[0].parts, service.namespaced_type.name]))@(SERVICE_REQUEST_MESSAGE_SUFFIX)>(),
  ::rosidl_typesupport_fastrtps_cpp::get_message_type_support_handle<@('::'.join([package_name, *interface_path.parents[0].parts, service.namespaced_type.name]))@(SERVICE_RESPONSE_MESSAGE_SUFFIX)>(),
  ::rosidl_typesupport_fastrtps_cpp::get_message_type_support_handle<@('::'.join([package_name, *interface_path.parents[0].parts, service.namespaced_type.name]))@(SERVICE_EVENT_MESSAGE_SUFFIX)>(),
  &::rosidl_typesupport_cpp::service_create_event_message<@('::'.join([package_name, *interface_path.parents[0].parts, service.namespaced_type.name]))>,
  &::rosidl_typesupport_cpp::service_destroy_event_message<@('::'.join([package_name, *interface_path.parents[0].parts, service.namespaced_type.name]))>,
  &@(idl_structure_type_to_c_typename(service.namespaced_type))__@(GET_HASH_FUNC),
  &@(idl_structure_type_to_c_typename(service.namespaced_type))__@(GET_DESCRIPTION_FUNC),
  &@(idl_structure_type_to_c_typename(service.namespaced_type))__@(GET_SOURCES_FUNC),
};

#ifdef __cplusplus
}
#endif

}  // namespace typesupport_fastrtps_cpp
@[  for ns in reversed(service.namespaced_type.namespaces)]@

}  // namespace @(ns)
@[  end for]@

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_@(package_name)
const rosidl_service_type_support_t *
get_service_type_support_handle<@('::'.join([package_name] + list(interface_path.parents[0].parts) + [service.namespaced_type.name]))>()
{
  return &@('::'.join([package_name] + list(interface_path.parents[0].parts)))::typesupport_fastrtps_cpp::_@(service.namespaced_type.name)__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, @(', '.join([package_name] + list(interface_path.parents[0].parts))), @(service.namespaced_type.name))() {
  return &@('::'.join([package_name] + list(interface_path.parents[0].parts)))::typesupport_fastrtps_cpp::_@(service.namespaced_type.name)__handle;
}

#ifdef __cplusplus
}
#endif
