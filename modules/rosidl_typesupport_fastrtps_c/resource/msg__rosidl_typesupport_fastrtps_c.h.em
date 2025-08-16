@# Included from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
@{

from rosidl_pycommon import convert_camel_case_to_lower_case_underscore

include_parts = [package_name] + list(interface_path.parents[0].parts) + [
    'detail', convert_camel_case_to_lower_case_underscore(interface_path.stem)]
include_base = '/'.join(include_parts)

header_files = [
    'stddef.h',
    'rosidl_runtime_c/message_type_support_struct.h',
    'rosidl_typesupport_interface/macros.h',
    package_name + '/msg/rosidl_typesupport_fastrtps_c__visibility_control.h',
    include_base + '__struct.h',
    'fastcdr/Cdr.h',
]
}@
@[for header_file in header_files]@
@[    if header_file in include_directives]@
// already included above
// @
@[    else]@
@{include_directives.add(header_file)}@
@[    end if]@
@[    if '/' not in header_file]@
#include <@(header_file)>
@[    else]@
#include "@(header_file)"
@[    end if]@
@[end for]@

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_@(package_name)
bool cdr_serialize_@('__'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name]))(
  const @('__'.join(message.structure.namespaced_type.namespaced_name())) * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_@(package_name)
bool cdr_deserialize_@('__'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name]))(
  eprosima::fastcdr::Cdr &,
  @('__'.join(message.structure.namespaced_type.namespaced_name())) * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_@(package_name)
size_t get_serialized_size_@('__'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name]))(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_@(package_name)
size_t max_serialized_size_@('__'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name]))(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_@(package_name)
bool cdr_serialize_key_@('__'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name]))(
  const @('__'.join(message.structure.namespaced_type.namespaced_name())) * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_@(package_name)
size_t get_serialized_size_key_@('__'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name]))(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_@(package_name)
size_t max_serialized_size_key_@('__'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name]))(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_@(package_name)
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, @(', '.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name])))();

#ifdef __cplusplus
}
#endif
