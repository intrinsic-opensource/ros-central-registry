@# Included from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
@{
from rosidl_generator_c import idl_structure_type_to_c_typename
from rosidl_generator_type_description import GET_DESCRIPTION_FUNC
from rosidl_generator_type_description import GET_HASH_FUNC
from rosidl_generator_type_description import GET_SOURCES_FUNC
from rosidl_parser.definition import AbstractGenericString
from rosidl_parser.definition import AbstractNestedType
from rosidl_parser.definition import AbstractSequence
from rosidl_parser.definition import AbstractString
from rosidl_parser.definition import AbstractWString
from rosidl_parser.definition import ACTION_FEEDBACK_SUFFIX
from rosidl_parser.definition import ACTION_GOAL_SUFFIX
from rosidl_parser.definition import ACTION_RESULT_SUFFIX
from rosidl_parser.definition import SERVICE_EVENT_MESSAGE_SUFFIX
from rosidl_parser.definition import SERVICE_REQUEST_MESSAGE_SUFFIX
from rosidl_parser.definition import SERVICE_RESPONSE_MESSAGE_SUFFIX
from rosidl_parser.definition import Array
from rosidl_parser.definition import BasicType
from rosidl_parser.definition import BoundedSequence
from rosidl_parser.definition import NamespacedType
from rosidl_pycommon import convert_camel_case_to_lower_case_underscore

include_parts = [package_name] + list(interface_path.parents[0].parts) + [
    'detail', convert_camel_case_to_lower_case_underscore(interface_path.stem)]
include_base = '/'.join(include_parts)


header_files = [
    'cassert',
    'cstddef',
    'limits',
    'string',
    # Provides the rosidl_typesupport_fastrtps_c__identifier symbol declaration.
    'rosidl_typesupport_fastrtps_c/identifier.h',
    'rosidl_typesupport_fastrtps_c/serialization_helpers.hpp',
    'rosidl_typesupport_fastrtps_c/wstring_conversion.hpp',
    # Provides the definition of the message_type_support_callbacks_t struct.
    'rosidl_typesupport_fastrtps_cpp/message_type_support.h',
    package_name + '/msg/rosidl_typesupport_fastrtps_c__visibility_control.h',
    include_base + '__struct.h',
    include_base + '__functions.h',
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

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

@# // Include the message header for each non-primitive field.
#if defined(__cplusplus)
extern "C"
{
#endif

@{
includes = {}
for member in message.structure.members:
    keys = set([])
    if isinstance(member.type, AbstractSequence) and isinstance(member.type.value_type, BasicType):
        keys.add('rosidl_runtime_c/primitives_sequence.h')
        keys.add('rosidl_runtime_c/primitives_sequence_functions.h')
    type_ = member.type
    if isinstance(type_, AbstractNestedType):
        type_ = type_.value_type
    if isinstance(type_, AbstractString):
        keys.add('rosidl_runtime_c/string.h')
        keys.add('rosidl_runtime_c/string_functions.h')
    elif isinstance(type_, AbstractWString):
        keys.add('rosidl_runtime_c/u16string.h')
        keys.add('rosidl_runtime_c/u16string_functions.h')
    elif isinstance(type_, NamespacedType):
        import sys
        if (
            type_.name.endswith(SERVICE_REQUEST_MESSAGE_SUFFIX) or
            type_.name.endswith(SERVICE_RESPONSE_MESSAGE_SUFFIX)
        ):
            continue
        if (
            type_.name.endswith(ACTION_GOAL_SUFFIX) or
            type_.name.endswith(ACTION_RESULT_SUFFIX) or
            type_.name.endswith(ACTION_FEEDBACK_SUFFIX)
        ):
            typename = type_.name.rsplit('_', 1)[0]
        else:
            typename = type_.name
        keys.add('/'.join(type_.namespaces + ['detail', convert_camel_case_to_lower_case_underscore(typename)]) + '__functions.h')
    for key in keys:
        if key not in includes:
            includes[key] = set([])
        includes[key].add(member.name)
}@
@[for header_file in sorted(includes.keys())]@
@[    if header_file in include_directives]@
// already included above
// @
@[    else]@
@{include_directives.add(header_file)}@
@[    end if]@
#include "@(header_file)"  // @(', '.join(sorted(includes[header_file])))
@[end for]@

// forward declare type support functions
@{
forward_declares = {}
for member in message.structure.members:
    type_ = member.type
    if isinstance(type_, AbstractNestedType):
        type_ = type_.value_type
    if isinstance(type_, NamespacedType):
        key = (*type_.namespaces, type_.name)
        if key not in includes:
            forward_declares[key] = set([])
        forward_declares[key].add(member.name)
}@
@[for key in sorted(forward_declares.keys())]@

@[  if key[0] != package_name]@
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_@(package_name)
@[  end if]@
bool cdr_serialize_@('__'.join(key))(
  const @('__'.join(key)) * ros_message,
  eprosima::fastcdr::Cdr & cdr);

@[  if key[0] != package_name]@
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_@(package_name)
@[  end if]@
bool cdr_deserialize_@('__'.join(key))(
  eprosima::fastcdr::Cdr & cdr,
  @('__'.join(key)) * ros_message);

@[  if key[0] != package_name]@
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_@(package_name)
@[  end if]@
size_t get_serialized_size_@('__'.join(key))(
  const void * untyped_ros_message,
  size_t current_alignment);

@[  if key[0] != package_name]@
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_@(package_name)
@[  end if]@
size_t max_serialized_size_@('__'.join(key))(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

@[  if key[0] != package_name]@
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_@(package_name)
@[  end if]@
bool cdr_serialize_key_@('__'.join(key))(
  const @('__'.join(key)) * ros_message,
  eprosima::fastcdr::Cdr & cdr);

@[  if key[0] != package_name]@
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_@(package_name)
@[  end if]@
size_t get_serialized_size_key_@('__'.join(key))(
  const void * untyped_ros_message,
  size_t current_alignment);

@[  if key[0] != package_name]@
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_@(package_name)
@[  end if]@
size_t max_serialized_size_key_@('__'.join(key))(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

@[  if key[0] != package_name]@
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_@(package_name)
@[  end if]@
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, @(', '.join(key)))();
@[end for]@

@# // Make callback functions specific to this message type.

using _@(message.structure.namespaced_type.name)__ros_msg_type = @('__'.join(message.structure.namespaced_type.namespaced_name()));

@{

# Generates the definition for the serialization family of methods given a structure member
#   member: the member to serialize
#   suffix: the suffix name of the method. Will be used in case of recursion

def generate_member_for_cdr_serialize(member, suffix):
  from rosidl_generator_cpp import msg_type_only_to_cpp
  from rosidl_generator_cpp import msg_type_to_cpp
  from rosidl_parser.definition import AbstractGenericString
  from rosidl_parser.definition import AbstractNestedType
  from rosidl_parser.definition import AbstractSequence
  from rosidl_parser.definition import AbstractString
  from rosidl_parser.definition import AbstractWString
  from rosidl_parser.definition import Array
  from rosidl_parser.definition import BasicType
  from rosidl_parser.definition import BoundedSequence
  from rosidl_parser.definition import NamespacedType
  strlist = []
  strlist.append('// Field name: %s' % (member.name))
  strlist.append('{')

  type_ = member.type
  if isinstance(type_, AbstractNestedType):
    type_ = type_.value_type

  if isinstance(member.type, AbstractNestedType):
    if isinstance(member.type, Array):
      strlist.append('  size_t size = %d;' % (member.type.size))
      strlist.append('  auto array_ptr = ros_message->%s;' % (member.name))
    else:
      strlist.append('  size_t size = ros_message->%s.size;' % (member.name))
      strlist.append('  auto array_ptr = ros_message->%s.data;' % (member.name))
      if isinstance(member.type, BoundedSequence):
        strlist.append('  if (size > %d) {' % (member.type.maximum_size))
        strlist.append('    fprintf(stderr, \"array size exceeds upper bound\\n\");')
        strlist.append('    return false;')
        strlist.append('  }')
      strlist.append('  cdr << static_cast<uint32_t>(size);')
    if isinstance(member.type.value_type, AbstractString):
      strlist.append('  for (size_t i = 0; i < size; ++i) {')
      strlist.append('    const rosidl_runtime_c__String * str = &array_ptr[i];')
      strlist.append('    if (str->capacity == 0 || str->capacity <= str->size) {')
      strlist.append('      fprintf(stderr, \"string capacity not greater than size\\n\");')
      strlist.append('      return false;')
      strlist.append('    }')
      strlist.append('    if (str->data[str->size] != \'\\0\') {')
      strlist.append('      fprintf(stderr, \"string not null-terminated\\n\");')
      strlist.append('      return false;')
      strlist.append('    }')
      strlist.append('    cdr << str->data;')
      strlist.append('  }')
    elif isinstance(member.type.value_type, AbstractWString):
      strlist.append('  for (size_t i = 0; i < size; ++i) {')
      strlist.append('    const rosidl_runtime_c__U16String * str = &array_ptr[i];')
      strlist.append('    if (str->capacity == 0 || str->capacity <= str->size) {')
      strlist.append('      fprintf(stderr, \"string capacity not greater than size\\n\");')
      strlist.append('      return false;')
      strlist.append('    }')
      strlist.append('    if (str->data[str->size] != \'\\0\') {')
      strlist.append('      fprintf(stderr, \"string not null-terminated\\n\");')
      strlist.append('      return false;')
      strlist.append('    }')
      strlist.append('    rosidl_typesupport_fastrtps_c::cdr_serialize(cdr, *str);')
      strlist.append('  }')
    elif isinstance(member.type.value_type, BasicType) and member.type.value_type.typename == 'wchar':
      strlist.append('  for (size_t i = 0; i < size; ++i) {')
      strlist.append('    cdr_serialize%s_%s(' % (suffix, ('__'.join(member.type.value_type.namespaced_name()))))
      strlist.append('      static_cast<wchar_t *>(&array_ptr[i]), cdr);')
      strlist.append('  }')
    elif isinstance(member.type.value_type, BasicType):
      strlist.append('  cdr.serialize_array(array_ptr, size);')
    else :
      strlist.append('  for (size_t i = 0; i < size; ++i) {')
      strlist.append('    cdr_serialize%s_%s(' % (suffix, ('__'.join(member.type.value_type.namespaced_name()))))
      strlist.append('      &array_ptr[i], cdr);')
      strlist.append('  }')
  elif isinstance(member.type, AbstractString):
    strlist.append('  const rosidl_runtime_c__String * str = &ros_message->%s;' % (member.name))
    strlist.append('  if (str->capacity == 0 || str->capacity <= str->size) {')
    strlist.append('    fprintf(stderr, \"string capacity not greater than size\\n\");')
    strlist.append('    return false;')
    strlist.append('  }')
    strlist.append('  if (str->data[str->size] != \'\\0\') {')
    strlist.append('    fprintf(stderr, \"string not null-terminated\\n\");')
    strlist.append('    return false;')
    strlist.append('  }')
    strlist.append('  cdr << str->data;')
  elif isinstance(member.type, AbstractWString):
    strlist.append('  rosidl_typesupport_fastrtps_c::cdr_serialize(cdr, ros_message->%s);' % (member.name))
  elif isinstance(member.type, BasicType) and member.type.typename == 'boolean':
    strlist.append('  cdr << (ros_message->%s ? true : false);' % (member.name))
  elif isinstance(member.type, BasicType) and member.type.typename == 'wchar':
    strlist.append('  cdr << static_cast<wchar_t>(ros_message->%s);' % (member.name))
  elif isinstance(member.type, BasicType):
    strlist.append('  cdr << ros_message->%s;' % (member.name))
  else:
    strlist.append('  cdr_serialize%s_%s(' % (suffix, ('__'.join(member.type.namespaced_name()))))
    strlist.append('    &ros_message->%s, cdr);' % (member.name))
  strlist.append('}')

  return strlist
}@

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_@(package_name)
bool cdr_serialize_@('__'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name]))(
  const @('__'.join(message.structure.namespaced_type.namespaced_name())) * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
@[for member in message.structure.members]@
@[  for line in generate_member_for_cdr_serialize(member, '')]@
  @(line)
@[  end for]@

@[end for]@
  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_@(package_name)
bool cdr_deserialize_@('__'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name]))(
  eprosima::fastcdr::Cdr & cdr,
  @('__'.join(message.structure.namespaced_type.namespaced_name())) * ros_message)
{
@[for member in message.structure.members]@
  // Field name: @(member.name)
  {
@{
type_ = member.type
if isinstance(type_, AbstractNestedType):
    type_ = type_.value_type
}@
@[  if isinstance(member.type, AbstractNestedType)]@
@[    if isinstance(member.type, Array)]@
    size_t size = @(member.type.size);
    auto array_ptr = ros_message->@(member.name);
@[    else]@
@{
if isinstance(member.type.value_type, AbstractString):
    array_init = 'rosidl_runtime_c__String__Sequence__init'
    array_fini = 'rosidl_runtime_c__String__Sequence__fini'
elif isinstance(member.type.value_type, AbstractWString):
    array_init = 'rosidl_runtime_c__U16String__Sequence__init'
    array_fini = 'rosidl_runtime_c__U16String__Sequence__fini'
elif isinstance(member.type.value_type, BasicType):
    type_ = member.type.value_type.typename
    type_ = type_.replace(' ', '_')
    array_init = 'rosidl_runtime_c__{type_}__Sequence__init'.format(**locals())
    array_fini = 'rosidl_runtime_c__{type_}__Sequence__fini'.format(**locals())
else:
    array_init = '__'.join(type_.namespaced_name()) + '__Sequence__init'
    array_fini = '__'.join(type_.namespaced_name()) + '__Sequence__fini'
}@
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->@(member.name).data) {
      @(array_fini)(&ros_message->@(member.name));
    }
    if (!@(array_init)(&ros_message->@(member.name), size)) {
      fprintf(stderr, "failed to create array for field '@(member.name)'");
      return false;
    }
    auto array_ptr = ros_message->@(member.name).data;
@[    end if]@
@[    if isinstance(member.type.value_type, AbstractString)]@
    for (size_t i = 0; i < size; ++i) {
      std::string tmp;
      cdr >> tmp;
      auto & ros_i = array_ptr[i];
      if (!ros_i.data) {
        rosidl_runtime_c__String__init(&ros_i);
      }
      bool succeeded = rosidl_runtime_c__String__assign(
        &ros_i,
        tmp.c_str());
      if (!succeeded) {
        fprintf(stderr, "failed to assign string into field '@(member.name)'\n");
        return false;
      }
    }
@[    elif isinstance(member.type.value_type, AbstractWString)]@
    std::wstring wstr;
    for (size_t i = 0; i < size; ++i) {
      auto & ros_i = array_ptr[i];
      if (!ros_i.data) {
        rosidl_runtime_c__U16String__init(&ros_i);
      }
      bool succeeded = rosidl_typesupport_fastrtps_c::cdr_deserialize(cdr, ros_i);
      if (!succeeded) {
        fprintf(stderr, "failed to create wstring from u16string\n");
        rosidl_runtime_c__U16String__fini(&ros_i);
        return false;
      }
    }
@[    elif isinstance(member.type.value_type, BasicType) and member.type.value_type.typename == 'boolean']@
    for (size_t i = 0; i < size; ++i) {
      uint8_t tmp;
      cdr >> tmp;
      array_ptr[i] = tmp ? true : false;
    }
@[    elif isinstance(member.type.value_type, BasicType) and member.type.value_type.typename == 'wchar']@
    for (size_t i = 0; i < size; ++i) {
      wchar_t tmp;
      cdr >> tmp;
      array_ptr[i] = static_cast<char16_t>(tmp);
    }
@[    elif isinstance(member.type.value_type, BasicType)]@
    cdr.deserialize_array(array_ptr, size);
@[    else]@
    for (size_t i = 0; i < size; ++i) {
      cdr_deserialize_@('__'.join(member.type.value_type.namespaced_name()))(cdr, &array_ptr[i]);
    }
@[    end if]@
@[   elif isinstance(member.type, AbstractString)]@
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->@(member.name).data) {
      rosidl_runtime_c__String__init(&ros_message->@(member.name));
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->@(member.name),
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field '@(member.name)'\n");
      return false;
    }
@[   elif isinstance(member.type, AbstractWString)]@
    if (!ros_message->@(member.name).data) {
      rosidl_runtime_c__U16String__init(&ros_message->@(member.name));
    }
    bool succeeded = rosidl_typesupport_fastrtps_c::cdr_deserialize(cdr, ros_message->@(member.name));
    if (!succeeded) {
      fprintf(stderr, "failed to create wstring from u16string\n");
      rosidl_runtime_c__U16String__fini(&ros_message->@(member.name));
      return false;
    }
@[ elif isinstance(member.type, BasicType) and member.type.typename == 'boolean']@
    uint8_t tmp;
    cdr >> tmp;
    ros_message->@(member.name) = tmp ? true : false;
@[ elif isinstance(member.type, BasicType) and member.type.typename == 'wchar']@
    wchar_t tmp;
    cdr >> tmp;
    ros_message->@(member.name) = static_cast<char16_t>(tmp);
@[  elif isinstance(member.type, BasicType)]@
    cdr >> ros_message->@(member.name);
@[  else]@
    cdr_deserialize_@('__'.join(member.type.namespaced_name()))(cdr, &ros_message->@(member.name));
@[  end if]@
  }

@[end for]@
  return true;
}  // NOLINT(readability/fn_size)

@{

# Generates the definition for the get_serialized_size family of methods given a structure member
#   member: the member to serialize
#   suffix: the suffix name of the method. Will be used in case of recursion

def generate_member_for_get_serialized_size(member, suffix):
  from rosidl_generator_cpp import msg_type_only_to_cpp
  from rosidl_generator_cpp import msg_type_to_cpp
  from rosidl_parser.definition import AbstractGenericString
  from rosidl_parser.definition import AbstractNestedType
  from rosidl_parser.definition import AbstractSequence
  from rosidl_parser.definition import AbstractString
  from rosidl_parser.definition import AbstractWString
  from rosidl_parser.definition import Array
  from rosidl_parser.definition import BasicType
  from rosidl_parser.definition import BoundedSequence
  from rosidl_parser.definition import NamespacedType
  strlist = []
  strlist.append('// Field name: %s' % (member.name))
  if isinstance(member.type, AbstractNestedType):
    strlist.append('{')
    if isinstance(member.type, Array):
      strlist.append('  size_t array_size = %d;' % (member.type.size))
      strlist.append('  auto array_ptr = ros_message->%s;' % (member.name))
    else:
      strlist.append('  size_t array_size = ros_message->%s.size;' % (member.name))
      strlist.append('  auto array_ptr = ros_message->%s.data;' % (member.name))
      strlist.append('  current_alignment += padding +')
      strlist.append('    eprosima::fastcdr::Cdr::alignment(current_alignment, padding);')
    if isinstance(member.type.value_type, AbstractGenericString):
      strlist.append('  for (size_t index = 0; index < array_size; ++index) {')
      strlist.append('    current_alignment += padding +')
      strlist.append('      eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +')
      if isinstance(member.type.value_type, AbstractWString):
        strlist.append('      wchar_size *')
      strlist.append('      (array_ptr[index].size + 1);')
      strlist.append('  }')
    elif isinstance(member.type.value_type, BasicType):
      strlist.append('  (void)array_ptr;')
      strlist.append('  size_t item_size = sizeof(array_ptr[0]);')
      strlist.append('  current_alignment += array_size * item_size +')
      strlist.append('    eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);')
    else:
      strlist.append('  for (size_t index = 0; index < array_size; ++index) {')
      strlist.append('    current_alignment += get_serialized_size%s_%s(' % (suffix, ('__'.join(member.type.value_type.namespaced_name()))))
      strlist.append('      &array_ptr[index], current_alignment);')
      strlist.append('  }')
    strlist.append('}')
  else:
    if isinstance(member.type, AbstractGenericString):
      strlist.append('current_alignment += padding +')
      strlist.append('  eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +')
      if isinstance(member.type, AbstractWString):
        strlist.append('  wchar_size *')
      strlist.append('  (ros_message->%s.size + 1);' % (member.name))
    elif isinstance(member.type, BasicType):
      strlist.append('{')
      strlist.append('  size_t item_size = sizeof(ros_message->%s);' % (member.name))
      strlist.append('  current_alignment += item_size +')
      strlist.append('    eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);')
      strlist.append('}')
    else:
      strlist.append('current_alignment += get_serialized_size%s_%s(' % (suffix, ('__'.join(member.type.namespaced_name()))))
      strlist.append('  &(ros_message->%s), current_alignment);' % (member.name))
  return strlist
}@

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_@(package_name)
size_t get_serialized_size_@('__'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name]))(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _@(message.structure.namespaced_type.name)__ros_msg_type * ros_message = static_cast<const _@(message.structure.namespaced_type.name)__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

@[for member in message.structure.members]@
@[  for line in generate_member_for_get_serialized_size(member, '')]@
  @(line)
@[  end for]@

@[end for]@
  return current_alignment - initial_alignment;
}

@{

# Generates the definition for the max_serialized_size family of methods given a structure member
#   member: the member to serialize
#   suffix: the suffix name of the method. Will be used in case of recursion

def generate_member_for_max_serialized_size(member, suffix):
  from rosidl_generator_cpp import msg_type_only_to_cpp
  from rosidl_generator_cpp import msg_type_to_cpp
  from rosidl_parser.definition import AbstractGenericString
  from rosidl_parser.definition import AbstractNestedType
  from rosidl_parser.definition import AbstractSequence
  from rosidl_parser.definition import AbstractString
  from rosidl_parser.definition import AbstractWString
  from rosidl_parser.definition import Array
  from rosidl_parser.definition import BasicType
  from rosidl_parser.definition import BoundedSequence
  from rosidl_parser.definition import NamespacedType
  strlist = []
  strlist.append('// Field name: %s' % (member.name))
  strlist.append('{')

  if isinstance(member.type, AbstractNestedType):
    if isinstance(member.type, Array):
      strlist.append('  size_t array_size = %d;' % (member.type.size))
    elif isinstance(member.type, BoundedSequence):
      strlist.append('  size_t array_size = %d;' % (member.type.maximum_size))
    else:
      strlist.append('  size_t array_size = 0;')
      strlist.append('  full_bounded = false;')
    if isinstance(member.type, AbstractSequence):
      strlist.append('  is_plain = false;')
      strlist.append('  current_alignment += padding +')
      strlist.append('    eprosima::fastcdr::Cdr::alignment(current_alignment, padding);')
  else:
    strlist.append('  size_t array_size = 1;')

  type_ = member.type
  if isinstance(type_, AbstractNestedType):
    type_ = type_.value_type

  if isinstance(type_, AbstractGenericString):
    strlist.append('  full_bounded = false;')
    strlist.append('  is_plain = false;')
    strlist.append('  for (size_t index = 0; index < array_size; ++index) {')
    strlist.append('    current_alignment += padding +')
    strlist.append('      eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +')
    if type_.has_maximum_size():
      if isinstance(type_, AbstractWString):
        strlist.append('      wchar_size *')
      strlist.append('      %d +' % (type_.maximum_size))
    if isinstance(type_, AbstractWString):
      strlist.append('      wchar_size *')
    strlist.append('      1;')
    strlist.append('  }')
  elif isinstance(type_, BasicType):
    if type_.typename in ('boolean', 'octet', 'char', 'uint8', 'int8'):
      strlist.append('  last_member_size = array_size * sizeof(uint8_t);')
      strlist.append('  current_alignment += array_size * sizeof(uint8_t);')
    elif type_.typename in ('wchar', 'int16', 'uint16'):
      strlist.append('  last_member_size = array_size * sizeof(uint16_t);')
      strlist.append('  current_alignment += array_size * sizeof(uint16_t) +')
      strlist.append('    eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));')
    elif type_.typename in ('int32', 'uint32', 'float'):
      strlist.append('  last_member_size = array_size * sizeof(uint32_t);')
      strlist.append('  current_alignment += array_size * sizeof(uint32_t) +')
      strlist.append('    eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));')
    elif type_.typename in ('int64', 'uint64', 'double'):
      strlist.append('  last_member_size = array_size * sizeof(uint64_t);')
      strlist.append('  current_alignment += array_size * sizeof(uint64_t) +')
      strlist.append('    eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));')
    elif type_.typename == 'long double':
      strlist.append('  last_member_size = array_size * sizeof(long double);')
      strlist.append('  current_alignment += array_size * sizeof(long double) +')
      strlist.append('    eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(long double));')
  else:
    strlist.append('  last_member_size = 0;')
    strlist.append('  for (size_t index = 0; index < array_size; ++index) {')
    strlist.append('    bool inner_full_bounded;')
    strlist.append('    bool inner_is_plain;')
    strlist.append('    size_t inner_size;')
    strlist.append('    inner_size =')
    strlist.append('      max_serialized_size%s_%s(' % (suffix, ('__'.join(type_.namespaced_name()))))
    strlist.append('      inner_full_bounded, inner_is_plain, current_alignment);')
    strlist.append('    last_member_size += inner_size;')
    strlist.append('    current_alignment += inner_size;')
    strlist.append('    full_bounded &= inner_full_bounded;')
    strlist.append('    is_plain &= inner_is_plain;')
    strlist.append('  }')
  strlist.append('}')
  return strlist
}@

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_@(package_name)
size_t max_serialized_size_@('__'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name]))(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

@{
last_member_name_ = None
}@
@[for member in message.structure.members]@
@{
last_member_name_ = member.name
}@
@[  for line in generate_member_for_max_serialized_size(member, '')]@
  @(line)
@[  end for]@

@[end for]@

  size_t ret_val = current_alignment - initial_alignment;
@[if last_member_name_ is not None]@
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = @('__'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name]));
    is_plain =
      (
      offsetof(DataType, @(last_member_name_)) +
      last_member_size
      ) == ret_val;
  }
@[end if]@
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_@(package_name)
bool cdr_serialize_key_@('__'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name]))(
  const @('__'.join(message.structure.namespaced_type.namespaced_name())) * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
@[for member in message.structure.members]@
@[  if not member.has_annotation('key') and message.structure.has_any_member_with_annotation('key')]@
@[  continue]@
@[  end if]@
@[  for line in generate_member_for_cdr_serialize(member, '_key')]@
  @(line)
@[  end for]@

@[end for]@
  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_@(package_name)
size_t get_serialized_size_key_@('__'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name]))(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _@(message.structure.namespaced_type.name)__ros_msg_type * ros_message = static_cast<const _@(message.structure.namespaced_type.name)__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

@[for member in message.structure.members]@
@[  if not member.has_annotation('key') and message.structure.has_any_member_with_annotation('key')]@
@[  continue]@
@[  end if]@
@[  for line in generate_member_for_get_serialized_size(member, '_key')]@
  @(line)
@[  end for]@

@[end for]@
  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_@(package_name)
size_t max_serialized_size_key_@('__'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name]))(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
@{
last_member_name_ = None
}@
@[for member in message.structure.members]@
@{
last_member_name_ = member.name
}@
@[  if not member.has_annotation('key') and message.structure.has_any_member_with_annotation('key')]@
@[  continue]@
@[  end if]@
@[  for line in generate_member_for_max_serialized_size(member, '_key')]@
  @(line)
@[  end for]@

@[end for]@
  size_t ret_val = current_alignment - initial_alignment;
@[if last_member_name_ is not None]@
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = @('__'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name]));
    is_plain =
      (
      offsetof(DataType, @(last_member_name_)) +
      last_member_size
      ) == ret_val;
  }
@[end if]@
  return ret_val;
}

@[  if message.structure.has_any_member_with_annotation('key') ]@
static bool _@(message.structure.namespaced_type.name)__cdr_serialize_key(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const @('__'.join(message.structure.namespaced_type.namespaced_name())) * ros_message = static_cast<const @('__'.join(message.structure.namespaced_type.namespaced_name())) *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_key_@('__'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name]))(ros_message, cdr);
}

static size_t _@(message.structure.namespaced_type.name)__get_serialized_size_key(
  const void * untyped_ros_message)
{
  return get_serialized_size_key_@('__'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name]))(
      untyped_ros_message, 0);
}

static
size_t
_@(message.structure.namespaced_type.name)__max_serialized_size_key(
  bool & is_unbounded)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_key_@('__'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name]))(
    full_bounded, is_plain, 0);

  is_unbounded = !full_bounded;
  return ret_val;
}

static message_type_support_key_callbacks_t __key_callbacks_@(message.structure.namespaced_type.name) = {
  _@(message.structure.namespaced_type.name)__max_serialized_size_key,
  _@(message.structure.namespaced_type.name)__get_serialized_size_key,
  _@(message.structure.namespaced_type.name)__cdr_serialize_key
};
@[  end if]@
@
@# // Collect the callback functions and provide a function to get the type support struct.

static bool _@(message.structure.namespaced_type.name)__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const @('__'.join(message.structure.namespaced_type.namespaced_name())) * ros_message = static_cast<const @('__'.join(message.structure.namespaced_type.namespaced_name())) *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_@('__'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name]))(ros_message, cdr);
}

static bool _@(message.structure.namespaced_type.name)__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  @('__'.join(message.structure.namespaced_type.namespaced_name())) * ros_message = static_cast<@('__'.join(message.structure.namespaced_type.namespaced_name())) *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_@('__'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name]))(cdr, ros_message);
}

static uint32_t _@(message.structure.namespaced_type.name)__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_@('__'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name]))(
      untyped_ros_message, 0));
}

static size_t _@(message.structure.namespaced_type.name)__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_@('__'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name]))(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

@
@# // Collect the callback functions and provide a function to get the type support struct.

static message_type_support_callbacks_t __callbacks_@(message.structure.namespaced_type.name) = {
  "@('::'.join([package_name] + list(interface_path.parents[0].parts)))",
  "@(message.structure.namespaced_type.name)",
  _@(message.structure.namespaced_type.name)__cdr_serialize,
  _@(message.structure.namespaced_type.name)__cdr_deserialize,
  _@(message.structure.namespaced_type.name)__get_serialized_size,
  _@(message.structure.namespaced_type.name)__max_serialized_size,
@[  if message.structure.has_any_member_with_annotation('key') ]@
  &__key_callbacks_@(message.structure.namespaced_type.name)
@[  else]@
  nullptr
@[  end if]@
};

static rosidl_message_type_support_t _@(message.structure.namespaced_type.name)__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_@(message.structure.namespaced_type.name),
  get_message_typesupport_handle_function,
  &@(idl_structure_type_to_c_typename(message.structure.namespaced_type))__@(GET_HASH_FUNC),
  &@(idl_structure_type_to_c_typename(message.structure.namespaced_type))__@(GET_DESCRIPTION_FUNC),
  &@(idl_structure_type_to_c_typename(message.structure.namespaced_type))__@(GET_SOURCES_FUNC),
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, @(', '.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name])))() {
  return &_@(message.structure.namespaced_type.name)__type_support;
}

#if defined(__cplusplus)
}
#endif
