@# Included from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
@{
from rosidl_generator_c import idl_structure_type_to_c_typename
from rosidl_generator_type_description import GET_DESCRIPTION_FUNC
from rosidl_generator_type_description import GET_HASH_FUNC
from rosidl_generator_type_description import GET_SOURCES_FUNC
from rosidl_parser.definition import AbstractGenericString
from rosidl_parser.definition import AbstractNestedType
from rosidl_parser.definition import AbstractSequence
from rosidl_parser.definition import AbstractWString
from rosidl_parser.definition import Array
from rosidl_parser.definition import BasicType
from rosidl_parser.definition import BoundedSequence
from rosidl_parser.definition import NamespacedType

header_files = [
    'cstddef',
    'limits',
    'stdexcept',
    'string',
    'rosidl_typesupport_cpp/message_type_support.hpp',
    'rosidl_typesupport_fastrtps_cpp/identifier.hpp',
    'rosidl_typesupport_fastrtps_cpp/message_type_support.h',
    'rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp',
    'rosidl_typesupport_fastrtps_cpp/serialization_helpers.hpp',
    'rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp',
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


// forward declaration of message dependencies and their conversion functions
@[for member in message.structure.members]@
@{
type_ = member.type
if isinstance(type_, AbstractNestedType):
    type_ = type_.value_type
}@
@[  if isinstance(type_, NamespacedType)]@
@[    if type_.namespaced_name() in forward_declared_types]@
// functions for @('::'.join(type_.namespaced_name())) already declared above
@[    else]@
@{forward_declared_types.add(type_.namespaced_name())}@
@[      for ns in type_.namespaces]@
namespace @(ns)
{
@[      end for]@
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const @('::'.join(type_.namespaced_name())) &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  @('::'.join(type_.namespaced_name())) &);
size_t get_serialized_size(
  const @('::'.join(type_.namespaced_name())) &,
  size_t current_alignment);
size_t
max_serialized_size_@(type_.name)(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
bool cdr_serialize_key(
  const @('::'.join(type_.namespaced_name())) &,
  eprosima::fastcdr::Cdr &);
size_t get_serialized_size_key(
  const @('::'.join(type_.namespaced_name())) &,
  size_t current_alignment);
size_t
max_serialized_size_key_@(type_.name)(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
@[      for ns in reversed(type_.namespaces)]@
}  // namespace @(ns)
@[      end for]@
@[    end if]@

@[  end if]@
@[end for]@
@
@[  for ns in message.structure.namespaced_type.namespaces]@

namespace @(ns)
{
@[  end for]@

@{forward_declared_types.add(message.structure.namespaced_type.namespaced_name())}@
namespace typesupport_fastrtps_cpp
{

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
  from rosidl_parser.definition import AbstractWString
  from rosidl_parser.definition import Array
  from rosidl_parser.definition import BasicType
  from rosidl_parser.definition import BoundedSequence
  from rosidl_parser.definition import NamespacedType
  strlist = []
  strlist.append('// Member: %s' % (member.name))
  if isinstance(member.type, AbstractNestedType):
    strlist.append('{')
    if isinstance(member.type, Array):
      if not isinstance(member.type.value_type, (NamespacedType, AbstractWString)):
        strlist.append('  cdr << ros_message.%s;' % (member.name))
      else:
        strlist.append('  for (size_t i = 0; i < %d; i++) {' % (member.type.size))
        if isinstance(member.type.value_type, NamespacedType):
          strlist.append('    %s::typesupport_fastrtps_cpp::cdr_serialize%s(' % (('::'.join(member.type.value_type.namespaces)), suffix))
          strlist.append('      ros_message.%s[i],' % (member.name))
          strlist.append('      cdr);')
        else:
          strlist.append('    rosidl_typesupport_fastrtps_cpp::cdr_serialize(cdr, ros_message.%s[i]);' % (member.name))
        strlist.append('  }')
    else:
      if isinstance(member.type, BoundedSequence) or isinstance(member.type.value_type, (NamespacedType, AbstractWString)):
        strlist.append('  size_t size = ros_message.%s.size();' % (member.name))
        if isinstance(member.type, BoundedSequence):
          strlist.append('  if (size > %d) {' % (member.type.maximum_size))
          strlist.append('    throw std::runtime_error("array size exceeds upper bound");')
          strlist.append('  }')
      if not isinstance(member.type.value_type, (NamespacedType, AbstractWString)) and not isinstance(member.type, BoundedSequence):
        strlist.append('  cdr << ros_message.%s;' % (member.name))
      else:
        strlist.append('  cdr << static_cast<uint32_t>(size);')
        if isinstance(member.type.value_type, BasicType) and member.type.value_type.typename not in ('boolean', 'wchar'):
          strlist.append('  if (size > 0) {')
          strlist.append('    cdr.serialize_array(&(ros_message.%s[0]), size);' % (member.name))
          strlist.append('  }')
        else:
          strlist.append('  for (size_t i = 0; i < size; i++) {')
          if isinstance(member.type.value_type, BasicType) and member.type.value_type.typename == 'boolean':
            strlist.append('    cdr << (ros_message.%s[i] ? true : false);' % (member.name))
          elif isinstance(member.type.value_type, BasicType) and member.type.value_type.typename == 'wchar':
            strlist.append('    cdr << static_cast<wchar_t>(ros_message.%s[i]);' % (member.name))
          elif isinstance(member.type.value_type, AbstractWString):
            strlist.append('    rosidl_typesupport_fastrtps_cpp::cdr_serialize(cdr, ros_message.%s[i]);' % (member.name))
          elif not isinstance(member.type.value_type, NamespacedType):
            strlist.append('    cdr << ros_message.%s[i];' % (member.name))
          else:
            strlist.append('    %s::typesupport_fastrtps_cpp::cdr_serialize%s(' % (('::'.join(member.type.value_type.namespaces)), suffix))
            strlist.append('      ros_message.%s[i],' % (member.name))
            strlist.append('      cdr);')
          strlist.append('  }')
    strlist.append('}')
  elif isinstance(member.type, BasicType) and member.type.typename == 'boolean':
    strlist.append('cdr << (ros_message.%s ? true : false);' % (member.name))
  elif isinstance(member.type, BasicType) and member.type.typename == 'wchar':
    strlist.append('cdr << static_cast<wchar_t>(ros_message.%s);' % (member.name))
  elif isinstance(member.type, AbstractWString):
    strlist.append('{')
    strlist.append('  rosidl_typesupport_fastrtps_cpp::cdr_serialize(cdr, ros_message.%s);' % (member.name))
    strlist.append('}')
  elif not isinstance(member.type, NamespacedType):
    strlist.append('cdr << ros_message.%s;' % (member.name))
  else:
    strlist.append('%s::typesupport_fastrtps_cpp::cdr_serialize%s(' % (('::'.join(member.type.namespaces)), suffix))
    strlist.append('  ros_message.%s,' % (member.name))
    strlist.append('  cdr);')
  return strlist
}@

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_@(package_name)
cdr_serialize(
  const @('::'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name])) & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
@[for member in message.structure.members]@
@[  for line in generate_member_for_cdr_serialize(member, '')]@
  @(line)
@[  end for]@

@[end for]@
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_@(package_name)
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  @('::'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name])) & ros_message)
{
@[for member in message.structure.members]@
  // Member: @(member.name)
@[  if isinstance(member.type, AbstractNestedType)]@
  {
@[    if isinstance(member.type, Array)]@
@[      if not isinstance(member.type.value_type, (NamespacedType, AbstractWString))]@
    cdr >> ros_message.@(member.name);
@[      else]@
    for (size_t i = 0; i < @(member.type.size); i++) {
@[        if isinstance(member.type.value_type, NamespacedType)]@
      @('::'.join(member.type.value_type.namespaces))::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr,
        ros_message.@(member.name)[i]);
@[        else]@
      bool succeeded = rosidl_typesupport_fastrtps_cpp::cdr_deserialize(cdr, ros_message.@(member.name)[i]);
      if (!succeeded) {
        fprintf(stderr, "failed to deserialize u16string\n");
        return false;
      }
@[        end if]@
    }
@[      end if]@
@[    else]@
@[      if not isinstance(member.type.value_type, (NamespacedType, AbstractWString)) and not isinstance(member.type, BoundedSequence)]@
    cdr >> ros_message.@(member.name);
@[      else]@
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.@(member.name).resize(size);
@[        if isinstance(member.type.value_type, BasicType) and member.type.value_type.typename not in ('boolean', 'wchar')]@
    if (size > 0) {
      cdr.deserialize_array(&(ros_message.@(member.name)[0]), size);
    }
@[        else]@
    for (size_t i = 0; i < size; i++) {
@[            if isinstance(member.type.value_type, BasicType) and member.type.value_type.typename == 'boolean']@
      uint8_t tmp;
      cdr >> tmp;
      ros_message.@(member.name)[i] = tmp ? true : false;
@[            elif isinstance(member.type.value_type, BasicType) and member.type.value_type.typename == 'wchar']@
      wchar_t tmp;
      cdr >> tmp;
      ros_message.@(member.name)[i] = static_cast<char16_t>(tmp);
@[            elif isinstance(member.type.value_type, AbstractWString)]@
      bool succeeded = rosidl_typesupport_fastrtps_cpp::cdr_deserialize(cdr, ros_message.@(member.name)[i]);
      if (!succeeded) {
        fprintf(stderr, "failed to deserialize u16string\n");
        return false;
      }
@[            elif not isinstance(member.type.value_type, NamespacedType)]@
      cdr >> ros_message.@(member.name)[i];
@[            else]@
      @('::'.join(member.type.value_type.namespaces))::typesupport_fastrtps_cpp::cdr_deserialize(
        cdr, ros_message.@(member.name)[i]);
@[            end if]@
    }
@[          end if]@
@[      end if]@
@[    end if]@
  }
@[  elif isinstance(member.type, BasicType) and member.type.typename == 'boolean']@
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.@(member.name) = tmp ? true : false;
  }
@[  elif isinstance(member.type, BasicType) and member.type.typename == 'wchar']@
  {
    wchar_t tmp;
    cdr >> tmp;
    ros_message.@(member.name) = static_cast<char16_t>(tmp);
  }
@[  elif isinstance(member.type, AbstractWString)]@
  {
    bool succeeded = rosidl_typesupport_fastrtps_cpp::cdr_deserialize(cdr, ros_message.@(member.name));
    if (!succeeded) {
      fprintf(stderr, "failed to deserialize u16string\n");
      return false;
    }
  }
@[  elif not isinstance(member.type, NamespacedType)]@
  cdr >> ros_message.@(member.name);
@[  else]@
  @('::'.join(member.type.namespaces))::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.@(member.name));
@[  end if]@

@[end for]@
  return true;
}

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
  from rosidl_parser.definition import AbstractWString
  from rosidl_parser.definition import Array
  from rosidl_parser.definition import BasicType
  from rosidl_parser.definition import BoundedSequence
  from rosidl_parser.definition import NamespacedType
  strlist = []
  strlist.append('// Member: %s' % (member.name))

  if isinstance(member.type, AbstractNestedType):
    strlist.append('{')
    if isinstance(member.type, Array):
      strlist.append('  size_t array_size = %d;' % (member.type.size))
    else:
      strlist.append('  size_t array_size = ros_message.%s.size();' % (member.name))
      if isinstance(member.type, BoundedSequence):
        strlist.append('  if (array_size > %d) {' % (member.type.maximum_size))
        strlist.append('    throw std::runtime_error("array size exceeds upper bound");')
        strlist.append('  }')
      strlist.append('  current_alignment += padding +')
      strlist.append('    eprosima::fastcdr::Cdr::alignment(current_alignment, padding);')
    if isinstance(member.type.value_type, AbstractGenericString):
      strlist.append('  for (size_t index = 0; index < array_size; ++index) {')
      strlist.append('    current_alignment += padding +')
      strlist.append('      eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +')
      if isinstance(member.type.value_type, AbstractWString):
        strlist.append('      wchar_size *')
      strlist.append('      (ros_message.%s[index].size() + 1);' % (member.name))
      strlist.append('  }')
    elif isinstance(member.type.value_type, BasicType):
      strlist.append('  size_t item_size = sizeof(ros_message.%s[0]);' % (member.name))
      strlist.append('  current_alignment += array_size * item_size +')
      strlist.append('    eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);')
    else:
      strlist.append('  for (size_t index = 0; index < array_size; ++index) {')
      strlist.append('    current_alignment +=')
      strlist.append('      %s::typesupport_fastrtps_cpp::get_serialized_size%s(' % (('::'.join(member.type.value_type.namespaces)), suffix))
      strlist.append('      ros_message.%s[index], current_alignment);' % (member.name))
      strlist.append('  }')
    strlist.append('}')
  else:
    if isinstance(member.type, AbstractGenericString):
      strlist.append('current_alignment += padding +')
      strlist.append('  eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +')
      if isinstance(member.type, AbstractWString):
        strlist.append('  wchar_size *')
      strlist.append('  (ros_message.%s.size() + 1);' % (member.name))
    elif isinstance(member.type, BasicType):
      strlist.append('{')
      strlist.append('  size_t item_size = sizeof(ros_message.%s);' % (member.name))
      strlist.append('  current_alignment += item_size +')
      strlist.append('    eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);')
      strlist.append('}')
    else:
      strlist.append('current_alignment +=')
      strlist.append('  %s::typesupport_fastrtps_cpp::get_serialized_size%s(' % (('::'.join(member.type.namespaces)), suffix))
      strlist.append('  ros_message.%s, current_alignment);' % (member.name))

  return strlist;
}@

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_@(package_name)
get_serialized_size(
  const @('::'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name])) & ros_message,
  size_t current_alignment)
{
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
  from rosidl_parser.definition import AbstractWString
  from rosidl_parser.definition import Array
  from rosidl_parser.definition import BasicType
  from rosidl_parser.definition import BoundedSequence
  from rosidl_parser.definition import NamespacedType
  strlist = []
  strlist.append('// Member: %s' % (member.name))
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
    strlist.append('    size_t inner_size =')
    strlist.append('      %s::typesupport_fastrtps_cpp::max_serialized_size%s_%s(' % (('::'.join(type_.namespaces)), suffix, type_.name))
    strlist.append('      inner_full_bounded, inner_is_plain, current_alignment);')
    strlist.append('    last_member_size += inner_size;')
    strlist.append('    current_alignment += inner_size;')
    strlist.append('    full_bounded &= inner_full_bounded;')
    strlist.append('    is_plain &= inner_is_plain;')
    strlist.append('  }')
  strlist.append('}')
  return strlist
}@

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_@(package_name)
max_serialized_size_@(message.structure.namespaced_type.name)(
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
    using DataType = @('::'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name]));
    is_plain =
      (
      offsetof(DataType, @(last_member_name_)) +
      last_member_size
      ) == ret_val;
  }

@[end if]@
  return ret_val;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_@(package_name)
cdr_serialize_key(
  const @('::'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name])) & ros_message,
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

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_@(package_name)
get_serialized_size_key(
  const @('::'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name])) & ros_message,
  size_t current_alignment)
{
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

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_@(package_name)
max_serialized_size_key_@(message.structure.namespaced_type.name)(
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
    using DataType = @('::'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name]));
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
  auto typed_message =
    static_cast<const @('::'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name])) *>(
    untyped_ros_message);

  return cdr_serialize_key(*typed_message, cdr);
}

static
size_t
_@(message.structure.namespaced_type.name)__get_serialized_size_key(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const @('::'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name])) *>(
    untyped_ros_message);

  return get_serialized_size_key(*typed_message, 0);
}

static size_t _@(message.structure.namespaced_type.name)__max_serialized_size_key(
  bool & is_unbounded)
{
  bool full_bounded = true;
  bool is_plain = true;

  size_t ret_val = max_serialized_size_key_@(message.structure.namespaced_type.name)(
    full_bounded,
    is_plain,
    0);

  is_unbounded = !full_bounded;
  return ret_val;
}

static message_type_support_key_callbacks_t _@(message.structure.namespaced_type.name)__key_callbacks = {
  _@(message.structure.namespaced_type.name)__max_serialized_size_key,
  _@(message.structure.namespaced_type.name)__get_serialized_size_key,
  _@(message.structure.namespaced_type.name)__cdr_serialize_key
};
@[  end if]@

static bool _@(message.structure.namespaced_type.name)__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const @('::'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name])) *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _@(message.structure.namespaced_type.name)__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<@('::'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name])) *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _@(message.structure.namespaced_type.name)__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const @('::'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name])) *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _@(message.structure.namespaced_type.name)__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_@(message.structure.namespaced_type.name)(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _@(message.structure.namespaced_type.name)__callbacks = {
  "@('::'.join([package_name] + list(interface_path.parents[0].parts)))",
  "@(message.structure.namespaced_type.name)",
  _@(message.structure.namespaced_type.name)__cdr_serialize,
  _@(message.structure.namespaced_type.name)__cdr_deserialize,
  _@(message.structure.namespaced_type.name)__get_serialized_size,
  _@(message.structure.namespaced_type.name)__max_serialized_size,
@[  if message.structure.has_any_member_with_annotation('key') ]@
  &_@(message.structure.namespaced_type.name)__key_callbacks
@[  else]@
  nullptr
@[  end if]@
};

static rosidl_message_type_support_t _@(message.structure.namespaced_type.name)__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_@(message.structure.namespaced_type.name)__callbacks,
  get_message_typesupport_handle_function,
  &@(idl_structure_type_to_c_typename(message.structure.namespaced_type))__@(GET_HASH_FUNC),
  &@(idl_structure_type_to_c_typename(message.structure.namespaced_type))__@(GET_DESCRIPTION_FUNC),
  &@(idl_structure_type_to_c_typename(message.structure.namespaced_type))__@(GET_SOURCES_FUNC),
};

}  // namespace typesupport_fastrtps_cpp
@[  for ns in reversed(message.structure.namespaced_type.namespaces)]@

}  // namespace @(ns)
@[  end for]@

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_@(package_name)
const rosidl_message_type_support_t *
get_message_type_support_handle<@('::'.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name]))>()
{
  return &@('::'.join([package_name] + list(interface_path.parents[0].parts)))::typesupport_fastrtps_cpp::_@(message.structure.namespaced_type.name)__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, @(', '.join([package_name] + list(interface_path.parents[0].parts) + [message.structure.namespaced_type.name])))() {
  return &@('::'.join([package_name] + list(interface_path.parents[0].parts)))::typesupport_fastrtps_cpp::_@(message.structure.namespaced_type.name)__handle;
}

#ifdef __cplusplus
}
#endif
