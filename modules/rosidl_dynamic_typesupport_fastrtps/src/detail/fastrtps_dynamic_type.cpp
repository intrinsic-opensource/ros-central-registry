// Copyright 2022 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "fastrtps_dynamic_type.hpp"

#include <fastdds/dds/xtypes/dynamic_types/DynamicType.hpp>
#include <fastdds/dds/xtypes/dynamic_types/DynamicTypeBuilder.hpp>
#include <fastdds/dds/xtypes/dynamic_types/DynamicTypeMember.hpp>
#include <fastdds/dds/xtypes/dynamic_types/TypeDescriptor.hpp>

#include <rcutils/allocator.h>
#include <rcutils/strdup.h>

#include "rcpputils/scope_exit.hpp"

#include <rosidl_runtime_c/type_description/field__functions.h>
#include <rosidl_runtime_c/type_description/field__struct.h>
#include <rosidl_runtime_c/type_description/field_type__functions.h>
#include <rosidl_runtime_c/type_description/field_type__struct.h>
#include <rosidl_runtime_c/type_description/individual_type_description__functions.h>
#include <rosidl_runtime_c/type_description/individual_type_description__struct.h>
#include <rosidl_runtime_c/type_description/type_description__functions.h>
#include <rosidl_runtime_c/type_description/type_description__struct.h>
#include <rosidl_runtime_c/type_description_utils.h>

#include <rosidl_dynamic_typesupport/api/serialization_support_interface.h>
#include <rosidl_dynamic_typesupport/types.h>

#include <memory>
#include <string>
#include <utility>

#include "fastrtps_serialization_support.hpp"
#include "macros.hpp"
#include "utils.hpp"

using eprosima::fastdds::dds::DynamicType;
using eprosima::fastdds::dds::DynamicTypeBuilder;
using eprosima::fastdds::dds::DynamicTypeMember;
using eprosima::fastdds::dds::MemberDescriptor;
using eprosima::fastdds::dds::TypeDescriptor;

#define CONTAINER_UNLIMITED 0

// =================================================================================================
// DYNAMIC TYPE
// =================================================================================================

// DYNAMIC TYPE UTILS =======================================================================
rcutils_ret_t
fastdds__dynamic_type_equals(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_type_impl_t * type_impl,
  const rosidl_dynamic_typesupport_dynamic_type_impl_t * other_type_impl,
  bool * equals)
{
  (void) serialization_support_impl;
  auto type =
    static_cast<const fastdds__rosidl_dynamic_typesupport_dynamic_type_impl *>(type_impl->handle);
  auto other =
    static_cast<const fastdds__rosidl_dynamic_typesupport_dynamic_type_impl *>(other_type_impl->
    handle);

  *equals = type->ref_type->equals(other->ref_type);
  return RCUTILS_RET_OK;
}

rcutils_ret_t
fastdds__dynamic_type_get_member_count(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_type_impl_t * type_impl,
  size_t * member_count)
{
  (void) serialization_support_impl;
  auto type =
    static_cast<const fastdds__rosidl_dynamic_typesupport_dynamic_type_impl *>(type_impl->handle);

  *member_count = type->ref_type->get_member_count();
  return RCUTILS_RET_OK;
}

// DYNAMIC TYPE CONSTRUCTION =======================================================================
rcutils_ret_t
fastdds__dynamic_type_builder_init(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const char * name,
  size_t name_length,
  rcutils_allocator_t * allocator,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl)
{
  (void) allocator;

  auto fastdds_impl =
    static_cast<fastdds__serialization_support_impl_handle_t *>(serialization_support_impl->handle);

  // We must replace "/" with "::" in type names
  std::string name_string = fastdds__replace_string(std::string(name, name_length), "/", "::");

  auto type_builder_handle =
    new (std::nothrow) fastdds__rosidl_dynamic_typesupport_dynamic_type_builder_impl();
  if (!type_builder_handle) {
    RCUTILS_SET_ERROR_MSG("Could not init new struct type builder");
    return RCUTILS_RET_BAD_ALLOC;
  }

  auto cleanup_type_builder_handle = rcpputils::make_scope_exit(
    [type_builder_handle]()
    {
      delete type_builder_handle;
    });

  TypeDescriptor::_ref_type struct_descriptor =
    eprosima::fastdds::dds::traits<TypeDescriptor>::make_shared();
  struct_descriptor->kind(eprosima::fastdds::dds::TK_STRUCTURE);
  struct_descriptor->name(name_string);

  type_builder_handle->ref_type = fastdds_impl->type_factory_->create_type(struct_descriptor);
  if (!type_builder_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not create new struct type builder");
    return RCUTILS_RET_BAD_ALLOC;
  }

  type_builder_impl->handle = std::move(type_builder_handle);

  cleanup_type_builder_handle.cancel();

  return RCUTILS_RET_OK;
}

rcutils_ret_t
fastdds__dynamic_type_builder_clone(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * other,
  rcutils_allocator_t * allocator,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl)
{
  auto fastdds_impl =
    static_cast<fastdds__serialization_support_impl_handle_t *>(serialization_support_impl->handle);

  auto type_builder_handle =
    new (std::nothrow) fastdds__rosidl_dynamic_typesupport_dynamic_type_builder_impl();
  if (!type_builder_handle) {
    RCUTILS_SET_ERROR_MSG("Could not init new struct type builder");
    return RCUTILS_RET_BAD_ALLOC;
  }

  auto cleanup_type_builder_handle = rcpputils::make_scope_exit(
    [type_builder_handle]()
    {
      delete type_builder_handle;
    });
  auto other_type_builder_handle =
    static_cast<const fastdds__rosidl_dynamic_typesupport_dynamic_type_builder_impl *>(other->handle);
  if (!other_type_builder_handle || !other_type_builder_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to type impl");
    return RCUTILS_RET_ERROR;
  }
  type_builder_handle->ref_type =
    fastdds_impl->type_factory_->create_type_copy(other_type_builder_handle->ref_type->build());
  if (!type_builder_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not clone struct type builder");
    return RCUTILS_RET_ERROR;
  }

  type_builder_impl->handle = std::move(type_builder_handle);
  type_builder_impl->allocator = *allocator;

  cleanup_type_builder_handle.cancel();
  return RCUTILS_RET_OK;
}

rcutils_ret_t
fastdds__dynamic_type_builder_fini(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl)
{
  (void) serialization_support_impl;
  auto type_builder_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_type_builder_impl *>(
    type_builder_impl->handle);
  if (!type_builder_handle || !type_builder_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to type impl");
    return RCUTILS_RET_ERROR;
  }
  type_builder_handle->ref_type.reset();
  delete type_builder_handle;
  return RCUTILS_RET_OK;
}

rcutils_ret_t
fastdds__dynamic_type_init_from_dynamic_type_builder(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  rcutils_allocator_t * allocator,
  rosidl_dynamic_typesupport_dynamic_type_impl_t * type_impl)
{
  (void) serialization_support_impl;

  auto type_builder_handle =
    static_cast<const fastdds__rosidl_dynamic_typesupport_dynamic_type_builder_impl *>(
    type_builder_impl->handle);
  if (!type_builder_handle || !type_builder_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to type impl");
    return RCUTILS_RET_ERROR;
  }

  auto type_handle = new (std::nothrow) fastdds__rosidl_dynamic_typesupport_dynamic_type_impl();
  if (!type_handle) {
    RCUTILS_SET_ERROR_MSG("Could not init new struct type");
    return RCUTILS_RET_BAD_ALLOC;
  }

  auto cleanup_type_handle = rcpputils::make_scope_exit(
    [type_handle]()
    {
      delete type_handle;
    });

  type_handle->ref_type = type_builder_handle->ref_type->build();
  if (!type_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not create dynamic type from dynamic type builder");
    return RCUTILS_RET_BAD_ALLOC;
  }

  type_impl->handle = type_handle;
  type_impl->allocator = *allocator;
  return RCUTILS_RET_OK;
}

rcutils_ret_t
fastdds__dynamic_type_clone(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_type_impl_t * other,
  rcutils_allocator_t * allocator,
  rosidl_dynamic_typesupport_dynamic_type_impl_t * type_impl)
{
  (void) serialization_support_impl;
  type_impl->allocator = *allocator;

  auto type_impl_handle =
    static_cast<const fastdds__rosidl_dynamic_typesupport_dynamic_type_impl *>(other->handle);
  if (!type_impl_handle || !type_impl_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to type impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  auto type_impl_out_handle =
    new (std::nothrow) fastdds__rosidl_dynamic_typesupport_dynamic_type_impl();
  if (!type_impl_out_handle) {
    RCUTILS_SET_ERROR_MSG("Could not init new struct type");
    return RCUTILS_RET_BAD_ALLOC;
  }

  auto cleanup_type_impl_out_handle = rcpputils::make_scope_exit(
    [type_impl_out_handle]()
    {
      delete type_impl_out_handle;
    });

  type_impl_out_handle->ref_type = type_impl_handle->ref_type;

  type_impl->handle = type_impl_out_handle;

  cleanup_type_impl_out_handle.cancel();

  return RCUTILS_RET_OK;
}

rcutils_ret_t
fastdds__dynamic_type_fini(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_impl_t * type_impl)
{
  auto fastdds_impl =
    static_cast<fastdds__serialization_support_impl_handle_t *>(serialization_support_impl->handle);
  auto type_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_type_impl *>(type_impl->handle);

  if (!type_handle || !type_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to type impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  FASTDDS_CHECK_RET_FOR_NOT_OK_WITH_MSG(
    fastdds_impl->type_factory_->delete_type(type_handle->ref_type),
    "Could not fini type");
  delete type_handle;
  return RCUTILS_RET_OK;
}

rcutils_ret_t
fastdds__dynamic_type_get_name(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_type_impl_t * type_impl,
  const char ** name,
  size_t * name_length)
{
  (void) serialization_support_impl;
  auto type_handle =
    static_cast<const fastdds__rosidl_dynamic_typesupport_dynamic_type_impl *>(type_impl->handle);

  if (!type_handle || !type_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to type impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  // Undo the mangling
  std::string tmp_name = fastdds__replace_string(
    type_handle->ref_type->get_name().to_string(), "::", "/");
  *name = rcutils_strdup(tmp_name.c_str(), type_impl->allocator);
  *name_length = tmp_name.size();
  return RCUTILS_RET_OK;
}

rcutils_ret_t
fastdds__dynamic_type_builder_get_name(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  const char ** name,
  size_t * name_length)
{
  (void) serialization_support_impl;

  auto type_builder_handle =
    static_cast<const fastdds__rosidl_dynamic_typesupport_dynamic_type_builder_impl *>(
    type_builder_impl->handle);

  if (!type_builder_handle || !type_builder_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to type builder impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  // Undo the mangling
  std::string tmp_name = fastdds__replace_string(
    type_builder_handle->ref_type->get_name().to_string(), "::", "/");
  *name = rcutils_strdup(tmp_name.c_str(), type_builder_impl->allocator);
  *name_length = tmp_name.size();
  return RCUTILS_RET_OK;
}

rcutils_ret_t
fastdds__dynamic_type_builder_set_name(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  const char * name,
  size_t name_length)
{
  auto fastdds_impl =
    static_cast<fastdds__serialization_support_impl_handle_t *>(serialization_support_impl->handle);

  auto type_builder_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_type_builder_impl *>(
    type_builder_impl->handle);

  if (!type_builder_handle || !type_builder_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to type builder impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  TypeDescriptor::_ref_type descriptor =
    eprosima::fastdds::dds::traits<TypeDescriptor>::make_shared();

  type_builder_handle->ref_type->get_descriptor(descriptor);
  std::string name_string = fastdds__replace_string(std::string(name, name_length), "/", "::");
  descriptor->name(name_string);

  auto new_type_builder = fastdds_impl->type_factory_->create_type(descriptor);
  if (!new_type_builder) {
    RCUTILS_SET_ERROR_MSG("Could not create new type builder");
    return RCUTILS_RET_BAD_ALLOC;
  }

  for (uint32_t index {0}; index < type_builder_handle->ref_type->get_member_count(); ++index) {
    DynamicTypeMember::_ref_type member;
    if (eprosima::fastdds::dds::RETCODE_OK !=
      type_builder_handle->ref_type->get_member_by_index(member, index))
    {
      RCUTILS_SET_ERROR_MSG("Could not get type builder member");
      return RCUTILS_RET_ERROR;
    }

    MemberDescriptor::_ref_type member_descriptor =
      eprosima::fastdds::dds::traits<MemberDescriptor>::make_shared();
    if (eprosima::fastdds::dds::RETCODE_OK !=
      member->get_descriptor(member_descriptor))
    {
      RCUTILS_SET_ERROR_MSG("Could not get type builder member descriptor");
      return RCUTILS_RET_ERROR;
    }

    if (eprosima::fastdds::dds::RETCODE_OK !=
      new_type_builder->add_member(member_descriptor))
    {
      RCUTILS_SET_ERROR_MSG("Could not create new type builder member");
      return RCUTILS_RET_ERROR;
    }
  }

  type_builder_handle->ref_type = new_type_builder;

  return RCUTILS_RET_OK;
}

// DYNAMIC TYPE PRIMITIVE MEMBERS ==================================================================
#define FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_MEMBER_FN(FunctionT, MemberT) \
  rcutils_ret_t fastdds__dynamic_type_builder_add_ ## FunctionT ## _member( \
    rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, \
    rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl, \
    rosidl_dynamic_typesupport_member_id_t id, const char * name, size_t name_length, \
    const char * default_value, \
    size_t default_value_length) \
  { \
    auto fastdds_impl = \
      static_cast<fastdds__serialization_support_impl_handle_t *>(serialization_support_impl->handle); \
 \
    auto type_builder_handle = \
      static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_type_builder_impl *>( \
      type_builder_impl-> \
      handle); \
    if (!type_builder_handle || !type_builder_handle->ref_type) \
    { \
      RCUTILS_SET_ERROR_MSG("Could not get handle to type builder impl"); \
      return RCUTILS_RET_INVALID_ARGUMENT; \
    } \
 \
    MemberDescriptor::_ref_type descriptor = \
      eprosima::fastdds::dds::traits<MemberDescriptor>::make_shared(); \
    descriptor->type(fastdds_impl->type_factory_->get_primitive_type(MemberT)); \
    descriptor->name(std::string(name, name_length)); \
    descriptor->id(fastdds__size_t_to_uint32_t(id)); \
    descriptor->default_value(std::string(default_value, default_value_length)); \
    FASTDDS_CHECK_RET_FOR_NOT_OK_AND_RETURN_WITH_MSG( \
      type_builder_handle->ref_type->add_member(descriptor), \
      "Could not add `" #MemberT "` member to type builder"); \
  }

FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_MEMBER_FN(bool, eprosima::fastdds::dds::TK_BOOLEAN)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_MEMBER_FN(byte, eprosima::fastdds::dds::TK_BYTE)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_MEMBER_FN(char, eprosima::fastdds::dds::TK_CHAR8)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_MEMBER_FN(wchar, eprosima::fastdds::dds::TK_CHAR16)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_MEMBER_FN(float32, eprosima::fastdds::dds::TK_FLOAT32)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_MEMBER_FN(float64, eprosima::fastdds::dds::TK_FLOAT64)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_MEMBER_FN(int8, eprosima::fastdds::dds::TK_INT8)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_MEMBER_FN(uint8, eprosima::fastdds::dds::TK_UINT8)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_MEMBER_FN(int16, eprosima::fastdds::dds::TK_INT16)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_MEMBER_FN(uint16, eprosima::fastdds::dds::TK_UINT16)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_MEMBER_FN(int32, eprosima::fastdds::dds::TK_INT32)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_MEMBER_FN(uint32, eprosima::fastdds::dds::TK_UINT32)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_MEMBER_FN(int64, eprosima::fastdds::dds::TK_INT64)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_MEMBER_FN(uint64, eprosima::fastdds::dds::TK_UINT64)
#undef FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_MEMBER_FN

rcutils_ret_t
fastdds__dynamic_type_builder_add_string_member(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name,
  size_t name_length,
  const char * default_value,
  size_t default_value_length)
{
  return fastdds__dynamic_type_builder_add_bounded_string_member(
    serialization_support_impl, type_builder_impl, id,
    name, name_length, default_value,
    default_value_length, CONTAINER_UNLIMITED);
}

rcutils_ret_t
fastdds__dynamic_type_builder_add_wstring_member(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name,
  size_t name_length,
  const char * default_value,
  size_t default_value_length)
{
  return fastdds__dynamic_type_builder_add_bounded_wstring_member(
    serialization_support_impl, type_builder_impl, id,
    name, name_length, default_value,
    default_value_length, CONTAINER_UNLIMITED);
}

rcutils_ret_t
fastdds__dynamic_type_builder_add_fixed_string_member(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name,
  size_t name_length,
  const char * default_value,
  size_t default_value_length,
  size_t string_length)
{
  // Fixed string is bounded on the wire
  return fastdds__dynamic_type_builder_add_bounded_wstring_member(
    serialization_support_impl, type_builder_impl, id,
    name, name_length, default_value,
    default_value_length, string_length);
}

rcutils_ret_t
fastdds__dynamic_type_builder_add_fixed_wstring_member(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name,
  size_t name_length,
  const char * default_value,
  size_t default_value_length,
  size_t wstring_length)
{
  // Fixed string is bounded on the wire
  return fastdds__dynamic_type_builder_add_bounded_wstring_member(
    serialization_support_impl, type_builder_impl, id,
    name, name_length, default_value,
    default_value_length, wstring_length);
}

rcutils_ret_t
fastdds__dynamic_type_builder_add_bounded_string_member(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name,
  size_t name_length,
  const char * default_value,
  size_t default_value_length,
  size_t string_bound)
{
  auto fastdds_impl =
    static_cast<fastdds__serialization_support_impl_handle_t *>(serialization_support_impl->handle);

  auto type_builder_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_type_builder_impl *>(
    type_builder_impl->handle);

  if (!type_builder_handle || !type_builder_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to type builder impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  MemberDescriptor::_ref_type descriptor =
    eprosima::fastdds::dds::traits<MemberDescriptor>::make_shared();
  descriptor->type(
    fastdds_impl->type_factory_->create_string_type(
      fastdds__size_t_to_uint32_t(
        string_bound))->build());
  descriptor->name(std::string(name, name_length));
  descriptor->id(fastdds__size_t_to_uint32_t(id));
  descriptor->default_value(std::string(default_value, default_value_length));

  FASTDDS_CHECK_RET_FOR_NOT_OK_AND_RETURN_WITH_MSG(
    type_builder_handle->ref_type
    ->add_member(descriptor),
    "Could not add string member");
}

rcutils_ret_t
fastdds__dynamic_type_builder_add_bounded_wstring_member(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name,
  size_t name_length,
  const char * default_value,
  size_t default_value_length,
  size_t wstring_bound)
{
  auto fastdds_impl =
    static_cast<fastdds__serialization_support_impl_handle_t *>(serialization_support_impl->handle);

  auto type_builder_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_type_builder_impl *>(
    type_builder_impl->handle);

  if (!type_builder_handle || !type_builder_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to type builder impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  MemberDescriptor::_ref_type descriptor =
    eprosima::fastdds::dds::traits<MemberDescriptor>::make_shared();
  descriptor->type(
    fastdds_impl->type_factory_->create_wstring_type(
      fastdds__size_t_to_uint32_t(
        wstring_bound))->build());
  descriptor->name(std::string(name, name_length));
  descriptor->id(fastdds__size_t_to_uint32_t(id));
  descriptor->default_value(std::string(default_value, default_value_length));

  FASTDDS_CHECK_RET_FOR_NOT_OK_AND_RETURN_WITH_MSG(
    type_builder_handle->ref_type
    ->add_member(descriptor),
    "Could not add wstring member");
}

// DYNAMIC TYPE STATIC ARRAY MEMBERS ===============================================================
#define FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN(FunctionT, MemberT) \
  rcutils_ret_t fastdds__dynamic_type_builder_add_ ## FunctionT ## _array_member( \
    rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, \
    rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl, \
    rosidl_dynamic_typesupport_member_id_t id, const char * name, size_t name_length, \
    const char * default_value, \
    size_t default_value_length, size_t array_length) \
  { \
    auto fastdds_impl = \
      static_cast<fastdds__serialization_support_impl_handle_t *>(serialization_support_impl->handle); \
 \
    auto type_builder_handle = \
      static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_type_builder_impl *>( \
      type_builder_impl-> \
      handle); \
    if (!type_builder_handle || !type_builder_handle->ref_type) \
    { \
      RCUTILS_SET_ERROR_MSG("Could not get handle to type builder impl"); \
      return RCUTILS_RET_INVALID_ARGUMENT; \
    } \
 \
    MemberDescriptor::_ref_type descriptor = \
      eprosima::fastdds::dds::traits<MemberDescriptor>::make_shared(); \
    descriptor->type( \
      fastdds_impl->type_factory_->create_array_type( \
        fastdds_impl->type_factory_->get_primitive_type(MemberT), \
        {fastdds__size_t_to_uint32_t(array_length)})->build()); \
    descriptor->name(std::string(name, name_length)); \
    descriptor->id(fastdds__size_t_to_uint32_t(id)); \
    descriptor->default_value(std::string(default_value, default_value_length)); \
 \
    FASTDDS_CHECK_RET_FOR_NOT_OK_AND_RETURN_WITH_MSG( \
      type_builder_handle->ref_type->add_member(descriptor), \
      "Could not add `" #MemberT "` array member to type builder"); \
  }

FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN(bool, eprosima::fastdds::dds::TK_BOOLEAN)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN(byte, eprosima::fastdds::dds::TK_BYTE)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN(char, eprosima::fastdds::dds::TK_CHAR8)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN(wchar, eprosima::fastdds::dds::TK_CHAR16)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN(float32, eprosima::fastdds::dds::TK_FLOAT32)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN(float64, eprosima::fastdds::dds::TK_FLOAT64)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN(int8, eprosima::fastdds::dds::TK_INT8)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN(uint8, eprosima::fastdds::dds::TK_UINT8)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN(int16, eprosima::fastdds::dds::TK_INT16)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN(uint16, eprosima::fastdds::dds::TK_UINT16)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN(int32, eprosima::fastdds::dds::TK_INT32)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN(uint32, eprosima::fastdds::dds::TK_UINT32)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN(int64, eprosima::fastdds::dds::TK_INT64)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN(uint64, eprosima::fastdds::dds::TK_UINT64)
#undef FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN

rcutils_ret_t
fastdds__dynamic_type_builder_add_string_array_member(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name,
  size_t name_length,
  const char * default_value,
  size_t default_value_length,
  size_t array_length)
{
  return fastdds__dynamic_type_builder_add_bounded_string_array_member(
    serialization_support_impl, type_builder_impl, id, name, name_length, default_value,
    default_value_length,
    CONTAINER_UNLIMITED, array_length);
}

rcutils_ret_t
fastdds__dynamic_type_builder_add_wstring_array_member(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name,
  size_t name_length,
  const char * default_value,
  size_t default_value_length,
  size_t array_length)
{
  return fastdds__dynamic_type_builder_add_bounded_wstring_array_member(
    serialization_support_impl, type_builder_impl, id, name, name_length, default_value,
    default_value_length,
    CONTAINER_UNLIMITED, array_length);
}

rcutils_ret_t
fastdds__dynamic_type_builder_add_fixed_string_array_member(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name,
  size_t name_length,
  const char * default_value,
  size_t default_value_length,
  size_t string_length,
  size_t array_length)
{
  // Fixed string is bounded on the wire
  return fastdds__dynamic_type_builder_add_bounded_string_array_member(
    serialization_support_impl, type_builder_impl, id, name, name_length, default_value,
    default_value_length,
    string_length, array_length);
}

rcutils_ret_t
fastdds__dynamic_type_builder_add_fixed_wstring_array_member(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name,
  size_t name_length,
  const char * default_value,
  size_t default_value_length,
  size_t wstring_length,
  size_t array_length)
{
  // Fixed string is bounded on the wire
  return fastdds__dynamic_type_builder_add_bounded_wstring_array_member(
    serialization_support_impl, type_builder_impl, id, name, name_length, default_value,
    default_value_length,
    wstring_length, array_length);
}

rcutils_ret_t
fastdds__dynamic_type_builder_add_bounded_string_array_member(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name,
  size_t name_length,
  const char * default_value,
  size_t default_value_length,
  size_t string_bound,
  size_t array_length)
{
  auto fastdds_impl =
    static_cast<fastdds__serialization_support_impl_handle_t *>(serialization_support_impl->handle);

  auto type_builder_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_type_builder_impl *>(
    type_builder_impl->handle);

  if (!type_builder_handle || !type_builder_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to type builder impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  MemberDescriptor::_ref_type descriptor =
    eprosima::fastdds::dds::traits<MemberDescriptor>::make_shared();
  descriptor->type(
    fastdds_impl->type_factory_->create_array_type(
      fastdds_impl->type_factory_->create_string_type(
        fastdds__size_t_to_uint32_t(string_bound))->build(),
      {fastdds__size_t_to_uint32_t(array_length)})->build());
  descriptor->name(std::string(name, name_length));
  descriptor->id(fastdds__size_t_to_uint32_t(id));
  descriptor->default_value(std::string(default_value, default_value_length));

  FASTDDS_CHECK_RET_FOR_NOT_OK_AND_RETURN_WITH_MSG(
    type_builder_handle->ref_type->add_member(descriptor),
    "Could not add bounded `string` array member to type builder");
}

rcutils_ret_t
fastdds__dynamic_type_builder_add_bounded_wstring_array_member(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name,
  size_t name_length,
  const char * default_value,
  size_t default_value_length,
  size_t wstring_bound,
  size_t array_length)
{
  auto fastdds_impl =
    static_cast<fastdds__serialization_support_impl_handle_t *>(serialization_support_impl->handle);

  auto type_builder_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_type_builder_impl *>(
    type_builder_impl->handle);

  if (!type_builder_handle || !type_builder_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to type builder impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  MemberDescriptor::_ref_type descriptor =
    eprosima::fastdds::dds::traits<MemberDescriptor>::make_shared();
  descriptor->type(
    fastdds_impl->type_factory_->create_array_type(
      fastdds_impl->type_factory_->create_wstring_type(
        fastdds__size_t_to_uint32_t(wstring_bound))->build(),
      {fastdds__size_t_to_uint32_t(array_length)})->build());
  descriptor->name(std::string(name, name_length));
  descriptor->id(fastdds__size_t_to_uint32_t(id));
  descriptor->default_value(std::string(default_value, default_value_length));

  FASTDDS_CHECK_RET_FOR_NOT_OK_AND_RETURN_WITH_MSG(
    type_builder_handle->ref_type->add_member(descriptor),
    "Could not add bounded `wstring` array member to type builder");
}

// DYNAMIC TYPE UNBOUNDED SEQUENCE MEMBERS =========================================================
#define FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(FunctionT) \
  rcutils_ret_t fastdds__dynamic_type_builder_add_ ## FunctionT ## _unbounded_sequence_member( \
    rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, \
    rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl, \
    rosidl_dynamic_typesupport_member_id_t id, const char * name, size_t name_length, \
    const char * default_value, \
    size_t default_value_length) \
  { \
    return fastdds__dynamic_type_builder_add_ ## FunctionT ## _bounded_sequence_member( \
      serialization_support_impl, type_builder_impl, fastdds__size_t_to_uint32_t( \
        id), name, name_length, \
      default_value, default_value_length, CONTAINER_UNLIMITED); \
  }

// NOTE(methylDragon): These call the bounded sequence versions with "unlimited" bound!
//                     So the type args are the same!!
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(bool)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(byte)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(char)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(wchar)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(float32)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(float64)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(int8)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(uint8)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(int16)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(uint16)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(int32)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(uint32)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(int64)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(uint64)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(string)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(wstring)
#undef FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN

rcutils_ret_t
fastdds__dynamic_type_builder_add_fixed_string_unbounded_sequence_member(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name,
  size_t name_length,
  const char * default_value,
  size_t default_value_length,
  size_t string_length)
{
  // Fixed string is bounded on the wire
  return fastdds__dynamic_type_builder_add_bounded_string_bounded_sequence_member(
    serialization_support_impl, type_builder_impl, id, name, name_length, default_value,
    default_value_length,
    string_length, CONTAINER_UNLIMITED);
}

rcutils_ret_t
fastdds__dynamic_type_builder_add_fixed_wstring_unbounded_sequence_member(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name,
  size_t name_length,
  const char * default_value,
  size_t default_value_length,
  size_t wstring_length)
{
  // Fixed string is bounded on the wire
  return fastdds__dynamic_type_builder_add_bounded_wstring_bounded_sequence_member(
    serialization_support_impl, type_builder_impl, id, name, name_length, default_value,
    default_value_length,
    wstring_length, CONTAINER_UNLIMITED);
}

rcutils_ret_t
fastdds__dynamic_type_builder_add_bounded_string_unbounded_sequence_member(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name,
  size_t name_length,
  const char * default_value,
  size_t default_value_length,
  size_t string_bound)
{
  return fastdds__dynamic_type_builder_add_bounded_string_bounded_sequence_member(
    serialization_support_impl, type_builder_impl, id, name, name_length, default_value,
    default_value_length,
    string_bound, CONTAINER_UNLIMITED);
}

rcutils_ret_t
fastdds__dynamic_type_builder_add_bounded_wstring_unbounded_sequence_member(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name,
  size_t name_length,
  const char * default_value,
  size_t default_value_length,
  size_t wstring_bound)
{
  return fastdds__dynamic_type_builder_add_bounded_wstring_bounded_sequence_member(
    serialization_support_impl, type_builder_impl, id, name, name_length, default_value,
    default_value_length,
    wstring_bound, CONTAINER_UNLIMITED);
}

// DYNAMIC TYPE BOUNDED SEQUENCE MEMBERS ===========================================================
#define FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN(FunctionT, MemberT) \
  rcutils_ret_t fastdds__dynamic_type_builder_add_ ## FunctionT ## _bounded_sequence_member( \
    rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, \
    rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl, \
    rosidl_dynamic_typesupport_member_id_t id, const char * name, size_t name_length, \
    const char * default_value, \
    size_t default_value_length, size_t sequence_bound) \
  { \
    auto fastdds_impl = \
      static_cast<fastdds__serialization_support_impl_handle_t *>(serialization_support_impl->handle); \
 \
    auto type_builder_handle = \
      static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_type_builder_impl *>( \
      type_builder_impl-> \
      handle); \
    if (!type_builder_handle || !type_builder_handle->ref_type) \
    { \
      RCUTILS_SET_ERROR_MSG("Could not get handle to type builder impl"); \
      return RCUTILS_RET_INVALID_ARGUMENT; \
    } \
 \
    MemberDescriptor::_ref_type descriptor = \
      eprosima::fastdds::dds::traits<MemberDescriptor>::make_shared(); \
    descriptor->type( \
      fastdds_impl->type_factory_->create_sequence_type( \
        fastdds_impl->type_factory_->get_primitive_type(MemberT), \
        fastdds__size_t_to_uint32_t(sequence_bound))->build()); \
    descriptor->name(std::string(name, name_length)); \
    descriptor->id(fastdds__size_t_to_uint32_t(id)); \
    descriptor->default_value(std::string(default_value, default_value_length)); \
 \
    FASTDDS_CHECK_RET_FOR_NOT_OK_AND_RETURN_WITH_MSG( \
      type_builder_handle->ref_type->add_member(descriptor), \
      "Could not add `" #MemberT "` bounded sequence member to type builder"); \
  }

FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN(
  bool,
  eprosima::fastdds::dds::TK_BOOLEAN)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN(byte, eprosima::fastdds::dds::TK_BYTE)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN(char, eprosima::fastdds::dds::TK_CHAR8)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN(
  wchar,
  eprosima::fastdds::dds::TK_CHAR16)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN(
  float32,
  eprosima::fastdds::dds::TK_FLOAT32)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN(
  float64,
  eprosima::fastdds::dds::TK_FLOAT64)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN(int8, eprosima::fastdds::dds::TK_INT8)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN(uint8, eprosima::fastdds::dds::TK_UINT8)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN(int16, eprosima::fastdds::dds::TK_INT16)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN(
  uint16,
  eprosima::fastdds::dds::TK_UINT16)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN(int32, eprosima::fastdds::dds::TK_INT32)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN(
  uint32,
  eprosima::fastdds::dds::TK_UINT32)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN(int64, eprosima::fastdds::dds::TK_INT64)
FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN(
  uint64,
  eprosima::fastdds::dds::TK_UINT64)
#undef FASTDDS_DYNAMIC_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN

rcutils_ret_t
fastdds__dynamic_type_builder_add_string_bounded_sequence_member(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name,
  size_t name_length,
  const char * default_value,
  size_t default_value_length,
  size_t sequence_bound)
{
  return fastdds__dynamic_type_builder_add_bounded_string_bounded_sequence_member(
    serialization_support_impl, type_builder_impl, id, name, name_length, default_value,
    default_value_length,
    CONTAINER_UNLIMITED, fastdds__size_t_to_uint32_t(sequence_bound));
}

rcutils_ret_t
fastdds__dynamic_type_builder_add_wstring_bounded_sequence_member(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name,
  size_t name_length,
  const char * default_value,
  size_t default_value_length,
  size_t sequence_bound)
{
  return fastdds__dynamic_type_builder_add_bounded_wstring_bounded_sequence_member(
    serialization_support_impl, type_builder_impl, id, name, name_length, default_value,
    default_value_length,
    CONTAINER_UNLIMITED, fastdds__size_t_to_uint32_t(sequence_bound));
}

rcutils_ret_t
fastdds__dynamic_type_builder_add_fixed_string_bounded_sequence_member(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name,
  size_t name_length,
  const char * default_value,
  size_t default_value_length,
  size_t string_length,
  size_t sequence_bound)
{
  return fastdds__dynamic_type_builder_add_bounded_string_bounded_sequence_member(
    serialization_support_impl, type_builder_impl, id, name, name_length, default_value,
    default_value_length,
    string_length, fastdds__size_t_to_uint32_t(sequence_bound));
}

rcutils_ret_t
fastdds__dynamic_type_builder_add_fixed_wstring_bounded_sequence_member(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name,
  size_t name_length,
  const char * default_value,
  size_t default_value_length,
  size_t wstring_length,
  size_t sequence_bound)
{
  return fastdds__dynamic_type_builder_add_bounded_wstring_bounded_sequence_member(
    serialization_support_impl, type_builder_impl, id, name, name_length, default_value,
    default_value_length,
    wstring_length, fastdds__size_t_to_uint32_t(sequence_bound));
}

rcutils_ret_t
fastdds__dynamic_type_builder_add_bounded_string_bounded_sequence_member(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name,
  size_t name_length,
  const char * default_value,
  size_t default_value_length,
  size_t string_bound,
  size_t sequence_bound)
{
  auto fastdds_impl =
    static_cast<fastdds__serialization_support_impl_handle_t *>(serialization_support_impl->handle);

  auto type_builder_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_type_builder_impl *>(
    type_builder_impl->handle);

  if (!type_builder_handle || !type_builder_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to type builder impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  MemberDescriptor::_ref_type descriptor =
    eprosima::fastdds::dds::traits<MemberDescriptor>::make_shared();
  descriptor->type(
    fastdds_impl->type_factory_->create_sequence_type(
      fastdds_impl->type_factory_->create_string_type(
        fastdds__size_t_to_uint32_t(string_bound))->build(),
      fastdds__size_t_to_uint32_t(sequence_bound))->build());
  descriptor->name(std::string(name, name_length));
  descriptor->id(fastdds__size_t_to_uint32_t(id));
  descriptor->default_value(std::string(default_value, default_value_length));

  FASTDDS_CHECK_RET_FOR_NOT_OK_AND_RETURN_WITH_MSG(
    type_builder_handle->ref_type->add_member(descriptor),
    "Could not add bounded `string` bounded sequence member to type builder");
}

rcutils_ret_t
fastdds__dynamic_type_builder_add_bounded_wstring_bounded_sequence_member(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name,
  size_t name_length,
  const char * default_value,
  size_t default_value_length,
  size_t wstring_bound,
  size_t sequence_bound)
{
  auto fastdds_impl =
    static_cast<fastdds__serialization_support_impl_handle_t *>(serialization_support_impl->handle);

  auto type_builder_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_type_builder_impl *>(
    type_builder_impl->handle);

  if (!type_builder_handle || !type_builder_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to type builder impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  MemberDescriptor::_ref_type descriptor =
    eprosima::fastdds::dds::traits<MemberDescriptor>::make_shared();
  descriptor->type(
    fastdds_impl->type_factory_->create_sequence_type(
      fastdds_impl->type_factory_->create_wstring_type(
        fastdds__size_t_to_uint32_t(wstring_bound))->build(),
      fastdds__size_t_to_uint32_t(sequence_bound))->build());
  descriptor->name(std::string(name, name_length));
  descriptor->id(fastdds__size_t_to_uint32_t(id));
  descriptor->default_value(std::string(default_value, default_value_length));

  FASTDDS_CHECK_RET_FOR_NOT_OK_AND_RETURN_WITH_MSG(
    type_builder_handle->ref_type->add_member(descriptor),
    "Could not add bounded `wstring` bounded sequence member to type builder");
}

// DYNAMIC TYPE NESTED MEMBERS =====================================================================
rcutils_ret_t
fastdds__dynamic_type_builder_add_complex_member(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name,
  size_t name_length,
  const char * default_value,
  size_t default_value_length,
  rosidl_dynamic_typesupport_dynamic_type_impl_t * nested_struct)
{
  (void) serialization_support_impl;

  auto type_builder_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_type_builder_impl *>(
    type_builder_impl->handle);

  if (!type_builder_handle || !type_builder_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to type builder impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  auto nested_struct_type_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_type_impl *>(
    nested_struct->handle);

  if (!nested_struct_type_handle || !nested_struct_type_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to type impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  MemberDescriptor::_ref_type descriptor =
    eprosima::fastdds::dds::traits<MemberDescriptor>::make_shared();
  descriptor->type(nested_struct_type_handle->ref_type);
  descriptor->name(std::string(name, name_length));
  descriptor->id(fastdds__size_t_to_uint32_t(id));
  descriptor->default_value(std::string(default_value, default_value_length));

  FASTDDS_CHECK_RET_FOR_NOT_OK_AND_RETURN_WITH_MSG(
    type_builder_handle->ref_type->add_member(descriptor),
    "Could not add complex member to type builder");
}

rcutils_ret_t
fastdds__dynamic_type_builder_add_complex_array_member(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name,
  size_t name_length,
  const char * default_value,
  size_t default_value_length,
  rosidl_dynamic_typesupport_dynamic_type_impl_t * nested_struct,
  size_t array_length)
{
  auto fastdds_impl =
    static_cast<fastdds__serialization_support_impl_handle_t *>(serialization_support_impl->handle);

  auto type_builder_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_type_builder_impl *>(
    type_builder_impl->handle);

  if (!type_builder_handle || !type_builder_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to type builder impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  auto nested_struct_type_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_type_impl *>(
    nested_struct->handle);

  if (!nested_struct_type_handle || !nested_struct_type_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to type impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  MemberDescriptor::_ref_type descriptor =
    eprosima::fastdds::dds::traits<MemberDescriptor>::make_shared();
  descriptor->type(
    fastdds_impl->type_factory_->create_array_type(
      nested_struct_type_handle->ref_type,
      {fastdds__size_t_to_uint32_t(array_length)})->build());
  descriptor->name(std::string(name, name_length));
  descriptor->id(fastdds__size_t_to_uint32_t(id));
  descriptor->default_value(std::string(default_value, default_value_length));

  FASTDDS_CHECK_RET_FOR_NOT_OK_AND_RETURN_WITH_MSG(
    type_builder_handle->ref_type->add_member(descriptor),
    "Could not add complex array member to type builder");
}

rcutils_ret_t
fastdds__dynamic_type_builder_add_complex_unbounded_sequence_member(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name,
  size_t name_length,
  const char * default_value,
  size_t default_value_length,
  rosidl_dynamic_typesupport_dynamic_type_impl_t * nested_struct)
{
  return fastdds__dynamic_type_builder_add_complex_bounded_sequence_member(
    serialization_support_impl, type_builder_impl, id, name, name_length, default_value,
    default_value_length,
    nested_struct, CONTAINER_UNLIMITED);
}

rcutils_ret_t
fastdds__dynamic_type_builder_add_complex_bounded_sequence_member(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name,
  size_t name_length,
  const char * default_value,
  size_t default_value_length,
  rosidl_dynamic_typesupport_dynamic_type_impl_t * nested_struct,
  size_t sequence_bound)
{
  auto fastdds_impl =
    static_cast<fastdds__serialization_support_impl_handle_t *>(serialization_support_impl->handle);

  auto type_builder_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_type_builder_impl *>(
    type_builder_impl->handle);

  if (!type_builder_handle || !type_builder_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to type builder impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  auto nested_struct_type_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_type_impl *>(
    nested_struct->handle);

  if (!nested_struct_type_handle || !nested_struct_type_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to type impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  MemberDescriptor::_ref_type descriptor =
    eprosima::fastdds::dds::traits<MemberDescriptor>::make_shared();
  descriptor->type(
    fastdds_impl->type_factory_->create_sequence_type(
      nested_struct_type_handle->ref_type,
      fastdds__size_t_to_uint32_t(sequence_bound))->build());
  descriptor->name(std::string(name, name_length));
  descriptor->id(fastdds__size_t_to_uint32_t(id));
  descriptor->default_value(std::string(default_value, default_value_length));

  FASTDDS_CHECK_RET_FOR_NOT_OK_AND_RETURN_WITH_MSG(
    type_builder_handle->ref_type->add_member(descriptor),
    "Could not add complex bounded sequence member to type builder");
}

rcutils_ret_t
fastdds__dynamic_type_builder_add_complex_member_builder(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name,
  size_t name_length,
  const char * default_value,
  size_t default_value_length,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * nested_struct_builder)
{
  (void) serialization_support_impl;

  auto type_builder_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_type_builder_impl *>(
    type_builder_impl->handle);

  if (!type_builder_handle || !type_builder_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to type builder impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  auto nested_struct_type_builder_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_type_builder_impl *>(
    nested_struct_builder->handle);

  if (!nested_struct_type_builder_handle || !nested_struct_type_builder_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to type builder impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  MemberDescriptor::_ref_type descriptor =
    eprosima::fastdds::dds::traits<MemberDescriptor>::make_shared();
  descriptor->type(nested_struct_type_builder_handle->ref_type->build());
  descriptor->name(std::string(name, name_length));
  descriptor->id(fastdds__size_t_to_uint32_t(id));
  descriptor->default_value(std::string(default_value, default_value_length));

  FASTDDS_CHECK_RET_FOR_NOT_OK_AND_RETURN_WITH_MSG(
    type_builder_handle->ref_type->add_member(descriptor),
    "Could not add complex member to type builder (via builder)");
}

rcutils_ret_t
fastdds__dynamic_type_builder_add_complex_array_member_builder(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name,
  size_t name_length,
  const char * default_value,
  size_t default_value_length,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * nested_struct_builder,
  size_t array_length)
{
  auto fastdds_impl =
    static_cast<fastdds__serialization_support_impl_handle_t *>(serialization_support_impl->handle);

  auto type_builder_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_type_builder_impl *>(
    type_builder_impl->handle);

  if (!type_builder_handle || !type_builder_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to type builder impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  auto nested_struct_type_builder_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_type_builder_impl *>(
    nested_struct_builder->handle);

  if (!nested_struct_type_builder_handle || !nested_struct_type_builder_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to type builder impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  MemberDescriptor::_ref_type descriptor =
    eprosima::fastdds::dds::traits<MemberDescriptor>::make_shared();
  descriptor->type(
    fastdds_impl->type_factory_->create_array_type(
      nested_struct_type_builder_handle->ref_type->build(),
      {fastdds__size_t_to_uint32_t(array_length)})->build());
  descriptor->name(std::string(name, name_length));
  descriptor->id(fastdds__size_t_to_uint32_t(id));
  descriptor->default_value(std::string(default_value, default_value_length));

  FASTDDS_CHECK_RET_FOR_NOT_OK_AND_RETURN_WITH_MSG(
    type_builder_handle->ref_type->add_member(descriptor),
    "Could not add complex array member to type builder (via builder)");
}

rcutils_ret_t
fastdds__dynamic_type_builder_add_complex_unbounded_sequence_member_builder(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name,
  size_t name_length,
  const char * default_value,
  size_t default_value_length,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * nested_struct_builder)
{
  return fastdds__dynamic_type_builder_add_complex_bounded_sequence_member_builder(
    serialization_support_impl, type_builder_impl, id, name, name_length, default_value,
    default_value_length,
    nested_struct_builder, CONTAINER_UNLIMITED);
}

rcutils_ret_t
fastdds__dynamic_type_builder_add_complex_bounded_sequence_member_builder(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name,
  size_t name_length,
  const char * default_value,
  size_t default_value_length,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * nested_struct_builder,
  size_t sequence_bound)
{
  auto fastdds_impl =
    static_cast<fastdds__serialization_support_impl_handle_t *>(serialization_support_impl->handle);

  auto type_builder_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_type_builder_impl *>(
    type_builder_impl->handle);

  if (!type_builder_handle || !type_builder_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to type builder impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  auto nested_struct_type_builder_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_type_builder_impl *>(
    nested_struct_builder->handle);

  if (!nested_struct_type_builder_handle || !nested_struct_type_builder_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to type builder impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  MemberDescriptor::_ref_type descriptor =
    eprosima::fastdds::dds::traits<MemberDescriptor>::make_shared();
  descriptor->type(
    fastdds_impl->type_factory_->create_sequence_type(
      nested_struct_type_builder_handle->ref_type->build(),
      fastdds__size_t_to_uint32_t(sequence_bound))->build());
  descriptor->name(std::string(name, name_length));
  descriptor->id(fastdds__size_t_to_uint32_t(id));
  descriptor->default_value(std::string(default_value, default_value_length));

  FASTDDS_CHECK_RET_FOR_NOT_OK_AND_RETURN_WITH_MSG(
    type_builder_handle->ref_type->add_member(descriptor),
    "Could not add complex bounded sequence member to type builder");
}
