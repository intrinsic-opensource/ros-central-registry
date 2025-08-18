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

#include "rosidl_dynamic_typesupport/api/dynamic_type.h"

#include <assert.h>
#include <stdlib.h>

#include <rcutils/error_handling.h>
#include <rcutils/logging_macros.h>
#include <rcutils/types/rcutils_ret.h>

#include <rosidl_runtime_c/type_description/field__functions.h>
#include <rosidl_runtime_c/type_description/field__struct.h>
#include <rosidl_runtime_c/type_description/field_type__functions.h>
#include <rosidl_runtime_c/type_description/field_type__struct.h>
#include <rosidl_runtime_c/type_description/individual_type_description__functions.h>
#include <rosidl_runtime_c/type_description/individual_type_description__struct.h>
#include <rosidl_runtime_c/type_description/type_description__functions.h>
#include <rosidl_runtime_c/type_description/type_description__struct.h>
#include <rosidl_runtime_c/type_description_utils.h>

#include "rosidl_dynamic_typesupport/api/serialization_support.h"
#include "rosidl_dynamic_typesupport/macros.h"
#include "rosidl_dynamic_typesupport/types.h"


// =================================================================================================
// DYNAMIC TYPE
// =================================================================================================
rosidl_dynamic_typesupport_dynamic_type_builder_impl_t
rosidl_dynamic_typesupport_get_zero_initialized_dynamic_type_builder_impl(void)
{
  static rosidl_dynamic_typesupport_dynamic_type_builder_impl_t zero_dynamic_type_builder_impl = {
    // .allocator  = // Initialized later
    .handle = NULL
  };
  zero_dynamic_type_builder_impl.allocator = rcutils_get_zero_initialized_allocator();
  return zero_dynamic_type_builder_impl;
}

rosidl_dynamic_typesupport_dynamic_type_builder_t
rosidl_dynamic_typesupport_get_zero_initialized_dynamic_type_builder(void)
{
  static rosidl_dynamic_typesupport_dynamic_type_builder_t zero_dynamic_type_builder = {
    // .allocator  = // Initialized later
    // .impl  = // Initialized later
    .serialization_support = NULL
  };
  zero_dynamic_type_builder.allocator = rcutils_get_zero_initialized_allocator();
  zero_dynamic_type_builder.impl =
    rosidl_dynamic_typesupport_get_zero_initialized_dynamic_type_builder_impl();
  return zero_dynamic_type_builder;
}

rosidl_dynamic_typesupport_dynamic_type_impl_t
rosidl_dynamic_typesupport_get_zero_initialized_dynamic_type_impl(void)
{
  static rosidl_dynamic_typesupport_dynamic_type_impl_t zero_dynamic_type_impl = {
    // .allocator  = // Initialized later
    .handle = NULL
  };
  zero_dynamic_type_impl.allocator = rcutils_get_zero_initialized_allocator();
  return zero_dynamic_type_impl;
}

rosidl_dynamic_typesupport_dynamic_type_t
rosidl_dynamic_typesupport_get_zero_initialized_dynamic_type(void)
{
  static rosidl_dynamic_typesupport_dynamic_type_t zero_dynamic_type = {
    // .allocator  = // Initialized later
    // .impl  = // Initialized later
    .serialization_support = NULL
  };
  zero_dynamic_type.allocator = rcutils_get_zero_initialized_allocator();
  zero_dynamic_type.impl =
    rosidl_dynamic_typesupport_get_zero_initialized_dynamic_type_impl();
  return zero_dynamic_type;
}


// DYNAMIC TYPE UTILS ==============================================================================
rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_equals(
  const rosidl_dynamic_typesupport_dynamic_type_t * dynamic_type,
  const rosidl_dynamic_typesupport_dynamic_type_t * other,
  bool * equals)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(other, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(equals, RCUTILS_RET_INVALID_ARGUMENT);

  if (dynamic_type->serialization_support->serialization_library_identifier !=
    other->serialization_support->serialization_library_identifier)
  {
    RCUTILS_SET_ERROR_MSG("Library identifiers for dynamic types do not match");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }
  return (dynamic_type->serialization_support->methods.dynamic_type_equals)(
    &dynamic_type->serialization_support->impl, &dynamic_type->impl, &other->impl, equals);
}


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_get_member_count(
  const rosidl_dynamic_typesupport_dynamic_type_t * dynamic_type, size_t * member_count)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(member_count, RCUTILS_RET_INVALID_ARGUMENT);
  return (dynamic_type->serialization_support->methods.dynamic_type_get_member_count)(
    &dynamic_type->serialization_support->impl, &dynamic_type->impl, member_count);
}


// DYNAMIC TYPE CONSTRUCTION =======================================================================
rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_builder_init(
  rosidl_dynamic_typesupport_serialization_support_t * serialization_support,
  const char * name, size_t name_length,
  rcutils_allocator_t * allocator,
  rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(serialization_support, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(allocator, RCUTILS_RET_INVALID_ARGUMENT);
  if (!rcutils_allocator_is_valid(allocator)) {
    RCUTILS_SET_ERROR_MSG("allocator is invalid");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT);

  if (dynamic_type_builder->impl.handle != NULL) {
    ROSIDL_DYNAMIC_TYPESUPPORT_CHECK_RET_FOR_NOT_OK(
      rosidl_dynamic_typesupport_dynamic_type_builder_fini(dynamic_type_builder)
    );
  }

  dynamic_type_builder->serialization_support = serialization_support;
  dynamic_type_builder->allocator = *allocator;
  ROSIDL_DYNAMIC_TYPESUPPORT_CHECK_RET_FOR_NOT_OK_WITH_CLEANUP(
    (serialization_support->methods.dynamic_type_builder_init)(
      &serialization_support->impl, name, name_length, allocator, &dynamic_type_builder->impl),
    rosidl_dynamic_typesupport_dynamic_type_builder_fini(dynamic_type_builder)    // Cleanup
  );
  return RCUTILS_RET_OK;
}


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_builder_clone(
  const rosidl_dynamic_typesupport_dynamic_type_builder_t * other,
  rcutils_allocator_t * allocator,
  rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(other, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(allocator, RCUTILS_RET_INVALID_ARGUMENT);
  if (!rcutils_allocator_is_valid(allocator)) {
    RCUTILS_SET_ERROR_MSG("allocator is invalid");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT);

  if (dynamic_type_builder->impl.handle != NULL) {
    ROSIDL_DYNAMIC_TYPESUPPORT_CHECK_RET_FOR_NOT_OK(
      rosidl_dynamic_typesupport_dynamic_type_builder_fini(dynamic_type_builder)
    );
  }

  dynamic_type_builder->serialization_support = other->serialization_support;
  dynamic_type_builder->allocator = *allocator;
  ROSIDL_DYNAMIC_TYPESUPPORT_CHECK_RET_FOR_NOT_OK_WITH_CLEANUP(
    (other->serialization_support->methods.dynamic_type_builder_clone)(
      &other->serialization_support->impl, &other->impl, allocator, &dynamic_type_builder->impl),
    rosidl_dynamic_typesupport_dynamic_type_builder_fini(dynamic_type_builder) // Cleanup
  );
  return RCUTILS_RET_OK;
}


static rcutils_ret_t
handle_nested_type(
  rosidl_dynamic_typesupport_serialization_support_t * serialization_support,
  const rosidl_runtime_c__type_description__TypeDescription * description,
  rcutils_allocator_t * allocator,
  rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder,
  rosidl_runtime_c__type_description__Field * field,
  size_t field_offset)
{
  if (field->type.nested_type_name.data == NULL) {
    RCUTILS_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "Nested type name from nested field [%s]", field->name.data);
    return RCUTILS_RET_ERROR;
  }

  // Create a new type description to pass to the next layer
  rosidl_runtime_c__type_description__TypeDescription * recurse_desc = NULL;
  rosidl_runtime_c__type_description__IndividualTypeDescription * recurse_indiv_desc = NULL;

  // NOTE(methylDragon): recurse_indiv_desc borrows from
  //                     description->referenced_type_descriptions.
  //                     It is NOT a copy!! Do NOT finalize, modify, or delete it!
  rcutils_ret_t ret = rosidl_runtime_c_type_description_utils_find_referenced_type_description(
    &description->referenced_type_descriptions,
    field->type.nested_type_name.data,
    &recurse_indiv_desc);
  if (ret != RCUTILS_RET_OK || recurse_indiv_desc == NULL) {
    RCUTILS_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "Could not find referenced type description [%s]", field->type.nested_type_name.data);
    return ret;
  }

  ret = rosidl_runtime_c_type_description_utils_get_referenced_type_description_as_type_description(
    &description->referenced_type_descriptions, recurse_indiv_desc, &recurse_desc,
    true);  // Coerce to valid
  if (ret != RCUTILS_RET_OK || recurse_desc == NULL) {
    RCUTILS_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "Could not get referenced type description as full type description [%s]",
      field->type.nested_type_name.data);
    if (ret == RCUTILS_RET_OK) {
      ret = RCUTILS_RET_ERROR;
    }
    return ret;
  }

  // Recurse
  rosidl_dynamic_typesupport_dynamic_type_builder_t nested_type_builder =
    rosidl_dynamic_typesupport_get_zero_initialized_dynamic_type_builder();
  nested_type_builder.serialization_support = serialization_support;
  nested_type_builder.allocator = *allocator;

  ret = rosidl_dynamic_typesupport_dynamic_type_builder_init_from_description(
    serialization_support, recurse_desc, allocator, &nested_type_builder);
  rosidl_runtime_c__type_description__TypeDescription__destroy(recurse_desc);

  if (ret != RCUTILS_RET_OK) {
    RCUTILS_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "Could not construct nested type builder for field [%s]", field->name.data);
    if (ret == RCUTILS_RET_OK) {
      ret = RCUTILS_RET_ERROR;
    }
    return ret;
  }

  switch (field->type.type_id) {
    case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_NESTED_TYPE:
      ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_complex_member_builder(
        dynamic_type_builder, field_offset,
        field->name.data, field->name.size,
        field->default_value.data, field->default_value.size,
        &nested_type_builder);
      break;

    case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_NESTED_TYPE_ARRAY:
      ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_complex_array_member_builder(
        dynamic_type_builder, field_offset,
        field->name.data, field->name.size,
        field->default_value.data, field->default_value.size,
        &nested_type_builder, field->type.capacity);
      break;

    case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE:
      ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_complex_unbounded_sequence_member_builder(  // NOLINT
        dynamic_type_builder, field_offset,
        field->name.data, field->name.size,
        field->default_value.data, field->default_value.size,
        &nested_type_builder);
      break;

    case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE:
      ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_complex_bounded_sequence_member_builder(
        dynamic_type_builder, field_offset,
        field->name.data, field->name.size,
        field->default_value.data, field->default_value.size,
        &nested_type_builder, field->type.capacity);
      break;

    default:
      RCUTILS_SET_ERROR_MSG_WITH_FORMAT_STRING(
        "Invalid field type id: %d !", field->type.type_id);
      ret = RCUTILS_RET_INVALID_ARGUMENT;
      break;
  }
  rosidl_dynamic_typesupport_dynamic_type_builder_fini(&nested_type_builder);
  return ret;
}


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_builder_init_from_description(
  rosidl_dynamic_typesupport_serialization_support_t * serialization_support,
  const rosidl_runtime_c__type_description__TypeDescription * description,
  rcutils_allocator_t * allocator,
  rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(serialization_support, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(description, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(allocator, RCUTILS_RET_INVALID_ARGUMENT);
  if (!rcutils_allocator_is_valid(allocator)) {
    RCUTILS_SET_ERROR_MSG("allocator is invalid");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT);

  if (rosidl_runtime_c_type_description_utils_type_description_is_valid(description) !=
    RCUTILS_RET_OK)
  {
    rcutils_error_string_t error_string = rcutils_get_error_string();
    rcutils_reset_error();
    RCUTILS_SET_ERROR_MSG_WITH_FORMAT_STRING("Type description is not valid: %s", error_string.str);
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  // NOTE(methylDragon): This was a potential place to do string replacements for type descriptions
  //                     from "/" delimiters to "::" delimiters to support DDS IDL names.
  //
  //                     But since this is meant to be middleware agnostic (potentially supporting
  //                     non-DDS middlewares), replacements should happen in the support libraries
  //                     on an as-needed basis.

  if (dynamic_type_builder->impl.handle != NULL) {
    ROSIDL_DYNAMIC_TYPESUPPORT_CHECK_RET_FOR_NOT_OK(
      rosidl_dynamic_typesupport_dynamic_type_builder_fini(dynamic_type_builder)
    );
  }

  const rosidl_runtime_c__type_description__IndividualTypeDescription * main_description =
    &description->type_description;
  ROSIDL_DYNAMIC_TYPESUPPORT_CHECK_RET_FOR_NOT_OK(
    rosidl_dynamic_typesupport_dynamic_type_builder_init(
      serialization_support,
      main_description->type_name.data,
      main_description->type_name.size,
      allocator,
      dynamic_type_builder)
  );

  rcutils_ret_t ret = RCUTILS_RET_ERROR;

  for (size_t i = 0; i < main_description->fields.size; i++) {
    rosidl_runtime_c__type_description__Field * field = &main_description->fields.data[i];

    switch (field->type.type_id) {
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_NOT_SET:
        RCUTILS_SET_ERROR_MSG_WITH_FORMAT_STRING(
          "Field type id not set for field [%s]", field->name.data);
        ret = RCUTILS_RET_ERROR;
        goto fail;

      // PRIMITIVES
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_BOOLEAN:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_bool_member(
          dynamic_type_builder, i, field->name.data, field->name.size,
          field->default_value.data, field->default_value.size);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_BYTE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_byte_member(
          dynamic_type_builder, i, field->name.data, field->name.size,
          field->default_value.data, field->default_value.size);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_CHAR:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_char_member(
          dynamic_type_builder, i, field->name.data, field->name.size,
          field->default_value.data, field->default_value.size);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_FLOAT32:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_float32_member(
          dynamic_type_builder, i, field->name.data, field->name.size,
          field->default_value.data, field->default_value.size);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_FLOAT64:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_float64_member(
          dynamic_type_builder, i, field->name.data, field->name.size,
          field->default_value.data, field->default_value.size);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_INT8:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_int8_member(
          dynamic_type_builder, i, field->name.data, field->name.size,
          field->default_value.data, field->default_value.size);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_UINT8:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_uint8_member(
          dynamic_type_builder, i, field->name.data, field->name.size,
          field->default_value.data, field->default_value.size);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_INT16:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_int16_member(
          dynamic_type_builder, i, field->name.data, field->name.size,
          field->default_value.data, field->default_value.size);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_UINT16:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_uint16_member(
          dynamic_type_builder, i, field->name.data, field->name.size,
          field->default_value.data, field->default_value.size);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_INT32:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_int32_member(
          dynamic_type_builder, i, field->name.data, field->name.size,
          field->default_value.data, field->default_value.size);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_UINT32:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_uint32_member(
          dynamic_type_builder, i, field->name.data, field->name.size,
          field->default_value.data, field->default_value.size);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_INT64:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_int64_member(
          dynamic_type_builder, i, field->name.data, field->name.size,
          field->default_value.data, field->default_value.size);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_UINT64:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_uint64_member(
          dynamic_type_builder, i, field->name.data, field->name.size,
          field->default_value.data, field->default_value.size);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_STRING:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_string_member(
          dynamic_type_builder, i, field->name.data, field->name.size,
          field->default_value.data, field->default_value.size);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_WSTRING:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_wstring_member(
          dynamic_type_builder, i, field->name.data, field->name.size,
          field->default_value.data, field->default_value.size);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_FIXED_STRING:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_fixed_string_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_FIXED_WSTRING:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_fixed_wstring_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_BOUNDED_STRING:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_bounded_string_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_BOUNDED_WSTRING:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_bounded_wstring_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.capacity);
        break;

      // STATIC ARRAYS
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_BOOLEAN_ARRAY:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_bool_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_BYTE_ARRAY:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_byte_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_CHAR_ARRAY:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_char_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_FLOAT32_ARRAY:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_float32_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_FLOAT64_ARRAY:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_float64_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_INT8_ARRAY:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_int8_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_UINT8_ARRAY:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_uint8_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_INT16_ARRAY:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_int16_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_UINT16_ARRAY:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_uint16_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_INT32_ARRAY:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_int32_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_UINT32_ARRAY:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_uint32_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_INT64_ARRAY:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_int64_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_UINT64_ARRAY:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_uint64_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_STRING_ARRAY:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_string_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_WSTRING_ARRAY:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_wstring_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_FIXED_STRING_ARRAY:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_fixed_string_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.string_capacity, field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_FIXED_WSTRING_ARRAY:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_fixed_wstring_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.string_capacity, field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_BOUNDED_STRING_ARRAY:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_bounded_string_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.string_capacity, field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_BOUNDED_WSTRING_ARRAY:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_bounded_wstring_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.string_capacity, field->type.capacity);
        break;

      // UNBOUNDED SEQUENCES
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_BOOLEAN_UNBOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_bool_unbounded_sequence_member(
          dynamic_type_builder, i, field->name.data, field->name.size,
          field->default_value.data, field->default_value.size);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_BYTE_UNBOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_byte_unbounded_sequence_member(
          dynamic_type_builder, i, field->name.data, field->name.size,
          field->default_value.data, field->default_value.size);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_CHAR_UNBOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_char_unbounded_sequence_member(
          dynamic_type_builder, i, field->name.data, field->name.size,
          field->default_value.data, field->default_value.size);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_FLOAT32_UNBOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_float32_unbounded_sequence_member(
          dynamic_type_builder, i, field->name.data, field->name.size,
          field->default_value.data, field->default_value.size);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_FLOAT64_UNBOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_float64_unbounded_sequence_member(
          dynamic_type_builder, i, field->name.data, field->name.size,
          field->default_value.data, field->default_value.size);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_INT8_UNBOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_int8_unbounded_sequence_member(
          dynamic_type_builder, i, field->name.data, field->name.size,
          field->default_value.data, field->default_value.size);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_UINT8_UNBOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_uint8_unbounded_sequence_member(
          dynamic_type_builder, i, field->name.data, field->name.size,
          field->default_value.data, field->default_value.size);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_INT16_UNBOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_int16_unbounded_sequence_member(
          dynamic_type_builder, i, field->name.data, field->name.size,
          field->default_value.data, field->default_value.size);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_UINT16_UNBOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_uint16_unbounded_sequence_member(
          dynamic_type_builder, i, field->name.data, field->name.size,
          field->default_value.data, field->default_value.size);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_INT32_UNBOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_int32_unbounded_sequence_member(
          dynamic_type_builder, i, field->name.data, field->name.size,
          field->default_value.data, field->default_value.size);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_UINT32_UNBOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_uint32_unbounded_sequence_member(
          dynamic_type_builder, i, field->name.data, field->name.size,
          field->default_value.data, field->default_value.size);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_INT64_UNBOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_int64_unbounded_sequence_member(
          dynamic_type_builder, i, field->name.data, field->name.size,
          field->default_value.data, field->default_value.size);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_UINT64_UNBOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_uint64_unbounded_sequence_member(
          dynamic_type_builder, i, field->name.data, field->name.size,
          field->default_value.data, field->default_value.size);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_STRING_UNBOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_string_unbounded_sequence_member(
          dynamic_type_builder, i, field->name.data, field->name.size,
          field->default_value.data, field->default_value.size);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_WSTRING_UNBOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_wstring_unbounded_sequence_member(
          dynamic_type_builder, i, field->name.data, field->name.size,
          field->default_value.data, field->default_value.size);
        break;
        /* *INDENT-OFF* */
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_FIXED_STRING_UNBOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_fixed_string_unbounded_sequence_member(  // NOLINT
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.string_capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_FIXED_WSTRING_UNBOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_fixed_wstring_unbounded_sequence_member(  // NOLINT
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.string_capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_BOUNDED_STRING_UNBOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_bounded_string_unbounded_sequence_member(  // NOLINT
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.string_capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_BOUNDED_WSTRING_UNBOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_bounded_wstring_unbounded_sequence_member(  // NOLINT
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.string_capacity);
        break;
        /* *INDENT-ON* */

      // BOUNDED SEQUENCES
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_BOOLEAN_BOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_bool_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_BYTE_BOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_byte_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_CHAR_BOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_char_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_FLOAT32_BOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_float32_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_FLOAT64_BOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_float64_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_INT8_BOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_int8_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_UINT8_BOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_uint8_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_INT16_BOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_int16_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_UINT16_BOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_uint16_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_INT32_BOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_int32_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_UINT32_BOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_uint32_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_INT64_BOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_int64_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_UINT64_BOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_uint64_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_STRING_BOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_string_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_WSTRING_BOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_wstring_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_FIXED_STRING_BOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_fixed_string_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.string_capacity, field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_FIXED_WSTRING_BOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_fixed_wstring_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.string_capacity, field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_BOUNDED_STRING_BOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_bounded_string_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.string_capacity, field->type.capacity);
        break;
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_BOUNDED_WSTRING_BOUNDED_SEQUENCE:
        ret = rosidl_dynamic_typesupport_dynamic_type_builder_add_bounded_wstring_array_member(
          dynamic_type_builder, i,
          field->name.data, field->name.size,
          field->default_value.data, field->default_value.size,
          field->type.string_capacity, field->type.capacity);
        break;

      // NESTED
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_NESTED_TYPE:
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_NESTED_TYPE_ARRAY:
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_NESTED_TYPE_UNBOUNDED_SEQUENCE:
      case ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE:
        ret = handle_nested_type(serialization_support, description, allocator, dynamic_type_builder, field, i);
        if (ret != RCUTILS_RET_OK) {
          goto fail;  // error already set
        }
        break;

      default:
        RCUTILS_SET_ERROR_MSG_WITH_FORMAT_STRING(
          "Invalid field type id: %d !", field->type.type_id);
        ret = RCUTILS_RET_INVALID_ARGUMENT;
        break;
    }  // end nested case

    if (ret != RCUTILS_RET_OK) {
      goto fail;
    }
  }  // looping over fields

  return RCUTILS_RET_OK;

fail:
  if (rosidl_dynamic_typesupport_dynamic_type_builder_fini(dynamic_type_builder) !=
    RCUTILS_RET_OK)
  {
    RCUTILS_SAFE_FWRITE_TO_STDERR(
      "While handling another error, could not fini dynamic type builder");
  }
  return ret;
}  // NOLINT(readability/fn_size)


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_builder_fini(
  rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT);
  ROSIDL_DYNAMIC_TYPESUPPORT_CHECK_RET_FOR_NOT_OK(
    (dynamic_type_builder->serialization_support->methods.dynamic_type_builder_fini)(
      &dynamic_type_builder->serialization_support->impl, &dynamic_type_builder->impl)
  );
  return RCUTILS_RET_OK;
}


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_builder_destroy(
  rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT);
  rcutils_allocator_t allocator = dynamic_type_builder->allocator;

  ROSIDL_DYNAMIC_TYPESUPPORT_CHECK_RET_FOR_NOT_OK(
    rosidl_dynamic_typesupport_dynamic_type_builder_fini(dynamic_type_builder));
  allocator.deallocate(dynamic_type_builder, allocator.state);
  return RCUTILS_RET_OK;
}


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_init_from_dynamic_type_builder(
  rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder,
  rcutils_allocator_t * allocator,
  rosidl_dynamic_typesupport_dynamic_type_t * dynamic_type)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(allocator, RCUTILS_RET_INVALID_ARGUMENT);
  if (!rcutils_allocator_is_valid(allocator)) {
    RCUTILS_SET_ERROR_MSG("allocator is invalid");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type, RCUTILS_RET_INVALID_ARGUMENT);

  if (dynamic_type->impl.handle != NULL) {
    ROSIDL_DYNAMIC_TYPESUPPORT_CHECK_RET_FOR_NOT_OK(
      rosidl_dynamic_typesupport_dynamic_type_fini(dynamic_type)
    );
  }

  dynamic_type->serialization_support = dynamic_type_builder->serialization_support;
  dynamic_type->allocator = *allocator;
  ROSIDL_DYNAMIC_TYPESUPPORT_CHECK_RET_FOR_NOT_OK_WITH_CLEANUP(
    (dynamic_type_builder->serialization_support->methods
    .dynamic_type_init_from_dynamic_type_builder)(
      &dynamic_type_builder->serialization_support->impl,
      &dynamic_type_builder->impl,
      allocator,
      &dynamic_type->impl),
    rosidl_dynamic_typesupport_dynamic_type_fini(dynamic_type) // Cleanup
  );
  return RCUTILS_RET_OK;
}


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_init_from_description(
  rosidl_dynamic_typesupport_serialization_support_t * serialization_support,
  const rosidl_runtime_c__type_description__TypeDescription * description,
  rcutils_allocator_t * allocator,
  rosidl_dynamic_typesupport_dynamic_type_t * dynamic_type)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(serialization_support, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(description, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(allocator, RCUTILS_RET_INVALID_ARGUMENT);
  if (!rcutils_allocator_is_valid(allocator)) {
    RCUTILS_SET_ERROR_MSG("allocator is invalid");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type, RCUTILS_RET_INVALID_ARGUMENT);

  if (dynamic_type->impl.handle != NULL) {
    ROSIDL_DYNAMIC_TYPESUPPORT_CHECK_RET_FOR_NOT_OK(
      rosidl_dynamic_typesupport_dynamic_type_fini(dynamic_type)
    );
  }
  dynamic_type->serialization_support = serialization_support;
  dynamic_type->allocator = *allocator;

  rosidl_dynamic_typesupport_dynamic_type_builder_t builder =
    rosidl_dynamic_typesupport_get_zero_initialized_dynamic_type_builder();
  builder.serialization_support = serialization_support;
  builder.allocator = *allocator;

  ROSIDL_DYNAMIC_TYPESUPPORT_CHECK_RET_FOR_NOT_OK(
    rosidl_dynamic_typesupport_dynamic_type_builder_init_from_description(
      serialization_support, description, allocator, &builder)
  );

  ROSIDL_DYNAMIC_TYPESUPPORT_CHECK_RET_FOR_NOT_OK_WITH_CLEANUP(
    rosidl_dynamic_typesupport_dynamic_type_init_from_dynamic_type_builder(
      &builder, allocator, dynamic_type),
    rosidl_dynamic_typesupport_dynamic_type_builder_fini(&builder)  // Cleanup
  );
  rosidl_dynamic_typesupport_dynamic_type_builder_fini(&builder);
  return RCUTILS_RET_OK;
}


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_clone(
  const rosidl_dynamic_typesupport_dynamic_type_t * other,
  rcutils_allocator_t * allocator,
  rosidl_dynamic_typesupport_dynamic_type_t * dynamic_type)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(other, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(allocator, RCUTILS_RET_INVALID_ARGUMENT);
  if (!rcutils_allocator_is_valid(allocator)) {
    RCUTILS_SET_ERROR_MSG("allocator is invalid");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type, RCUTILS_RET_INVALID_ARGUMENT);

  if (dynamic_type->impl.handle != NULL) {
    ROSIDL_DYNAMIC_TYPESUPPORT_CHECK_RET_FOR_NOT_OK(
      rosidl_dynamic_typesupport_dynamic_type_fini(dynamic_type)
    );
  }

  dynamic_type->serialization_support = other->serialization_support;
  dynamic_type->allocator = *allocator;
  ROSIDL_DYNAMIC_TYPESUPPORT_CHECK_RET_FOR_NOT_OK_WITH_CLEANUP(
    (other->serialization_support->methods.dynamic_type_clone)(
      &other->serialization_support->impl, &other->impl, allocator, &dynamic_type->impl),
    rosidl_dynamic_typesupport_dynamic_type_fini(dynamic_type) // Cleanup
  );
  return RCUTILS_RET_OK;
}


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_fini(
  rosidl_dynamic_typesupport_dynamic_type_t * dynamic_type)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type, RCUTILS_RET_INVALID_ARGUMENT);
  ROSIDL_DYNAMIC_TYPESUPPORT_CHECK_RET_FOR_NOT_OK(
    (dynamic_type->serialization_support->methods.dynamic_type_fini)(
      &dynamic_type->serialization_support->impl, &dynamic_type->impl);
  );
  return RCUTILS_RET_OK;
}


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_destroy(
  rosidl_dynamic_typesupport_dynamic_type_t * dynamic_type)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type, RCUTILS_RET_INVALID_ARGUMENT);
  rcutils_allocator_t allocator = dynamic_type->allocator;

  ROSIDL_DYNAMIC_TYPESUPPORT_CHECK_RET_FOR_NOT_OK(
    rosidl_dynamic_typesupport_dynamic_type_fini(dynamic_type));
  allocator.deallocate(dynamic_type, allocator.state);
  return RCUTILS_RET_OK;
}


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_get_name(
  const rosidl_dynamic_typesupport_dynamic_type_t * dynamic_type,
  const char ** name,
  size_t * name_length)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, RCUTILS_RET_INVALID_ARGUMENT);
  return (dynamic_type->serialization_support->methods.dynamic_type_get_name)(
    &dynamic_type->serialization_support->impl, &dynamic_type->impl, name, name_length);
}


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_builder_get_name(
  const rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder,
  const char ** name,
  size_t * name_length)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, RCUTILS_RET_INVALID_ARGUMENT);
  return (dynamic_type_builder->serialization_support->methods.dynamic_type_builder_get_name)(
    &dynamic_type_builder->serialization_support->impl, &dynamic_type_builder->impl,
    name, name_length);
}


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_builder_set_name(
  rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder,
  const char * name, size_t name_length)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, RCUTILS_RET_INVALID_ARGUMENT);
  return (dynamic_type_builder->serialization_support->methods.dynamic_type_builder_set_name)(
    &dynamic_type_builder->serialization_support->impl, &dynamic_type_builder->impl,
    name, name_length);
}


// DYNAMIC TYPE PRIMITIVE MEMBERS ==================================================================
#define ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_MEMBER_FN(FunctionT) \
  rcutils_ret_t \
  rosidl_dynamic_typesupport_dynamic_type_builder_add_ ## FunctionT ## _member( \
    rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder, \
    rosidl_dynamic_typesupport_member_id_t id, \
    const char * name, size_t name_length, \
    const char * default_value, size_t default_value_length) \
  { \
    RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT); \
    RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, RCUTILS_RET_INVALID_ARGUMENT); \
    RCUTILS_CHECK_ARGUMENT_FOR_NULL(default_value, RCUTILS_RET_INVALID_ARGUMENT); \
    return (dynamic_type_builder->serialization_support->methods \
           .dynamic_type_builder_add_ ## FunctionT ## _member)( \
      &dynamic_type_builder->serialization_support->impl, &dynamic_type_builder->impl, id, \
      name, name_length, \
      default_value, default_value_length); \
  }


ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_MEMBER_FN(bool)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_MEMBER_FN(byte)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_MEMBER_FN(char)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_MEMBER_FN(wchar)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_MEMBER_FN(float32)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_MEMBER_FN(float64)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_MEMBER_FN(int8)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_MEMBER_FN(uint8)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_MEMBER_FN(int16)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_MEMBER_FN(uint16)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_MEMBER_FN(int32)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_MEMBER_FN(uint32)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_MEMBER_FN(int64)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_MEMBER_FN(uint64)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_MEMBER_FN(string)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_MEMBER_FN(wstring)
#undef ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_MEMBER_FN


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_builder_add_fixed_string_member(
  rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name, size_t name_length,
  const char * default_value, size_t default_value_length,
  size_t string_length)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(default_value, RCUTILS_RET_INVALID_ARGUMENT);
  return (dynamic_type_builder->serialization_support->methods.
         dynamic_type_builder_add_fixed_string_member)(
    &dynamic_type_builder->serialization_support->impl,
    &dynamic_type_builder->impl,
    id,
    name, name_length,
    default_value, default_value_length,
    string_length);
}


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_builder_add_fixed_wstring_member(
  rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name, size_t name_length,
  const char * default_value, size_t default_value_length,
  size_t wstring_length)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(default_value, RCUTILS_RET_INVALID_ARGUMENT);
  return (dynamic_type_builder->serialization_support->methods.
         dynamic_type_builder_add_fixed_wstring_member)(
    &dynamic_type_builder->serialization_support->impl,
    &dynamic_type_builder->impl,
    id,
    name, name_length,
    default_value, default_value_length,
    wstring_length);
}


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_builder_add_bounded_string_member(
  rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name, size_t name_length,
  const char * default_value, size_t default_value_length,
  size_t string_sequence_bound)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(default_value, RCUTILS_RET_INVALID_ARGUMENT);
  return (dynamic_type_builder->serialization_support->methods.
         dynamic_type_builder_add_bounded_string_member)(
    &dynamic_type_builder->serialization_support->impl,
    &dynamic_type_builder->impl,
    id,
    name, name_length,
    default_value, default_value_length,
    string_sequence_bound);
}


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_builder_add_bounded_wstring_member(
  rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name, size_t name_length,
  const char * default_value, size_t default_value_length,
  size_t wstring_sequence_bound)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(default_value, RCUTILS_RET_INVALID_ARGUMENT);
  return (dynamic_type_builder->serialization_support->methods.
         dynamic_type_builder_add_bounded_wstring_member)(
    &dynamic_type_builder->serialization_support->impl,
    &dynamic_type_builder->impl,
    id,
    name, name_length,
    default_value, default_value_length,
    wstring_sequence_bound);
}


// DYNAMIC TYPE STATIC ARRAY MEMBERS ===============================================================
#define ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN(FunctionT) \
  rcutils_ret_t \
  rosidl_dynamic_typesupport_dynamic_type_builder_add_ ## FunctionT ## _array_member( \
    rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder, \
    rosidl_dynamic_typesupport_member_id_t id, \
    const char * name, size_t name_length, \
    const char * default_value, size_t default_value_length, \
    size_t array_length) \
  { \
    RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT); \
    RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, RCUTILS_RET_INVALID_ARGUMENT); \
    RCUTILS_CHECK_ARGUMENT_FOR_NULL(default_value, RCUTILS_RET_INVALID_ARGUMENT); \
    return (dynamic_type_builder->serialization_support->methods. \
           dynamic_type_builder_add_ ## FunctionT ## _array_member)( \
      &dynamic_type_builder->serialization_support->impl, &dynamic_type_builder->impl, \
      id, \
      name, name_length, \
      default_value, default_value_length, \
      array_length); \
  }


ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN(bool)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN(byte)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN(char)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN(wchar)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN(float32)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN(float64)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN(int8)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN(uint8)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN(int16)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN(uint16)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN(int32)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN(uint32)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN(int64)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN(uint64)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN(string)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN(wstring)
#undef ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_ARRAY_MEMBER_FN


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_builder_add_fixed_string_array_member(
  rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name, size_t name_length,
  const char * default_value, size_t default_value_length,
  size_t string_length, size_t array_length)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(default_value, RCUTILS_RET_INVALID_ARGUMENT);
  return (dynamic_type_builder->serialization_support->methods.
         dynamic_type_builder_add_fixed_string_array_member)(
    &dynamic_type_builder->serialization_support->impl, &dynamic_type_builder->impl,
    id,
    name, name_length,
    default_value, default_value_length,
    string_length, array_length);
}


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_builder_add_fixed_wstring_array_member(
  rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name, size_t name_length,
  const char * default_value, size_t default_value_length,
  size_t wstring_length, size_t array_length)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(default_value, RCUTILS_RET_INVALID_ARGUMENT);
  return (dynamic_type_builder->serialization_support->methods.
         dynamic_type_builder_add_fixed_wstring_array_member)(
    &dynamic_type_builder->serialization_support->impl, &dynamic_type_builder->impl,
    id,
    name, name_length,
    default_value, default_value_length,
    wstring_length, array_length);
}


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_builder_add_bounded_string_array_member(
  rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name, size_t name_length,
  const char * default_value, size_t default_value_length,
  size_t string_bound, size_t array_length)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(default_value, RCUTILS_RET_INVALID_ARGUMENT);
  return (dynamic_type_builder->serialization_support->methods.
         dynamic_type_builder_add_bounded_string_array_member)(
    &dynamic_type_builder->serialization_support->impl, &dynamic_type_builder->impl,
    id,
    name, name_length,
    default_value, default_value_length,
    string_bound, array_length);
}


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_builder_add_bounded_wstring_array_member(
  rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name, size_t name_length,
  const char * default_value, size_t default_value_length,
  size_t wstring_bound, size_t array_length)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(default_value, RCUTILS_RET_INVALID_ARGUMENT);
  return (dynamic_type_builder->serialization_support->methods.
         dynamic_type_builder_add_bounded_wstring_array_member)(
    &dynamic_type_builder->serialization_support->impl, &dynamic_type_builder->impl,
    id,
    name, name_length,
    default_value, default_value_length,
    wstring_bound, array_length);
}


// DYNAMIC TYPE UNBOUNDED SEQUENCE MEMBERS =========================================================
#define ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(FunctionT) \
  rcutils_ret_t \
  rosidl_dynamic_typesupport_dynamic_type_builder_add_ ## FunctionT ## _unbounded_sequence_member( \
    rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder, \
    rosidl_dynamic_typesupport_member_id_t id, \
    const char * name, size_t name_length, \
    const char * default_value, size_t default_value_length) \
  { \
    RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT); \
    RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, RCUTILS_RET_INVALID_ARGUMENT); \
    RCUTILS_CHECK_ARGUMENT_FOR_NULL(default_value, RCUTILS_RET_INVALID_ARGUMENT); \
    return (dynamic_type_builder->serialization_support->methods. \
           dynamic_type_builder_add_ ## FunctionT ## _unbounded_sequence_member)( \
      &dynamic_type_builder->serialization_support->impl, &dynamic_type_builder->impl, \
      id, \
      name, name_length, \
      default_value, default_value_length); \
  }


ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(bool)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(byte)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(char)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(wchar)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(float32)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(float64)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(int8)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(uint8)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(int16)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(uint16)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(int32)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(uint32)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(int64)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(uint64)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(string)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN(wstring)
#undef ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_UNBOUNDED_SEQUENCE_MEMBER_FN


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_builder_add_fixed_string_unbounded_sequence_member(
  rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name, size_t name_length,
  const char * default_value, size_t default_value_length,
  size_t string_length)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(default_value, RCUTILS_RET_INVALID_ARGUMENT);
  return (dynamic_type_builder->serialization_support->methods.
         dynamic_type_builder_add_fixed_string_unbounded_sequence_member)(
    &dynamic_type_builder->serialization_support->impl, &dynamic_type_builder->impl,
    id,
    name, name_length,
    default_value, default_value_length,
    string_length);
}


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_builder_add_fixed_wstring_unbounded_sequence_member(
  rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name, size_t name_length,
  const char * default_value, size_t default_value_length,
  size_t wstring_length)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(default_value, RCUTILS_RET_INVALID_ARGUMENT);
  return (dynamic_type_builder->serialization_support->methods.
         dynamic_type_builder_add_fixed_wstring_unbounded_sequence_member)(
    &dynamic_type_builder->serialization_support->impl, &dynamic_type_builder->impl,
    id,
    name, name_length,
    default_value, default_value_length,
    wstring_length);
}


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_builder_add_bounded_string_unbounded_sequence_member(
  rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name, size_t name_length,
  const char * default_value, size_t default_value_length,
  size_t string_sequence_bound)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(default_value, RCUTILS_RET_INVALID_ARGUMENT);
  return (dynamic_type_builder->serialization_support->methods.
         dynamic_type_builder_add_bounded_string_unbounded_sequence_member)(
    &dynamic_type_builder->serialization_support->impl, &dynamic_type_builder->impl,
    id,
    name, name_length,
    default_value, default_value_length,
    string_sequence_bound);
}


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_builder_add_bounded_wstring_unbounded_sequence_member(
  rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name, size_t name_length,
  const char * default_value, size_t default_value_length,
  size_t wstring_sequence_bound)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(default_value, RCUTILS_RET_INVALID_ARGUMENT);
  return (dynamic_type_builder->serialization_support->methods.
         dynamic_type_builder_add_bounded_wstring_unbounded_sequence_member)(
    &dynamic_type_builder->serialization_support->impl, &dynamic_type_builder->impl,
    id,
    name, name_length,
    default_value, default_value_length,
    wstring_sequence_bound);
}


// DYNAMIC TYPE BOUNDED SEQUENCE MEMBERS ===========================================================
#define ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN(FunctionT) \
  rcutils_ret_t \
  rosidl_dynamic_typesupport_dynamic_type_builder_add_ ## FunctionT ## _bounded_sequence_member( \
    rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder, \
    rosidl_dynamic_typesupport_member_id_t id, \
    const char * name, size_t name_length, \
    const char * default_value, size_t default_value_length, \
    size_t sequence_bound) \
  { \
    RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT); \
    RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, RCUTILS_RET_INVALID_ARGUMENT); \
    RCUTILS_CHECK_ARGUMENT_FOR_NULL(default_value, RCUTILS_RET_INVALID_ARGUMENT); \
    return (dynamic_type_builder->serialization_support->methods. \
           dynamic_type_builder_add_ ## FunctionT ## _bounded_sequence_member)( \
      &dynamic_type_builder->serialization_support->impl, &dynamic_type_builder->impl, \
      id, \
      name, name_length, \
      default_value, default_value_length, \
      sequence_bound); \
  }


ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN(bool)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN(byte)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN(char)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN(wchar)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN(float32)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN(float64)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN(int8)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN(uint8)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN(int16)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN(uint16)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN(int32)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN(uint32)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN(int64)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN(uint64)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN(string)
ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN(wstring)
#undef ROSIDL_DYNAMIC_TYPE_BUILDER_ADD_BOUNDED_SEQUENCE_MEMBER_FN


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_builder_add_fixed_string_bounded_sequence_member(
  rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name, size_t name_length,
  const char * default_value, size_t default_value_length,
  size_t string_length, size_t sequence_bound)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(default_value, RCUTILS_RET_INVALID_ARGUMENT);
  return (dynamic_type_builder->serialization_support->methods.
         dynamic_type_builder_add_fixed_string_bounded_sequence_member)(
    &dynamic_type_builder->serialization_support->impl, &dynamic_type_builder->impl,
    id,
    name, name_length,
    default_value, default_value_length,
    string_length, sequence_bound);
}


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_builder_add_fixed_wstring_bounded_sequence_member(
  rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name, size_t name_length,
  const char * default_value, size_t default_value_length,
  size_t wstring_length, size_t sequence_bound)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(default_value, RCUTILS_RET_INVALID_ARGUMENT);
  return (dynamic_type_builder->serialization_support->methods.
         dynamic_type_builder_add_fixed_wstring_bounded_sequence_member)(
    &dynamic_type_builder->serialization_support->impl, &dynamic_type_builder->impl,
    id,
    name, name_length,
    default_value, default_value_length,
    wstring_length, sequence_bound);
}


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_builder_add_bounded_string_bounded_sequence_member(
  rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name, size_t name_length,
  const char * default_value, size_t default_value_length,
  size_t string_bound, size_t sequence_bound)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(default_value, RCUTILS_RET_INVALID_ARGUMENT);
  return (dynamic_type_builder->serialization_support->methods.
         dynamic_type_builder_add_bounded_string_bounded_sequence_member)(
    &dynamic_type_builder->serialization_support->impl, &dynamic_type_builder->impl,
    id,
    name, name_length,
    default_value, default_value_length,
    string_bound, sequence_bound);
}


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_builder_add_bounded_wstring_bounded_sequence_member(
  rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name, size_t name_length,
  const char * default_value, size_t default_value_length,
  size_t wstring_bound, size_t sequence_bound)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(default_value, RCUTILS_RET_INVALID_ARGUMENT);
  return (dynamic_type_builder->serialization_support->methods.
         dynamic_type_builder_add_bounded_wstring_bounded_sequence_member)(
    &dynamic_type_builder->serialization_support->impl, &dynamic_type_builder->impl,
    id,
    name, name_length,
    default_value, default_value_length,
    wstring_bound, sequence_bound);
}


// DYNAMIC TYPE NESTED MEMBERS =====================================================================
rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_builder_add_complex_member(
  rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name, size_t name_length,
  const char * default_value, size_t default_value_length,
  rosidl_dynamic_typesupport_dynamic_type_t * nested_struct)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(default_value, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(nested_struct, RCUTILS_RET_INVALID_ARGUMENT);
  return (dynamic_type_builder->serialization_support->methods.
         dynamic_type_builder_add_complex_member)(
    &dynamic_type_builder->serialization_support->impl, &dynamic_type_builder->impl,
    id,
    name, name_length,
    default_value, default_value_length,
    &nested_struct->impl);
}


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_builder_add_complex_array_member(
  rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name, size_t name_length,
  const char * default_value, size_t default_value_length,
  rosidl_dynamic_typesupport_dynamic_type_t * nested_struct, size_t sequence_bound)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(default_value, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(nested_struct, RCUTILS_RET_INVALID_ARGUMENT);
  return (dynamic_type_builder->serialization_support->methods.
         dynamic_type_builder_add_complex_array_member)(
    &dynamic_type_builder->serialization_support->impl, &dynamic_type_builder->impl,
    id,
    name, name_length,
    default_value, default_value_length,
    &nested_struct->impl, sequence_bound);
}


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_builder_add_complex_unbounded_sequence_member(
  rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name, size_t name_length,
  const char * default_value, size_t default_value_length,
  rosidl_dynamic_typesupport_dynamic_type_t * nested_struct)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(default_value, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(nested_struct, RCUTILS_RET_INVALID_ARGUMENT);
  return (dynamic_type_builder->serialization_support->methods.
         dynamic_type_builder_add_complex_unbounded_sequence_member)(
    &dynamic_type_builder->serialization_support->impl, &dynamic_type_builder->impl,
    id,
    name, name_length,
    default_value, default_value_length,
    &nested_struct->impl);
}


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_builder_add_complex_bounded_sequence_member(
  rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name, size_t name_length,
  const char * default_value, size_t default_value_length,
  rosidl_dynamic_typesupport_dynamic_type_t * nested_struct, size_t sequence_bound)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(default_value, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(nested_struct, RCUTILS_RET_INVALID_ARGUMENT);
  return (dynamic_type_builder->serialization_support->methods.
         dynamic_type_builder_add_complex_bounded_sequence_member)(
    &dynamic_type_builder->serialization_support->impl, &dynamic_type_builder->impl,
    id,
    name, name_length,
    default_value, default_value_length,
    &nested_struct->impl, sequence_bound);
}


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_builder_add_complex_member_builder(
  rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name, size_t name_length,
  const char * default_value, size_t default_value_length,
  rosidl_dynamic_typesupport_dynamic_type_builder_t * nested_struct_builder)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(default_value, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(nested_struct_builder, RCUTILS_RET_INVALID_ARGUMENT);
  return (dynamic_type_builder->serialization_support->methods.
         dynamic_type_builder_add_complex_member_builder)(
    &dynamic_type_builder->serialization_support->impl, &dynamic_type_builder->impl,
    id,
    name, name_length,
    default_value, default_value_length,
    &nested_struct_builder->impl);
}


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_builder_add_complex_array_member_builder(
  rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name, size_t name_length,
  const char * default_value, size_t default_value_length,
  rosidl_dynamic_typesupport_dynamic_type_builder_t * nested_struct_builder, size_t sequence_bound)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(default_value, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(nested_struct_builder, RCUTILS_RET_INVALID_ARGUMENT);
  return (dynamic_type_builder->serialization_support->methods.
         dynamic_type_builder_add_complex_array_member_builder)(
    &dynamic_type_builder->serialization_support->impl, &dynamic_type_builder->impl,
    id,
    name, name_length,
    default_value, default_value_length,
    &nested_struct_builder->impl, sequence_bound);
}


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_builder_add_complex_unbounded_sequence_member_builder(
  rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name, size_t name_length,
  const char * default_value, size_t default_value_length,
  rosidl_dynamic_typesupport_dynamic_type_builder_t * nested_struct_builder)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(default_value, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(nested_struct_builder, RCUTILS_RET_INVALID_ARGUMENT);
  return (dynamic_type_builder->serialization_support->methods.
         dynamic_type_builder_add_complex_unbounded_sequence_member_builder)(
    &dynamic_type_builder->serialization_support->impl, &dynamic_type_builder->impl,
    id,
    name, name_length,
    default_value, default_value_length,
    &nested_struct_builder->impl);
}


rcutils_ret_t
rosidl_dynamic_typesupport_dynamic_type_builder_add_complex_bounded_sequence_member_builder(
  rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * name, size_t name_length,
  const char * default_value, size_t default_value_length,
  rosidl_dynamic_typesupport_dynamic_type_builder_t * nested_struct_builder, size_t sequence_bound)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(dynamic_type_builder, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(name, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(default_value, RCUTILS_RET_INVALID_ARGUMENT);
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(nested_struct_builder, RCUTILS_RET_INVALID_ARGUMENT);
  return (dynamic_type_builder->serialization_support->methods.
         dynamic_type_builder_add_complex_bounded_sequence_member_builder)(
    &dynamic_type_builder->serialization_support->impl, &dynamic_type_builder->impl,
    id,
    name, name_length,
    default_value, default_value_length,
    &nested_struct_builder->impl, sequence_bound);
}
