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

#include <rcutils/allocator.h>
#include <rcutils/error_handling.h>
#include <rcutils/types/rcutils_ret.h>
#include <rcutils/types/uint8_array.h>

#include <rosidl_dynamic_typesupport/api/serialization_support.h>
#include <rosidl_dynamic_typesupport/api/serialization_support_interface.h>
#include <rosidl_dynamic_typesupport/types.h>

#include <fastdds/dds/xtypes/dynamic_types/DynamicDataFactory.hpp>
#include <fastdds/dds/xtypes/dynamic_types/DynamicTypeBuilderFactory.hpp>

#include "rosidl_dynamic_typesupport_fastrtps/identifier.h"
#include "rosidl_dynamic_typesupport_fastrtps/serialization_support.h"

#include "detail/fastrtps_dynamic_data.hpp"
#include "detail/fastrtps_dynamic_type.hpp"
#include "detail/fastrtps_serialization_support.hpp"

// =================================================================================================
// SERIALIZATION SUPPORT IMPL
// =================================================================================================
rcutils_ret_t
rosidl_dynamic_typesupport_fastdds_init_serialization_support_impl(
  rcutils_allocator_t * allocator,
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(allocator, RCUTILS_RET_INVALID_ARGUMENT);
  if (!rcutils_allocator_is_valid(allocator)) {
    RCUTILS_SET_ERROR_MSG("allocator is invalid");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(serialization_support_impl, RCUTILS_RET_INVALID_ARGUMENT);

  auto serialization_support_impl_handle =
    static_cast<fastdds__serialization_support_impl_handle_t *>(allocator->zero_allocate(
      1, sizeof(fastdds__serialization_support_impl_handle_t),
      allocator->state));
  if (!serialization_support_impl_handle) {
    RCUTILS_SET_ERROR_MSG("could not allocate fastdds serialization support impl handle");
    return RCUTILS_RET_BAD_ALLOC;
  }

  serialization_support_impl->allocator = *allocator;
  serialization_support_impl->serialization_library_identifier =
    fastdds_serialization_support_library_identifier;

  serialization_support_impl_handle->type_factory_ =
    eprosima::fastdds::dds::DynamicTypeBuilderFactory::get_instance();
  serialization_support_impl_handle->data_factory_ =
    eprosima::fastdds::dds::DynamicDataFactory::get_instance();

  serialization_support_impl->handle = serialization_support_impl_handle;

  return RCUTILS_RET_OK;
}

// =================================================================================================
// SERIALIZATION SUPPORT INTERFACE
// =================================================================================================
rcutils_ret_t
rosidl_dynamic_typesupport_fastdds_init_serialization_support_interface(
  rcutils_allocator_t * allocator,
  rosidl_dynamic_typesupport_serialization_support_interface_t * serialization_support_interface)
{
  // TODO(methylDragon): Repalce this with a static variable and aggregate initialization once we
  //   migrate to C++20

  // CORE ==========================================================================================
  serialization_support_interface->allocator = *allocator;

  serialization_support_interface->serialization_library_identifier =
    fastdds_serialization_support_library_identifier;

  serialization_support_interface->serialization_support_impl_fini =
    fastdds__serialization_support_impl_fini;

  serialization_support_interface->serialization_support_interface_fini =
    fastdds__serialization_support_interface_fini;


  // ===============================================================================================
  // DYNAMIC TYPE METHODS
  // ===============================================================================================
  // DYNAMIC TYPE UTILS
  serialization_support_interface->dynamic_type_equals =
    fastdds__dynamic_type_equals;

  serialization_support_interface->dynamic_type_get_member_count =
    fastdds__dynamic_type_get_member_count;


  // DYNAMIC TYPE CONSTRUCTION
  serialization_support_interface->dynamic_type_builder_init =
    fastdds__dynamic_type_builder_init;

  serialization_support_interface->dynamic_type_builder_clone =
    fastdds__dynamic_type_builder_clone;

  serialization_support_interface->dynamic_type_builder_fini =
    fastdds__dynamic_type_builder_fini;

  serialization_support_interface->dynamic_type_init_from_dynamic_type_builder =
    fastdds__dynamic_type_init_from_dynamic_type_builder;

  serialization_support_interface->dynamic_type_clone =
    fastdds__dynamic_type_clone;

  serialization_support_interface->dynamic_type_fini =
    fastdds__dynamic_type_fini;

  serialization_support_interface->dynamic_type_get_name =
    fastdds__dynamic_type_get_name;

  serialization_support_interface->dynamic_type_builder_get_name =
    fastdds__dynamic_type_builder_get_name;

  serialization_support_interface->dynamic_type_builder_set_name =
    fastdds__dynamic_type_builder_set_name;


  // DYNAMIC TYPE PRIMITIVE MEMBERS ================================================================
  serialization_support_interface->dynamic_type_builder_add_bool_member =
    fastdds__dynamic_type_builder_add_bool_member;

  serialization_support_interface->dynamic_type_builder_add_byte_member =
    fastdds__dynamic_type_builder_add_byte_member;

  serialization_support_interface->dynamic_type_builder_add_char_member =
    fastdds__dynamic_type_builder_add_char_member;

  serialization_support_interface->dynamic_type_builder_add_wchar_member =
    fastdds__dynamic_type_builder_add_wchar_member;

  serialization_support_interface->dynamic_type_builder_add_float32_member =
    fastdds__dynamic_type_builder_add_float32_member;

  serialization_support_interface->dynamic_type_builder_add_float64_member =
    fastdds__dynamic_type_builder_add_float64_member;

  serialization_support_interface->dynamic_type_builder_add_int8_member =
    fastdds__dynamic_type_builder_add_int8_member;

  serialization_support_interface->dynamic_type_builder_add_uint8_member =
    fastdds__dynamic_type_builder_add_uint8_member;

  serialization_support_interface->dynamic_type_builder_add_int16_member =
    fastdds__dynamic_type_builder_add_int16_member;

  serialization_support_interface->dynamic_type_builder_add_uint16_member =
    fastdds__dynamic_type_builder_add_uint16_member;

  serialization_support_interface->dynamic_type_builder_add_int32_member =
    fastdds__dynamic_type_builder_add_int32_member;

  serialization_support_interface->dynamic_type_builder_add_uint32_member =
    fastdds__dynamic_type_builder_add_uint32_member;

  serialization_support_interface->dynamic_type_builder_add_int64_member =
    fastdds__dynamic_type_builder_add_int64_member;

  serialization_support_interface->dynamic_type_builder_add_uint64_member =
    fastdds__dynamic_type_builder_add_uint64_member;

  serialization_support_interface->dynamic_type_builder_add_string_member =
    fastdds__dynamic_type_builder_add_string_member;

  serialization_support_interface->dynamic_type_builder_add_wstring_member =
    fastdds__dynamic_type_builder_add_wstring_member;

  serialization_support_interface->dynamic_type_builder_add_fixed_string_member =
    fastdds__dynamic_type_builder_add_fixed_string_member;

  serialization_support_interface->dynamic_type_builder_add_fixed_wstring_member =
    fastdds__dynamic_type_builder_add_fixed_wstring_member;

  serialization_support_interface->dynamic_type_builder_add_bounded_string_member =
    fastdds__dynamic_type_builder_add_bounded_string_member;

  serialization_support_interface->dynamic_type_builder_add_bounded_wstring_member =
    fastdds__dynamic_type_builder_add_bounded_wstring_member;


  // DYNAMIC TYPE STATIC ARRAY MEMBERS =============================================================
  serialization_support_interface->dynamic_type_builder_add_fixed_string_array_member =
    fastdds__dynamic_type_builder_add_fixed_string_array_member;

  serialization_support_interface->dynamic_type_builder_add_bool_array_member =
    fastdds__dynamic_type_builder_add_bool_array_member;

  serialization_support_interface->dynamic_type_builder_add_byte_array_member =
    fastdds__dynamic_type_builder_add_byte_array_member;

  serialization_support_interface->dynamic_type_builder_add_char_array_member =
    fastdds__dynamic_type_builder_add_char_array_member;

  serialization_support_interface->dynamic_type_builder_add_wchar_array_member =
    fastdds__dynamic_type_builder_add_wchar_array_member;

  serialization_support_interface->dynamic_type_builder_add_float32_array_member =
    fastdds__dynamic_type_builder_add_float32_array_member;

  serialization_support_interface->dynamic_type_builder_add_float64_array_member =
    fastdds__dynamic_type_builder_add_float64_array_member;

  serialization_support_interface->dynamic_type_builder_add_int8_array_member =
    fastdds__dynamic_type_builder_add_int8_array_member;

  serialization_support_interface->dynamic_type_builder_add_uint8_array_member =
    fastdds__dynamic_type_builder_add_uint8_array_member;

  serialization_support_interface->dynamic_type_builder_add_int16_array_member =
    fastdds__dynamic_type_builder_add_int16_array_member;

  serialization_support_interface->dynamic_type_builder_add_uint16_array_member =
    fastdds__dynamic_type_builder_add_uint16_array_member;

  serialization_support_interface->dynamic_type_builder_add_int32_array_member =
    fastdds__dynamic_type_builder_add_int32_array_member;

  serialization_support_interface->dynamic_type_builder_add_uint32_array_member =
    fastdds__dynamic_type_builder_add_uint32_array_member;

  serialization_support_interface->dynamic_type_builder_add_int64_array_member =
    fastdds__dynamic_type_builder_add_int64_array_member;

  serialization_support_interface->dynamic_type_builder_add_uint64_array_member =
    fastdds__dynamic_type_builder_add_uint64_array_member;

  serialization_support_interface->dynamic_type_builder_add_string_array_member =
    fastdds__dynamic_type_builder_add_string_array_member;

  serialization_support_interface->dynamic_type_builder_add_wstring_array_member =
    fastdds__dynamic_type_builder_add_wstring_array_member;

  serialization_support_interface->dynamic_type_builder_add_fixed_string_array_member =
    fastdds__dynamic_type_builder_add_fixed_string_array_member;

  serialization_support_interface->dynamic_type_builder_add_fixed_wstring_array_member =
    fastdds__dynamic_type_builder_add_fixed_wstring_array_member;

  serialization_support_interface->dynamic_type_builder_add_bounded_string_array_member =
    fastdds__dynamic_type_builder_add_bounded_string_array_member;

  serialization_support_interface->dynamic_type_builder_add_bounded_wstring_array_member =
    fastdds__dynamic_type_builder_add_bounded_wstring_array_member;


  // DYNAMIC TYPE UNBOUNDED SEQUENCE MEMBERS =======================================================
  serialization_support_interface->dynamic_type_builder_add_bool_unbounded_sequence_member =
    fastdds__dynamic_type_builder_add_bool_unbounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_byte_unbounded_sequence_member =
    fastdds__dynamic_type_builder_add_byte_unbounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_char_unbounded_sequence_member =
    fastdds__dynamic_type_builder_add_char_unbounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_wchar_unbounded_sequence_member =
    fastdds__dynamic_type_builder_add_wchar_unbounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_float32_unbounded_sequence_member =
    fastdds__dynamic_type_builder_add_float32_unbounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_float64_unbounded_sequence_member =
    fastdds__dynamic_type_builder_add_float64_unbounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_int8_unbounded_sequence_member =
    fastdds__dynamic_type_builder_add_int8_unbounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_uint8_unbounded_sequence_member =
    fastdds__dynamic_type_builder_add_uint8_unbounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_int16_unbounded_sequence_member =
    fastdds__dynamic_type_builder_add_int16_unbounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_uint16_unbounded_sequence_member =
    fastdds__dynamic_type_builder_add_uint16_unbounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_int32_unbounded_sequence_member =
    fastdds__dynamic_type_builder_add_int32_unbounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_uint32_unbounded_sequence_member =
    fastdds__dynamic_type_builder_add_uint32_unbounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_int64_unbounded_sequence_member =
    fastdds__dynamic_type_builder_add_int64_unbounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_uint64_unbounded_sequence_member =
    fastdds__dynamic_type_builder_add_uint64_unbounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_string_unbounded_sequence_member =
    fastdds__dynamic_type_builder_add_string_unbounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_wstring_unbounded_sequence_member =
    fastdds__dynamic_type_builder_add_wstring_unbounded_sequence_member;

  serialization_support_interface->
  dynamic_type_builder_add_fixed_string_unbounded_sequence_member =
    fastdds__dynamic_type_builder_add_fixed_string_unbounded_sequence_member;

  serialization_support_interface->
  dynamic_type_builder_add_fixed_wstring_unbounded_sequence_member =
    fastdds__dynamic_type_builder_add_fixed_wstring_unbounded_sequence_member;

  serialization_support_interface->
  dynamic_type_builder_add_bounded_string_unbounded_sequence_member =
    fastdds__dynamic_type_builder_add_bounded_string_unbounded_sequence_member;

  serialization_support_interface->
  dynamic_type_builder_add_bounded_wstring_unbounded_sequence_member =
    fastdds__dynamic_type_builder_add_bounded_wstring_unbounded_sequence_member;


  // DYNAMIC TYPE BOUNDED SEQUENCE MEMBERS =========================================================
  serialization_support_interface->dynamic_type_builder_add_bool_bounded_sequence_member =
    fastdds__dynamic_type_builder_add_bool_bounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_byte_bounded_sequence_member =
    fastdds__dynamic_type_builder_add_byte_bounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_char_bounded_sequence_member =
    fastdds__dynamic_type_builder_add_char_bounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_wchar_bounded_sequence_member =
    fastdds__dynamic_type_builder_add_wchar_bounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_float32_bounded_sequence_member =
    fastdds__dynamic_type_builder_add_float32_bounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_float64_bounded_sequence_member =
    fastdds__dynamic_type_builder_add_float64_bounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_int8_bounded_sequence_member =
    fastdds__dynamic_type_builder_add_int8_bounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_uint8_bounded_sequence_member =
    fastdds__dynamic_type_builder_add_uint8_bounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_int16_bounded_sequence_member =
    fastdds__dynamic_type_builder_add_int16_bounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_uint16_bounded_sequence_member =
    fastdds__dynamic_type_builder_add_uint16_bounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_int32_bounded_sequence_member =
    fastdds__dynamic_type_builder_add_int32_bounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_uint32_bounded_sequence_member =
    fastdds__dynamic_type_builder_add_uint32_bounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_int64_bounded_sequence_member =
    fastdds__dynamic_type_builder_add_int64_bounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_uint64_bounded_sequence_member =
    fastdds__dynamic_type_builder_add_uint64_bounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_string_bounded_sequence_member =
    fastdds__dynamic_type_builder_add_string_bounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_wstring_bounded_sequence_member =
    fastdds__dynamic_type_builder_add_wstring_bounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_fixed_string_bounded_sequence_member =
    fastdds__dynamic_type_builder_add_fixed_string_bounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_fixed_wstring_bounded_sequence_member =
    fastdds__dynamic_type_builder_add_fixed_wstring_bounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_bounded_string_bounded_sequence_member =
    fastdds__dynamic_type_builder_add_bounded_string_bounded_sequence_member;

  serialization_support_interface->
  dynamic_type_builder_add_bounded_wstring_bounded_sequence_member =
    fastdds__dynamic_type_builder_add_bounded_wstring_bounded_sequence_member;


  // DYNAMIC TYPE NESTED MEMBERS
  serialization_support_interface->dynamic_type_builder_add_complex_member =
    fastdds__dynamic_type_builder_add_complex_member;

  serialization_support_interface->dynamic_type_builder_add_complex_array_member =
    fastdds__dynamic_type_builder_add_complex_array_member;

  serialization_support_interface->dynamic_type_builder_add_complex_unbounded_sequence_member =
    fastdds__dynamic_type_builder_add_complex_unbounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_complex_bounded_sequence_member =
    fastdds__dynamic_type_builder_add_complex_bounded_sequence_member;

  serialization_support_interface->dynamic_type_builder_add_complex_member_builder =
    fastdds__dynamic_type_builder_add_complex_member_builder;

  serialization_support_interface->dynamic_type_builder_add_complex_array_member_builder =
    fastdds__dynamic_type_builder_add_complex_array_member_builder;

  serialization_support_interface->
  dynamic_type_builder_add_complex_unbounded_sequence_member_builder =
    fastdds__dynamic_type_builder_add_complex_unbounded_sequence_member_builder;

  serialization_support_interface->
  dynamic_type_builder_add_complex_bounded_sequence_member_builder =
    fastdds__dynamic_type_builder_add_complex_bounded_sequence_member_builder;

  // ===============================================================================================
  // DYNAMIC DATA METHODS
  // ===============================================================================================
  // DYNAMIC DATA UTILS
  serialization_support_interface->dynamic_data_clear_all_values =
    fastdds__dynamic_data_clear_all_values;

  serialization_support_interface->dynamic_data_clear_nonkey_values =
    fastdds__dynamic_data_clear_nonkey_values;

  serialization_support_interface->dynamic_data_clear_sequence_data =
    fastdds__dynamic_data_clear_sequence_data;

  serialization_support_interface->dynamic_data_clear_value =
    fastdds__dynamic_data_clear_value;

  serialization_support_interface->dynamic_data_equals =
    fastdds__dynamic_data_equals;

  serialization_support_interface->dynamic_data_get_item_count =
    fastdds__dynamic_data_get_item_count;

  serialization_support_interface->dynamic_data_get_member_id_by_name =
    fastdds__dynamic_data_get_member_id_by_name;

  serialization_support_interface->dynamic_data_get_member_id_at_index =
    fastdds__dynamic_data_get_member_id_at_index;

  serialization_support_interface->dynamic_data_get_array_index =
    fastdds__dynamic_data_get_array_index;

  serialization_support_interface->dynamic_data_loan_value =
    fastdds__dynamic_data_loan_value;

  serialization_support_interface->dynamic_data_return_loaned_value =
    fastdds__dynamic_data_return_loaned_value;

  serialization_support_interface->dynamic_data_get_name =
    fastdds__dynamic_data_get_name;


  // DYNAMIC DATA CONSTRUCTION
  serialization_support_interface->dynamic_data_init_from_dynamic_type_builder =
    fastdds__dynamic_data_init_from_dynamic_type_builder;

  serialization_support_interface->dynamic_data_init_from_dynamic_type =
    fastdds__dynamic_data_init_from_dynamic_type;

  serialization_support_interface->dynamic_data_clone =
    fastdds__dynamic_data_clone;

  serialization_support_interface->dynamic_data_fini =
    fastdds__dynamic_data_fini;


  // DYNAMIC DATA SERIALIZATION
  serialization_support_interface->dynamic_data_serialize =
    fastdds__dynamic_data_serialize;

  serialization_support_interface->dynamic_data_deserialize =
    fastdds__dynamic_data_deserialize;


  // DYNAMIC DATA PRIMITIVE MEMBER GETTERS =========================================================
  serialization_support_interface->dynamic_data_get_bool_value =
    fastdds__dynamic_data_get_bool_value;

  serialization_support_interface->dynamic_data_get_byte_value =
    fastdds__dynamic_data_get_byte_value;

  serialization_support_interface->dynamic_data_get_char_value =
    fastdds__dynamic_data_get_char_value;

  serialization_support_interface->dynamic_data_get_wchar_value =
    fastdds__dynamic_data_get_wchar_value;

  serialization_support_interface->dynamic_data_get_float32_value =
    fastdds__dynamic_data_get_float32_value;

  serialization_support_interface->dynamic_data_get_float64_value =
    fastdds__dynamic_data_get_float64_value;

  serialization_support_interface->dynamic_data_get_int8_value =
    fastdds__dynamic_data_get_int8_value;

  serialization_support_interface->dynamic_data_get_uint8_value =
    fastdds__dynamic_data_get_uint8_value;

  serialization_support_interface->dynamic_data_get_int16_value =
    fastdds__dynamic_data_get_int16_value;

  serialization_support_interface->dynamic_data_get_uint16_value =
    fastdds__dynamic_data_get_uint16_value;

  serialization_support_interface->dynamic_data_get_int32_value =
    fastdds__dynamic_data_get_int32_value;

  serialization_support_interface->dynamic_data_get_uint32_value =
    fastdds__dynamic_data_get_uint32_value;

  serialization_support_interface->dynamic_data_get_int64_value =
    fastdds__dynamic_data_get_int64_value;

  serialization_support_interface->dynamic_data_get_uint64_value =
    fastdds__dynamic_data_get_uint64_value;

  serialization_support_interface->dynamic_data_get_string_value =
    fastdds__dynamic_data_get_string_value;

  serialization_support_interface->dynamic_data_get_wstring_value =
    fastdds__dynamic_data_get_wstring_value;

  serialization_support_interface->dynamic_data_get_fixed_string_value =
    fastdds__dynamic_data_get_fixed_string_value;

  serialization_support_interface->dynamic_data_get_fixed_wstring_value =
    fastdds__dynamic_data_get_fixed_wstring_value;

  serialization_support_interface->dynamic_data_get_bounded_string_value =
    fastdds__dynamic_data_get_bounded_string_value;

  serialization_support_interface->dynamic_data_get_bounded_wstring_value =
    fastdds__dynamic_data_get_bounded_wstring_value;


  // DYNAMIC DATA PRIMITIVE MEMBER SETTERS =========================================================
  serialization_support_interface->dynamic_data_set_bool_value =
    fastdds__dynamic_data_set_bool_value;

  serialization_support_interface->dynamic_data_set_byte_value =
    fastdds__dynamic_data_set_byte_value;

  serialization_support_interface->dynamic_data_set_char_value =
    fastdds__dynamic_data_set_char_value;

  serialization_support_interface->dynamic_data_set_wchar_value =
    fastdds__dynamic_data_set_wchar_value;

  serialization_support_interface->dynamic_data_set_float32_value =
    fastdds__dynamic_data_set_float32_value;

  serialization_support_interface->dynamic_data_set_float64_value =
    fastdds__dynamic_data_set_float64_value;

  serialization_support_interface->dynamic_data_set_int8_value =
    fastdds__dynamic_data_set_int8_value;

  serialization_support_interface->dynamic_data_set_uint8_value =
    fastdds__dynamic_data_set_uint8_value;

  serialization_support_interface->dynamic_data_set_int16_value =
    fastdds__dynamic_data_set_int16_value;

  serialization_support_interface->dynamic_data_set_uint16_value =
    fastdds__dynamic_data_set_uint16_value;

  serialization_support_interface->dynamic_data_set_int32_value =
    fastdds__dynamic_data_set_int32_value;

  serialization_support_interface->dynamic_data_set_uint32_value =
    fastdds__dynamic_data_set_uint32_value;

  serialization_support_interface->dynamic_data_set_int64_value =
    fastdds__dynamic_data_set_int64_value;

  serialization_support_interface->dynamic_data_set_uint64_value =
    fastdds__dynamic_data_set_uint64_value;

  serialization_support_interface->dynamic_data_set_string_value =
    fastdds__dynamic_data_set_string_value;

  serialization_support_interface->dynamic_data_set_wstring_value =
    fastdds__dynamic_data_set_wstring_value;

  serialization_support_interface->dynamic_data_set_fixed_string_value =
    fastdds__dynamic_data_set_fixed_string_value;

  serialization_support_interface->dynamic_data_set_fixed_wstring_value =
    fastdds__dynamic_data_set_fixed_wstring_value;

  serialization_support_interface->dynamic_data_set_bounded_string_value =
    fastdds__dynamic_data_set_bounded_string_value;

  serialization_support_interface->dynamic_data_set_bounded_wstring_value =
    fastdds__dynamic_data_set_bounded_wstring_value;


  // DYNAMIC TYPE SEQUENCES ========================================================================
  serialization_support_interface->dynamic_data_clear_sequence_data =
    fastdds__dynamic_data_clear_sequence_data;

  serialization_support_interface->dynamic_data_remove_sequence_data =
    fastdds__dynamic_data_remove_sequence_data;

  serialization_support_interface->dynamic_data_insert_sequence_data =
    fastdds__dynamic_data_insert_sequence_data;

  serialization_support_interface->dynamic_data_insert_bool_value =
    fastdds__dynamic_data_insert_bool_value;

  serialization_support_interface->dynamic_data_insert_byte_value =
    fastdds__dynamic_data_insert_byte_value;

  serialization_support_interface->dynamic_data_insert_char_value =
    fastdds__dynamic_data_insert_char_value;

  serialization_support_interface->dynamic_data_insert_wchar_value =
    fastdds__dynamic_data_insert_wchar_value;

  serialization_support_interface->dynamic_data_insert_float32_value =
    fastdds__dynamic_data_insert_float32_value;

  serialization_support_interface->dynamic_data_insert_float64_value =
    fastdds__dynamic_data_insert_float64_value;

  serialization_support_interface->dynamic_data_insert_int8_value =
    fastdds__dynamic_data_insert_int8_value;

  serialization_support_interface->dynamic_data_insert_uint8_value =
    fastdds__dynamic_data_insert_uint8_value;

  serialization_support_interface->dynamic_data_insert_int16_value =
    fastdds__dynamic_data_insert_int16_value;

  serialization_support_interface->dynamic_data_insert_uint16_value =
    fastdds__dynamic_data_insert_uint16_value;

  serialization_support_interface->dynamic_data_insert_int32_value =
    fastdds__dynamic_data_insert_int32_value;

  serialization_support_interface->dynamic_data_insert_uint32_value =
    fastdds__dynamic_data_insert_uint32_value;

  serialization_support_interface->dynamic_data_insert_int64_value =
    fastdds__dynamic_data_insert_int64_value;

  serialization_support_interface->dynamic_data_insert_uint64_value =
    fastdds__dynamic_data_insert_uint64_value;

  serialization_support_interface->dynamic_data_insert_string_value =
    fastdds__dynamic_data_insert_string_value;

  serialization_support_interface->dynamic_data_insert_wstring_value =
    fastdds__dynamic_data_insert_wstring_value;

  serialization_support_interface->dynamic_data_insert_fixed_string_value =
    fastdds__dynamic_data_insert_fixed_string_value;

  serialization_support_interface->dynamic_data_insert_fixed_wstring_value =
    fastdds__dynamic_data_insert_fixed_wstring_value;

  serialization_support_interface->dynamic_data_insert_bounded_string_value =
    fastdds__dynamic_data_insert_bounded_string_value;

  serialization_support_interface->dynamic_data_insert_bounded_wstring_value =
    fastdds__dynamic_data_insert_bounded_wstring_value;


  // DYNAMIC TYPE NESTED
  serialization_support_interface->dynamic_data_get_complex_value =
    fastdds__dynamic_data_get_complex_value;

  serialization_support_interface->dynamic_data_set_complex_value =
    fastdds__dynamic_data_set_complex_value;

  serialization_support_interface->dynamic_data_insert_complex_value_copy =
    fastdds__dynamic_data_insert_complex_value_copy;

  serialization_support_interface->dynamic_data_insert_complex_value =
    fastdds__dynamic_data_insert_complex_value;

  return RCUTILS_RET_OK;
}  // NOLINT(readability/fn_size)
