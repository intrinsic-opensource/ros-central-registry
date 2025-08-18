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

#ifndef DETAIL__FASTDDS_DYNAMIC_DATA_HPP_
#define DETAIL__FASTDDS_DYNAMIC_DATA_HPP_

#include <rosidl_dynamic_typesupport_fastrtps/visibility_control.h>

#include <rcutils/allocator.h>
#include <rcutils/types/rcutils_ret.h>
#include <rcutils/types/uint8_array.h>
#include <rosidl_dynamic_typesupport/api/serialization_support_interface.h>
#include <rosidl_dynamic_typesupport/uchar.h>


// =================================================================================================
// DYNAMIC DATA
// =================================================================================================

// DYNAMIC DATA UTILS ==============================================================================
ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_clear_all_values(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl);

// Clears all values, except for aggregated type keys
ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_clear_nonkey_values(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl);

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_clear_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id);

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_equals(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * other_data_impl,
  bool * equals);  // OUT

// Can be used to get sequence/array length, and also number of members for struct
ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_get_item_count(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  size_t * item_count);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_get_member_id_by_name(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  const char * name,
  size_t name_length,
  rosidl_dynamic_typesupport_member_id_t * member_id);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_get_member_id_at_index(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  size_t index,
  rosidl_dynamic_typesupport_member_id_t * member_id);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_get_array_index(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  size_t index,
  rosidl_dynamic_typesupport_member_id_t * array_index);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_loan_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  rcutils_allocator_t * allocator,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * loaned_data_impl);  // OUT

// The passed 'inner_data_impl' arg to return must be the immediate child of the passed 'data_impl'
// arg
ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_return_loaned_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * inner_data_impl);

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_get_name(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  const char ** name,  // OUT
  size_t * name_length);  // OUT


// DYNAMIC DATA CONSTRUCTION =======================================================================
ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_init_from_dynamic_type_builder(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  rcutils_allocator_t * allocator,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_init_from_dynamic_type(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_impl_t * type_impl,
  rcutils_allocator_t * allocator,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_clone(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * other_data_impl,
  rcutils_allocator_t * allocator,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_fini(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl);


// DYNAMIC DATA SERIALIZATION ======================================================================
ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_serialize(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rcutils_uint8_array_t * buffer);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_deserialize(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,  // OUT
  rcutils_uint8_array_t * buffer);


// DYNAMIC DATA PRIMITIVE MEMBERS GETTERS ==========================================================
ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_get_bool_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  bool * value);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_get_byte_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  unsigned
  char * value);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_get_char_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  char * value);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_get_wchar_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  char16_t * value);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_get_float32_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  float * value);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_get_float64_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  double * value);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_get_int8_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  int8_t * value);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_get_uint8_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  uint8_t * value);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_get_int16_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  int16_t * value);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_get_uint16_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  uint16_t * value);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_get_int32_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  int32_t * value);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_get_uint32_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  uint32_t * value);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_get_int64_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  int64_t * value);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_get_uint64_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  uint64_t * value);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_get_string_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  char ** value,  // OUT
  size_t * value_length);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_get_wstring_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  char16_t ** value,  // OUT
  size_t * value_length);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_get_fixed_string_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  char ** value,  // OUT
  size_t * value_length,  // OUT
  size_t string_length);

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_get_fixed_wstring_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  char16_t ** value,  // OUT
  size_t * value_length,  // OUT
  size_t wstring_length);

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_get_bounded_string_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  char ** value,  // OUT
  size_t * value_length,  // OUT
  size_t string_bound);

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_get_bounded_wstring_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  char16_t ** value,  // OUT
  size_t * value_length,  // OUT
  size_t wstring_bound);


// DYNAMIC DATA PRIMITIVE MEMBERS SETTERS ==========================================================
ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_set_bool_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  bool value);

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_set_byte_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  uint8_t value);

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_set_char_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  char value);

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_set_wchar_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  char16_t value);

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_set_float32_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  float value);

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_set_float64_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  double value);

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_set_int8_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  int8_t value);

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_set_uint8_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  uint8_t value);

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_set_int16_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  int16_t value);

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_set_uint16_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  uint16_t value);

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_set_int32_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  int32_t value);

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_set_uint32_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  uint32_t value);

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_set_int64_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  int64_t value);

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_set_uint64_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  uint64_t value);

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_set_string_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * value,
  size_t value_length);

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_set_wstring_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char16_t * value,
  size_t value_length);

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_set_fixed_string_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * value,
  size_t value_length,
  size_t string_length);

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_set_fixed_wstring_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char16_t * value,
  size_t value_length,
  size_t wstring_length);

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_set_bounded_string_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * value,
  size_t value_length,
  size_t string_bound);

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_set_bounded_wstring_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char16_t * value,
  size_t value_length,
  size_t wstring_bound);


// DYNAMIC DATA SEQUENCES ==========================================================================
ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_clear_sequence_data(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl);

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_remove_sequence_data(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id);

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_insert_sequence_data(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t * out_id);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_insert_bool_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  bool value,
  rosidl_dynamic_typesupport_member_id_t * out_id);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_insert_byte_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  uint8_t value,
  rosidl_dynamic_typesupport_member_id_t * out_id);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_insert_char_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  char value,
  rosidl_dynamic_typesupport_member_id_t * out_id);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_insert_wchar_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  char16_t value,
  rosidl_dynamic_typesupport_member_id_t * out_id);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_insert_float32_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  float value,
  rosidl_dynamic_typesupport_member_id_t * out_id);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_insert_float64_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  double value,
  rosidl_dynamic_typesupport_member_id_t * out_id);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_insert_int8_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  int8_t value,
  rosidl_dynamic_typesupport_member_id_t * out_id);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_insert_uint8_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  uint8_t value,
  rosidl_dynamic_typesupport_member_id_t * out_id);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_insert_int16_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  int16_t value,
  rosidl_dynamic_typesupport_member_id_t * out_id);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_insert_uint16_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  uint16_t value,
  rosidl_dynamic_typesupport_member_id_t * out_id);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_insert_int32_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  int32_t value,
  rosidl_dynamic_typesupport_member_id_t * out_id);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_insert_uint32_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  uint32_t value,
  rosidl_dynamic_typesupport_member_id_t * out_id);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_insert_int64_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  int64_t value,
  rosidl_dynamic_typesupport_member_id_t * out_id);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_insert_uint64_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  uint64_t value,
  rosidl_dynamic_typesupport_member_id_t * out_id);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_insert_string_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  const char * value,
  size_t value_length,
  rosidl_dynamic_typesupport_member_id_t * out_id);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_insert_wstring_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  const char16_t * value,
  size_t value_length,
  rosidl_dynamic_typesupport_member_id_t * out_id);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_insert_fixed_string_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  const char * value,
  size_t value_length,
  size_t string_length,
  rosidl_dynamic_typesupport_member_id_t * out_id);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_insert_fixed_wstring_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  const char16_t * value,
  size_t value_length,
  size_t wstring_length,
  rosidl_dynamic_typesupport_member_id_t * out_id);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_insert_bounded_string_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  const char * value,
  size_t value_length,
  size_t string_bound,
  rosidl_dynamic_typesupport_member_id_t * out_id);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_insert_bounded_wstring_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  const char16_t * value,
  size_t value_length,
  size_t wstring_bound,
  rosidl_dynamic_typesupport_member_id_t * out_id);  // OUT


// DYNAMIC DATA NESTED MEMBERS =====================================================================
ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_get_complex_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  rcutils_allocator_t * allocator,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * value);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_set_complex_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * value);

// This deep copies the passed data
ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_insert_complex_value_copy(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * value,
  rosidl_dynamic_typesupport_member_id_t * out_id);  // OUT

ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__dynamic_data_insert_complex_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * value,
  rosidl_dynamic_typesupport_member_id_t * out_id);  // OUT


#endif  // DETAIL__FASTDDS_DYNAMIC_DATA_HPP_
