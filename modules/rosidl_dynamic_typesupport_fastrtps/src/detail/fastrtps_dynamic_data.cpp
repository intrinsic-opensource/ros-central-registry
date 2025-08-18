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

#include "fastrtps_dynamic_data.hpp"

#include <fastdds/rtps/common/SerializedPayload.hpp>

#include <fastdds/dds/xtypes/dynamic_types/DynamicData.hpp>
#include <fastdds/dds/xtypes/dynamic_types/DynamicPubSubType.hpp>
#include <fastdds/dds/xtypes/dynamic_types/DynamicType.hpp>
#include <fastdds/dds/xtypes/dynamic_types/DynamicTypeBuilder.hpp>

#include <string.h>

#include <rcutils/allocator.h>
#include <rcutils/strdup.h>
#include <rcutils/types/rcutils_ret.h>
#include <rcutils/types/uint8_array.h>
#include <rosidl_dynamic_typesupport/api/serialization_support_interface.h>

#include "rcpputils/scope_exit.hpp"

#include <algorithm>
#include <codecvt>
#include <cstring>
#include <cwchar>
#include <locale>
#include <memory>
#include <string>
#include <utility>

#include "fastrtps_dynamic_type.hpp"
#include "fastrtps_serialization_support.hpp"
#include "macros.hpp"
#include "utils.hpp"

using eprosima::fastdds::dds::DynamicData;
using eprosima::fastdds::dds::DynamicTypeBuilder;

// =================================================================================================
// DYNAMIC DATA
// =================================================================================================

struct fastdds__rosidl_dynamic_typesupport_dynamic_data_impl
{
  DynamicData::_ref_type ref_type;
};

// DYNAMIC DATA UTILS ==============================================================================
rcutils_ret_t
fastdds__dynamic_data_clear_all_values(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl)
{
  (void) serialization_support_impl;

  auto data_handle =
    static_cast<const fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  FASTDDS_CHECK_RET_FOR_NOT_OK_AND_RETURN_WITH_MSG(
    data_handle->ref_type->clear_all_values(),
    "Could not clear all values");
}

rcutils_ret_t
fastdds__dynamic_data_clear_nonkey_values(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl)
{
  (void) serialization_support_impl;

  auto data_handle =
    static_cast<const fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  FASTDDS_CHECK_RET_FOR_NOT_OK_AND_RETURN_WITH_MSG(
    data_handle->ref_type->clear_nonkey_values(), "Could not clear nonkey values");
}

rcutils_ret_t
fastdds__dynamic_data_clear_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;

  auto data_handle =
    static_cast<const fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  FASTDDS_CHECK_RET_FOR_NOT_OK_AND_RETURN_WITH_MSG(
    data_handle->ref_type->clear_value(fastdds__size_t_to_uint32_t(id)),
    "Could not clear value");
}

rcutils_ret_t
fastdds__dynamic_data_equals(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * other_data_impl,
  bool * equals)
{
  (void) serialization_support_impl;

  auto data_handle =
    static_cast<const fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  auto other_data_handle =
    static_cast<const fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(other_data_impl->
    handle);
  if (!other_data_handle || !other_data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_ERROR;
  }
  *equals = data_handle->ref_type->equals(other_data_handle->ref_type);
  return RCUTILS_RET_OK;
}

rcutils_ret_t
fastdds__dynamic_data_get_item_count(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  size_t * item_count)
{
  (void) serialization_support_impl;

  auto data_handle =
    static_cast<const fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  *item_count = data_handle->ref_type->get_item_count();
  return RCUTILS_RET_OK;
}

rcutils_ret_t
fastdds__dynamic_data_get_member_id_by_name(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  const char * name,
  size_t name_length,
  rosidl_dynamic_typesupport_member_id_t * member_id)
{
  (void) serialization_support_impl;

  auto data_handle =
    static_cast<const fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  *member_id = data_handle->ref_type->get_member_id_by_name(std::string(name, name_length));
  return RCUTILS_RET_OK;
}

rcutils_ret_t
fastdds__dynamic_data_get_member_id_at_index(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  size_t index,
  rosidl_dynamic_typesupport_member_id_t * member_id)
{
  (void) serialization_support_impl;

  auto data_handle =
    static_cast<const fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  *member_id = data_handle->ref_type->get_member_id_at_index(fastdds__size_t_to_uint32_t(index));
  return RCUTILS_RET_OK;
}

rcutils_ret_t
fastdds__dynamic_data_get_array_index(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  size_t index,
  rosidl_dynamic_typesupport_member_id_t * array_index)
{
  (void) serialization_support_impl;

  auto data_handle =
    static_cast<const fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  *array_index = data_handle->ref_type->get_member_id_at_index(fastdds__size_t_to_uint32_t(index));

  return eprosima::fastdds::dds::MEMBER_ID_INVALID !=
         *array_index ? RCUTILS_RET_OK : RCUTILS_RET_ERROR;
}

rcutils_ret_t
fastdds__dynamic_data_loan_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  rcutils_allocator_t * allocator,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * loaned_data_impl)
{
  (void) serialization_support_impl;
  (void) allocator;

  auto data_handle =
    static_cast<const fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  auto loaned_data_impl_handle =
    new (std::nothrow) fastdds__rosidl_dynamic_typesupport_dynamic_data_impl();

  if (!loaned_data_impl_handle) {
    RCUTILS_SET_ERROR_MSG("Could not init new data");
    return RCUTILS_RET_BAD_ALLOC;
  }

  auto cleanup_loaned_data_impl_handle = rcpputils::make_scope_exit(
    [loaned_data_impl_handle]() {
      delete loaned_data_impl_handle;
    });

  loaned_data_impl_handle->ref_type =
    data_handle->ref_type->loan_value(fastdds__size_t_to_uint32_t(id));
  if (!loaned_data_impl_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not loan dynamic data");
    return RCUTILS_RET_ERROR;
  }

  loaned_data_impl->handle = std::move(loaned_data_impl_handle);

  cleanup_loaned_data_impl_handle.cancel();

  return RCUTILS_RET_OK;
}

rcutils_ret_t
fastdds__dynamic_data_return_loaned_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * inner_data_impl)
{
  (void) serialization_support_impl;

  auto data_handle =
    static_cast<const fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  auto inner_data_handle =
    static_cast<const fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(inner_data_impl->
    handle);
  if (!inner_data_handle || !inner_data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to inner data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  FASTDDS_CHECK_RET_FOR_NOT_OK_WITH_MSG(
    data_handle->ref_type->return_loaned_value(inner_data_handle->ref_type),
    "Could not return loaned value");
  return RCUTILS_RET_OK;
}

rcutils_ret_t
fastdds__dynamic_data_get_name(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  const char ** name,
  size_t * name_length)
{
  (void) serialization_support_impl;

  auto data_handle =
    static_cast<const fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  std::string tmp_name = data_handle->ref_type->type()->get_name().to_string();
  *name = rcutils_strdup(tmp_name.c_str(), rcutils_get_default_allocator());
  if (nullptr == *name) {
    RCUTILS_SET_ERROR_MSG("Failed to duplicate name for data name");
    return RCUTILS_RET_BAD_ALLOC;
  }
  *name_length = tmp_name.size();
  return RCUTILS_RET_OK;
}

// DYNAMIC DATA CONSTRUCTION =======================================================================
rcutils_ret_t
fastdds__dynamic_data_init_from_dynamic_type_builder(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl,
  rcutils_allocator_t * allocator,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl)
{
  (void) allocator;

  auto type_builder_handle =
    static_cast<const fastdds__rosidl_dynamic_typesupport_dynamic_type_builder_impl *>(
    type_builder_impl->handle);
  if (!type_builder_handle || !type_builder_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to type impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  auto data_impl_handle =
    new (std::nothrow) fastdds__rosidl_dynamic_typesupport_dynamic_data_impl();

  if (nullptr == data_impl_handle) {
    RCUTILS_SET_ERROR_MSG("Could not init new data");
    return RCUTILS_RET_BAD_ALLOC;
  }

  auto cleanup_data_impl_handle = rcpputils::make_scope_exit(
    [data_impl_handle]()
    {
      delete data_impl_handle;
    });

  data_impl_handle->ref_type =
    static_cast<fastdds__serialization_support_impl_handle_t *>(serialization_support_impl->handle)
    ->data_factory_->create_data(type_builder_handle->ref_type->build());
  if (!data_impl_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not init dynamic data from dynamic type builder");
    return RCUTILS_RET_BAD_ALLOC;
  }

  data_impl->handle = data_impl_handle;

  cleanup_data_impl_handle.cancel();

  return RCUTILS_RET_OK;
}

rcutils_ret_t
fastdds__dynamic_data_init_from_dynamic_type(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_type_impl_t * type_impl,
  rcutils_allocator_t * allocator,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl)
{
  (void) allocator;

  auto type_handle =
    static_cast<const fastdds__rosidl_dynamic_typesupport_dynamic_type_impl *>(
    type_impl->handle);
  if (!type_handle || !type_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to type impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  auto data_impl_handle =
    new (std::nothrow) fastdds__rosidl_dynamic_typesupport_dynamic_data_impl();

  if (nullptr == data_impl_handle) {
    RCUTILS_SET_ERROR_MSG("Could not init new data");
    return RCUTILS_RET_BAD_ALLOC;
  }

  auto cleanup_data_impl_handle = rcpputils::make_scope_exit(
    [data_impl_handle]()
    {
      delete data_impl_handle;
    });

  // NOTE(methylDragon): All this casting is unfortunately necessary...
  //
  //                     create_data only takes DynamicType_ptr (aka shared_ptr)
  //                     And passing a heap allocated shared_ptr is the only way to make sure the
  //                     lifetime of the dynamic type is preserved
  data_impl_handle->ref_type =
    static_cast<fastdds__serialization_support_impl_handle_t *>(serialization_support_impl->handle)
    ->data_factory_->create_data(type_handle->ref_type);
  if (!data_impl_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not init dynamic data from dynamic type");
    return RCUTILS_RET_BAD_ALLOC;
  }

  data_impl->handle = data_impl_handle;

  cleanup_data_impl_handle.cancel();

  return RCUTILS_RET_OK;
}

rcutils_ret_t
fastdds__dynamic_data_clone(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * other_data_impl,
  rcutils_allocator_t * allocator,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl)
{
  (void) serialization_support_impl;
  data_impl->allocator = *allocator;

  auto other_data_handle =
    static_cast<const fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(
    other_data_impl->handle);
  if (!other_data_handle || !other_data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  auto data_impl_handle =
    new (std::nothrow) fastdds__rosidl_dynamic_typesupport_dynamic_data_impl();

  if (nullptr == data_impl_handle) {
    RCUTILS_SET_ERROR_MSG("Could not init new data");
    return RCUTILS_RET_BAD_ALLOC;
  }

  auto cleanup_data_impl_handle = rcpputils::make_scope_exit(
    [data_impl_handle]()
    {
      delete data_impl_handle;
    });

  data_impl_handle->ref_type = other_data_handle->ref_type->clone();
  if (!data_impl_handle) {
    RCUTILS_SET_ERROR_MSG("Could not clone struct type builder");
    return RCUTILS_RET_ERROR;
  }

  data_impl->handle = data_impl_handle;

  cleanup_data_impl_handle.cancel();

  return RCUTILS_RET_OK;
}

rcutils_ret_t
fastdds__dynamic_data_fini(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl)
{
  auto data_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  FASTDDS_CHECK_RET_FOR_NOT_OK_WITH_MSG(
    static_cast<fastdds__serialization_support_impl_handle_t *>(serialization_support_impl->handle)
    ->data_factory_->delete_data(data_handle->ref_type),
    "Could not fini data");
  delete data_handle;
  return RCUTILS_RET_OK;
}

// DYNAMIC DATA SERIALIZATION ======================================================================

// NOTE(methylDragon): This is implemented but not tested since its not used anywhere yet...
rcutils_ret_t
fastdds__dynamic_data_serialize(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rcutils_uint8_array_t * buffer)
{
  (void) serialization_support_impl;

  auto data_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  auto m_type = std::make_shared<eprosima::fastdds::dds::DynamicPubSubType>();
  size_t data_length =
    static_cast<size_t>(m_type->calculate_serialized_size(
      &data_handle->ref_type,
      eprosima::fastdds::dds::DataRepresentationId_t::XCDR_DATA_REPRESENTATION));

  if (buffer->buffer_capacity < data_length) {
    if (rcutils_uint8_array_resize(buffer, data_length) != RCUTILS_RET_OK) {
      RCUTILS_SET_ERROR_MSG("Could not resize buffer");
      return RCUTILS_RET_BAD_ALLOC;
    }
  }
  buffer->buffer_capacity = data_length;

  auto payload =
    std::make_shared<eprosima::fastdds::rtps::SerializedPayload_t>(
    fastdds__size_t_to_uint32_t(
      data_length));
  bool success = m_type->serialize(
    &data_handle->ref_type, *payload,
    eprosima::fastdds::dds::DataRepresentationId_t::XCDR_DATA_REPRESENTATION);  // Serialize into payload

  if (success) {
    buffer->buffer_length = payload->length;

    // Pass ownership of serialized buffer to buffer argument
    // NOTE(methylDragon): Dubious cast... (we're going from octet to uint8_t, but it's just bytes?)
    buffer->buffer = static_cast<uint8_t *>(payload->data);
    payload->data = nullptr;

    return RCUTILS_RET_OK;
  } else {
    // We don't modify the buffer beyond expanding it up there
    // And we leave the serialized buffer in the payload to be deallocated on its destruction
    RCUTILS_SET_ERROR_MSG("Could not serialize dynamic data");
    return RCUTILS_RET_ERROR;
  }
}

rcutils_ret_t
fastdds__dynamic_data_deserialize(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rcutils_uint8_array_t * buffer)
{
  (void) serialization_support_impl;

  auto data_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  auto payload = std::make_shared<eprosima::fastdds::rtps::SerializedPayload_t>(
    fastdds__size_t_to_uint32_t(buffer->buffer_length));

  // NOTE(methylDragon): Deserialize should copy at this point, so this copy is not needed, I think
  // memcpy(payload->data, buffer->buffer, buffer->buffer_length);

  payload->data = buffer->buffer;   // Use the input buffer directly without copying
  payload->length = fastdds__size_t_to_uint32_t(buffer->buffer_length);

  auto m_type = std::make_shared<eprosima::fastdds::dds::DynamicPubSubType>();
  bool success = m_type->deserialize(*payload, &data_handle->ref_type);

  // Deserializes payload into dynamic data. This copies!
  if (success) {
    payload->data = nullptr;     // Data gets freed on buffer fini outside
    return RCUTILS_RET_OK;
  } else {
    payload->data = nullptr;     // Data gets freed on buffer fini outside
    RCUTILS_SET_ERROR_MSG("Could not deserialize dynamic data");
    return RCUTILS_RET_ERROR;
  }
}

// DYNAMIC DATA PRIMITIVE MEMBER GETTERS ===========================================================
#define FASTDDS_DYNAMIC_DATA_GET_VALUE_FN(FunctionT, ValueT, DataFnT) \
  rcutils_ret_t fastdds__dynamic_data_get_ ## FunctionT ## _value( \
    rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, \
    const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, \
    rosidl_dynamic_typesupport_member_id_t id, \
    ValueT * value) \
  { \
    (void) serialization_support_impl; \
    auto data_handle = \
      static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle); \
    if (!data_handle || !data_handle->ref_type) { \
      RCUTILS_SET_ERROR_MSG("Could not get handle to data impl"); \
      return RCUTILS_RET_INVALID_ARGUMENT; \
    } \
    FASTDDS_CHECK_RET_FOR_NOT_OK_AND_RETURN_WITH_MSG( \
      data_handle->ref_type \
      ->get_ ## DataFnT ## _value(*value, fastdds__size_t_to_uint32_t(id)), \
      "Could not get `" #FunctionT "` value (of type `" #ValueT "`)"); \
  }

FASTDDS_DYNAMIC_DATA_GET_VALUE_FN(bool, bool, boolean)
FASTDDS_DYNAMIC_DATA_GET_VALUE_FN(byte, unsigned char, byte)
FASTDDS_DYNAMIC_DATA_GET_VALUE_FN(char, char, char8)
// FASTDDS_DYNAMIC_DATA_GET_VALUE_FN(wchar, char16_t, char16)
FASTDDS_DYNAMIC_DATA_GET_VALUE_FN(float32, float, float32)
FASTDDS_DYNAMIC_DATA_GET_VALUE_FN(float64, double, float64)
FASTDDS_DYNAMIC_DATA_GET_VALUE_FN(int8, int8_t, int8)
FASTDDS_DYNAMIC_DATA_GET_VALUE_FN(uint8, uint8_t, uint8)
FASTDDS_DYNAMIC_DATA_GET_VALUE_FN(int16, int16_t, int16)
FASTDDS_DYNAMIC_DATA_GET_VALUE_FN(uint16, uint16_t, uint16)
FASTDDS_DYNAMIC_DATA_GET_VALUE_FN(int32, int32_t, int32)
FASTDDS_DYNAMIC_DATA_GET_VALUE_FN(uint32, uint32_t, uint32)
FASTDDS_DYNAMIC_DATA_GET_VALUE_FN(int64, int64_t, int64)
FASTDDS_DYNAMIC_DATA_GET_VALUE_FN(uint64, uint64_t, uint64)
#undef FASTDDS_DYNAMIC_DATA_GET_VALUE_FN

// This needs something different to do the conversion out
rcutils_ret_t
fastdds__dynamic_data_get_wchar_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  char16_t * value)
{
  (void) serialization_support_impl;
  wchar_t tmp_out;

  auto data_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  FASTDDS_CHECK_RET_FOR_NOT_OK_WITH_MSG(
    data_handle->ref_type
    ->get_char16_value(tmp_out, fastdds__size_t_to_uint32_t(id)),
    "Could not get `wchar` value (of type `char16_t`)");

  *value = static_cast<char16_t>(tmp_out);
  return RCUTILS_RET_OK;
}

rcutils_ret_t
fastdds__dynamic_data_get_string_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  char ** value,
  size_t * value_length)
{
  (void) serialization_support_impl;
  std::string tmp_string;

  auto data_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  FASTDDS_CHECK_RET_FOR_NOT_OK_WITH_MSG(
    data_handle->ref_type
    ->get_string_value(tmp_string, fastdds__size_t_to_uint32_t(id)),
    "Could not get `string` value (of type `string`)");

  *value_length = tmp_string.size();
  char * tmp_out = new char[*value_length + 1];  // TODO(methylDragon): Use alloc here
  memcpy(tmp_out, tmp_string.c_str(), *value_length);
  tmp_out[*value_length] = '\0';
  *value = tmp_out;
  return RCUTILS_RET_OK;
}

rcutils_ret_t
fastdds__dynamic_data_get_wstring_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  char16_t ** value,
  size_t * value_length)
{
  (void) serialization_support_impl;
  std::wstring tmp_wstring;

  auto data_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  FASTDDS_CHECK_RET_FOR_NOT_OK_WITH_MSG(
    data_handle->ref_type
    ->get_wstring_value(tmp_wstring, fastdds__size_t_to_uint32_t(id)),
    "Could not get `wstring` value (of type `char16_t *`)");

  *value_length = tmp_wstring.size();
  char16_t * tmp_out = new char16_t[*value_length + 1];  // TODO(methylDragon): Use alloc here
  fastdds__ucsncpy(tmp_out, fastdds__wstring_to_u16string(tmp_wstring).c_str(), *value_length);
  tmp_out[*value_length] = '\0';
  *value = tmp_out;
  return RCUTILS_RET_OK;
}

rcutils_ret_t
fastdds__dynamic_data_get_fixed_string_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  char ** value,
  size_t * value_length,
  size_t string_length)
{
  (void) serialization_support_impl;
  std::string tmp_string;

  auto data_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  // On the wire it's a bounded string
  FASTDDS_CHECK_RET_FOR_NOT_OK_WITH_MSG(
    data_handle->ref_type
    ->get_string_value(tmp_string, fastdds__size_t_to_uint32_t(id)),
    "Could not get fixed `string` value (of type `char *`)");

  size_t copy_length = std::min(tmp_string.size(), string_length);
  *value_length = string_length;
  char * tmp_out = new char[*value_length + 1]();  // TODO(methylDragon): Use alloc here
  memcpy(tmp_out, tmp_string.c_str(), copy_length);
  tmp_out[*value_length] = '\0';
  *value = tmp_out;
  return RCUTILS_RET_OK;
}

rcutils_ret_t
fastdds__dynamic_data_get_fixed_wstring_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  char16_t ** value,
  size_t * value_length,
  size_t wstring_length)
{
  (void) serialization_support_impl;
  std::wstring tmp_wstring;

  auto data_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  // On the wire it's a bounded string
  FASTDDS_CHECK_RET_FOR_NOT_OK_WITH_MSG(
    data_handle->ref_type
    ->get_wstring_value(tmp_wstring, fastdds__size_t_to_uint32_t(id)),
    "Could not get fixed `wstring` value (of type `char16_t *`)");

  size_t copy_length = std::min(tmp_wstring.size(), wstring_length);
  *value_length = wstring_length;
  char16_t * tmp_out = new char16_t[*value_length + 1];  // TODO(methylDragon): Use alloc here
  fastdds__ucsncpy(tmp_out, fastdds__wstring_to_u16string(tmp_wstring).c_str(), copy_length);
  tmp_out[*value_length] = '\0';
  *value = tmp_out;
  return RCUTILS_RET_OK;
}

rcutils_ret_t
fastdds__dynamic_data_get_bounded_string_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  char ** value,
  size_t * value_length,
  size_t string_bound)
{
  (void) serialization_support_impl;
  std::string tmp_string;

  auto data_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  FASTDDS_CHECK_RET_FOR_NOT_OK_WITH_MSG(
    data_handle->ref_type
    ->get_string_value(tmp_string, fastdds__size_t_to_uint32_t(id)),
    "Could not get bounded `string` value (of type `char *`)");

  *value_length = std::min(tmp_string.size(), string_bound);
  char * tmp_out = new char[*value_length + 1];  // TODO(methylDragon): Use alloc here
  memcpy(tmp_out, tmp_string.c_str(), *value_length);
  tmp_out[*value_length] = '\0';
  *value = tmp_out;
  return RCUTILS_RET_OK;
}

rcutils_ret_t
fastdds__dynamic_data_get_bounded_wstring_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  char16_t ** value,
  size_t * value_length,
  size_t wstring_bound)
{
  (void) serialization_support_impl;
  std::wstring tmp_wstring;

  auto data_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  FASTDDS_CHECK_RET_FOR_NOT_OK_WITH_MSG(
    data_handle->ref_type
    ->get_wstring_value(tmp_wstring, fastdds__size_t_to_uint32_t(id)),
    "Could not get bounded `wstring` value (of type `char16_t *`)");

  *value_length = std::min(tmp_wstring.size(), wstring_bound);
  char16_t * tmp_out = new char16_t[*value_length + 1];  // TODO(methylDragon): Use alloc here
  fastdds__ucsncpy(tmp_out, fastdds__wstring_to_u16string(tmp_wstring).c_str(), *value_length);
  tmp_out[*value_length] = '\0';
  *value = tmp_out;
  return RCUTILS_RET_OK;
}

// DYNAMIC DATA PRIMITIVE MEMBER SETTERS ===========================================================
#define FASTDDS_DYNAMIC_DATA_SET_VALUE_FN(FunctionT, ValueT, DataFnT) \
  rcutils_ret_t fastdds__dynamic_data_set_ ## FunctionT ## _value( \
    rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, \
    rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, \
    rosidl_dynamic_typesupport_member_id_t id, \
    ValueT value) \
  { \
    (void) serialization_support_impl; \
    auto data_handle = \
      static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle); \
    if (!data_handle || !data_handle->ref_type) { \
      RCUTILS_SET_ERROR_MSG("Could not get handle to data impl"); \
      return RCUTILS_RET_INVALID_ARGUMENT; \
    } \
    FASTDDS_CHECK_RET_FOR_NOT_OK_AND_RETURN_WITH_MSG( \
      static_cast<DynamicData *>(data_impl->handle) \
      ->set_ ## DataFnT ## _value(fastdds__size_t_to_uint32_t(id), value), \
      "Could not set `" #FunctionT "` value (of type `" #ValueT "`)"); \
  }

FASTDDS_DYNAMIC_DATA_SET_VALUE_FN(bool, bool, boolean)
FASTDDS_DYNAMIC_DATA_SET_VALUE_FN(byte, unsigned char, byte)
FASTDDS_DYNAMIC_DATA_SET_VALUE_FN(char, char, char8)
FASTDDS_DYNAMIC_DATA_SET_VALUE_FN(wchar, char16_t, char16)
FASTDDS_DYNAMIC_DATA_SET_VALUE_FN(float32, float, float32)
FASTDDS_DYNAMIC_DATA_SET_VALUE_FN(float64, double, float64)
FASTDDS_DYNAMIC_DATA_SET_VALUE_FN(int8, int8_t, int8)    // NOTE!!
FASTDDS_DYNAMIC_DATA_SET_VALUE_FN(uint8, uint8_t, uint8) // NOTE!!
FASTDDS_DYNAMIC_DATA_SET_VALUE_FN(int16, int16_t, int16)
FASTDDS_DYNAMIC_DATA_SET_VALUE_FN(uint16, uint16_t, uint16)
FASTDDS_DYNAMIC_DATA_SET_VALUE_FN(int32, int32_t, int32)
FASTDDS_DYNAMIC_DATA_SET_VALUE_FN(uint32, uint32_t, uint32)
FASTDDS_DYNAMIC_DATA_SET_VALUE_FN(int64, int64_t, int64)
FASTDDS_DYNAMIC_DATA_SET_VALUE_FN(uint64, uint64_t, uint64)
#undef FASTDDS_DYNAMIC_DATA_SET_VALUE_FN

rcutils_ret_t
fastdds__dynamic_data_set_string_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * value,
  size_t value_length)
{
  (void) serialization_support_impl;

  auto data_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  const std::string tmp_string(value, value_length);
  FASTDDS_CHECK_RET_FOR_NOT_OK_AND_RETURN_WITH_MSG(
    data_handle->ref_type->set_string_value(
      fastdds__size_t_to_uint32_t(id),
      tmp_string),
    "Could not set `string` value (of type `char *`)");
}

rcutils_ret_t
fastdds__dynamic_data_set_wstring_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char16_t * value,
  size_t value_length)
{
  (void) serialization_support_impl;

  auto data_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  const std::u16string tmp_u16string(value, value_length);
  FASTDDS_CHECK_RET_FOR_NOT_OK_AND_RETURN_WITH_MSG(
    data_handle->ref_type
    ->set_wstring_value(
      fastdds__size_t_to_uint32_t(id),
      fastdds__u16string_to_wstring(tmp_u16string)),
    "Could not set `wstring` value (of type `char16_t *`)");
}

rcutils_ret_t
fastdds__dynamic_data_set_fixed_string_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * value,
  size_t value_length,
  size_t string_length)
{
  (void) serialization_support_impl;

  auto data_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  std::string tmp_string(value, std::min(value_length, string_length));
  tmp_string.resize(string_length, '\0');
  FASTDDS_CHECK_RET_FOR_NOT_OK_AND_RETURN_WITH_MSG(
    data_handle->ref_type->set_string_value(
      fastdds__size_t_to_uint32_t(id),
      tmp_string),
    "Could not set fixed `string` value (of type `char *`)");
}

rcutils_ret_t
fastdds__dynamic_data_set_fixed_wstring_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char16_t * value,
  size_t value_length,
  size_t wstring_length)
{
  (void) serialization_support_impl;

  auto data_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  std::u16string tmp_u16string(value, std::min(value_length, wstring_length));
  tmp_u16string.resize(wstring_length, '\0');
  FASTDDS_CHECK_RET_FOR_NOT_OK_AND_RETURN_WITH_MSG(
    data_handle->ref_type
    ->set_wstring_value(
      fastdds__size_t_to_uint32_t(id),
      fastdds__u16string_to_wstring(tmp_u16string)),
    "Could not set fixed `wstring` value (of type `char16_t *`)");
}

rcutils_ret_t
fastdds__dynamic_data_set_bounded_string_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char * value,
  size_t value_length,
  size_t string_bound)
{
  (void) serialization_support_impl;

  auto data_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  const std::string tmp_string(value, std::min(value_length, string_bound));
  FASTDDS_CHECK_RET_FOR_NOT_OK_AND_RETURN_WITH_MSG(
    data_handle->ref_type->set_string_value(
      fastdds__size_t_to_uint32_t(id),
      tmp_string),
    "Could not set bounded `string` value (of type `char *`)");
}

rcutils_ret_t
fastdds__dynamic_data_set_bounded_wstring_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  const char16_t * value,
  size_t value_length,
  size_t wstring_bound)
{
  (void) serialization_support_impl;

  auto data_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  const std::u16string tmp_u16string(value, std::min(value_length, wstring_bound));
  FASTDDS_CHECK_RET_FOR_NOT_OK_AND_RETURN_WITH_MSG(
    data_handle->ref_type
    ->set_wstring_value(
      fastdds__size_t_to_uint32_t(id),
      fastdds__u16string_to_wstring(tmp_u16string)),
    "Could not set bounded `wstring` value (of type `char16_t *`)");
}

// DYNAMIC DATA SEQUENCES ==========================================================================
rcutils_ret_t
fastdds__dynamic_data_clear_sequence_data(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl)
{
  (void) serialization_support_impl;

  auto data_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  FASTDDS_CHECK_RET_FOR_NOT_OK_AND_RETURN_WITH_MSG(
    data_handle->ref_type->clear_all_values(),
    "Could not clear sequence data");
}

rcutils_ret_t
fastdds__dynamic_data_remove_sequence_data(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;

  auto data_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  FASTDDS_CHECK_RET_FOR_NOT_OK_AND_RETURN_WITH_MSG(
    data_handle->ref_type->clear_value(
      fastdds__size_t_to_uint32_t(
        id)),
    "Could not remove sequence data");
}

rcutils_ret_t
fastdds__dynamic_data_insert_sequence_data(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (void) serialization_support_impl;

  auto data_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  auto sequence_length = data_handle->ref_type->get_item_count();
  eprosima::fastdds::dds::ReturnCode_t ret_code {eprosima::fastdds::dds::RETCODE_ERROR};

  switch (data_handle->ref_type->type()->get_kind()) {
    case eprosima::fastdds::dds::TK_ANNOTATION:
    case eprosima::fastdds::dds::TK_ARRAY:
    case eprosima::fastdds::dds::TK_BITMASK:
    case eprosima::fastdds::dds::TK_BITSET:
    case eprosima::fastdds::dds::TK_MAP:
    case eprosima::fastdds::dds::TK_SEQUENCE:
    case eprosima::fastdds::dds::TK_STRUCTURE:
    case eprosima::fastdds::dds::TK_UNION:
      {
        auto new_data = data_handle->ref_type->loan_value(sequence_length);
        if (nullptr != new_data) {
          ret_code = data_handle->ref_type->return_loaned_value(new_data);
        }
      }

      break;
    default:
      ret_code = data_handle->ref_type->set_byte_value(sequence_length, 0);
      break;
  }

  *out_id = sequence_length;

  return eprosima::fastdds::dds::RETCODE_OK == ret_code ? RCUTILS_RET_OK : RCUTILS_RET_ERROR;
}

#define FASTDDS_DYNAMIC_DATA_INSERT_VALUE_FN(FunctionT, ValueT, DataFnT) \
  rcutils_ret_t fastdds__dynamic_data_insert_ ## FunctionT ## _value( \
    rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, \
    rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, ValueT value, \
    rosidl_dynamic_typesupport_member_id_t * out_id) \
  { \
    (void) serialization_support_impl; \
    eprosima::fastdds::dds::MemberId tmp_id; \
 \
    auto data_handle = \
      static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle); \
    if (!data_handle || !data_handle->ref_type) { \
      RCUTILS_SET_ERROR_MSG("Could not get handle to data impl"); \
      return RCUTILS_RET_INVALID_ARGUMENT; \
    } \
    tmp_id = data_handle->ref_type->get_item_count(); \
    FASTDDS_CHECK_RET_FOR_NOT_OK_WITH_MSG( \
      data_handle->ref_type->set_ ## DataFnT ## _value(tmp_id, value), \
      "Could not insert `" #FunctionT "` value (of type `" #ValueT "`)"); \
    *out_id = tmp_id; \
    return RCUTILS_RET_OK; \
  }

FASTDDS_DYNAMIC_DATA_INSERT_VALUE_FN(bool, bool, boolean)
FASTDDS_DYNAMIC_DATA_INSERT_VALUE_FN(byte, unsigned char, byte)
FASTDDS_DYNAMIC_DATA_INSERT_VALUE_FN(char, char, char8)
FASTDDS_DYNAMIC_DATA_INSERT_VALUE_FN(wchar, char16_t, char16)
FASTDDS_DYNAMIC_DATA_INSERT_VALUE_FN(float32, float, float32)
FASTDDS_DYNAMIC_DATA_INSERT_VALUE_FN(float64, double, float64)
FASTDDS_DYNAMIC_DATA_INSERT_VALUE_FN(int8, int8_t, char8)  // There is no int8 method
FASTDDS_DYNAMIC_DATA_INSERT_VALUE_FN(uint8, uint8_t, byte) // There is no uint8 method
FASTDDS_DYNAMIC_DATA_INSERT_VALUE_FN(int16, int16_t, int16)
FASTDDS_DYNAMIC_DATA_INSERT_VALUE_FN(uint16, uint16_t, uint16)
FASTDDS_DYNAMIC_DATA_INSERT_VALUE_FN(int32, int32_t, int32)
FASTDDS_DYNAMIC_DATA_INSERT_VALUE_FN(uint32, uint32_t, uint32)
FASTDDS_DYNAMIC_DATA_INSERT_VALUE_FN(int64, int64_t, int64)
FASTDDS_DYNAMIC_DATA_INSERT_VALUE_FN(uint64, uint64_t, uint64)
#undef FASTDDS_DYNAMIC_DATA_INSERT_VALUE_FN

rcutils_ret_t
fastdds__dynamic_data_insert_string_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  const char * value,
  size_t value_length,
  rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (void) serialization_support_impl;
  eprosima::fastdds::dds::MemberId tmp_id;

  auto data_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  tmp_id = data_handle->ref_type->get_item_count();

  FASTDDS_CHECK_RET_FOR_NOT_OK_WITH_MSG(
    data_handle->ref_type->set_string_value(
      tmp_id, std::string(value, value_length) ),
    "Could not insert `string` value (of type `char *`)");
  *out_id = tmp_id;
  return RCUTILS_RET_OK;
}

rcutils_ret_t
fastdds__dynamic_data_insert_wstring_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  const char16_t * value,
  size_t value_length,
  rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (void) serialization_support_impl;
  eprosima::fastdds::dds::MemberId tmp_id;

  auto data_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  tmp_id = data_handle->ref_type->get_item_count();

  FASTDDS_CHECK_RET_FOR_NOT_OK_WITH_MSG(
    data_handle->ref_type
    ->set_wstring_value(
      tmp_id, fastdds__u16string_to_wstring(std::u16string(value, value_length))),
    "Could not insert `wstring` value (of type `char16_t *`)");
  *out_id = tmp_id;
  return RCUTILS_RET_OK;
}

rcutils_ret_t
fastdds__dynamic_data_insert_fixed_string_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  const char * value,
  size_t value_length,
  size_t string_length,
  rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (void) serialization_support_impl;
  eprosima::fastdds::dds::MemberId tmp_id;

  auto data_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  std::string tmp_string = std::string(value, std::min(value_length, string_length));
  tmp_string.resize(string_length, '\0');

  tmp_id = data_handle->ref_type->get_item_count();

  FASTDDS_CHECK_RET_FOR_NOT_OK_WITH_MSG(
    data_handle->ref_type->set_string_value(tmp_id, tmp_string),
    "Could not insert fixed `string` value (of type `char *`)");
  *out_id = tmp_id;
  return RCUTILS_RET_OK;
}

rcutils_ret_t
fastdds__dynamic_data_insert_fixed_wstring_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  const char16_t * value,
  size_t value_length,
  size_t wstring_length,
  rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (void) serialization_support_impl;
  eprosima::fastdds::dds::MemberId tmp_id;

  auto data_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  std::u16string tmp_ustring = std::u16string(value, std::min(value_length, wstring_length));
  tmp_ustring.resize(wstring_length, '\0');

  tmp_id = data_handle->ref_type->get_item_count();

  FASTDDS_CHECK_RET_FOR_NOT_OK_WITH_MSG(
    data_handle->ref_type
    ->set_wstring_value(tmp_id, fastdds__u16string_to_wstring(tmp_ustring) ),
    "Could not insert fixed `wstring` value (of type `char16_t *`)");
  *out_id = tmp_id;
  return RCUTILS_RET_OK;
}

rcutils_ret_t
fastdds__dynamic_data_insert_bounded_string_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  const char * value,
  size_t value_length,
  size_t string_bound,
  rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (void) serialization_support_impl;
  eprosima::fastdds::dds::MemberId tmp_id;

  auto data_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  tmp_id = data_handle->ref_type->get_item_count();

  FASTDDS_CHECK_RET_FOR_NOT_OK_WITH_MSG(
    data_handle->ref_type
    ->set_string_value(tmp_id, std::string(value, std::min(value_length, string_bound))),
    "Could not insert bounded `string` value (of type `char *`)");
  *out_id = tmp_id;
  return RCUTILS_RET_OK;
}

rcutils_ret_t
fastdds__dynamic_data_insert_bounded_wstring_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  const char16_t * value,
  size_t value_length,
  size_t wstring_bound,
  rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (void) serialization_support_impl;
  eprosima::fastdds::dds::MemberId tmp_id;

  auto data_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  tmp_id = data_handle->ref_type->get_item_count();

  FASTDDS_CHECK_RET_FOR_NOT_OK_WITH_MSG(
    data_handle->ref_type
    ->set_wstring_value(
      tmp_id,
      fastdds__u16string_to_wstring(std::u16string(value, std::min(value_length, wstring_bound)))
    ),
    "Could not insert bounded `wstring` value (of type `char16_t *`)");
  *out_id = tmp_id;
  return RCUTILS_RET_OK;
}

// DYNAMIC DATA NESTED =============================================================================
rcutils_ret_t
fastdds__dynamic_data_get_complex_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  rcutils_allocator_t * allocator,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * value)
{
  (void) serialization_support_impl;
  (void) allocator;

  auto data_handle =
    static_cast<const fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  auto value_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(value->handle);
  if (!value_handle || !value_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  FASTDDS_CHECK_RET_FOR_NOT_OK_AND_RETURN_WITH_MSG(
    data_handle->ref_type
    ->get_complex_value(value_handle->ref_type, fastdds__size_t_to_uint32_t(id)),
    "Could not get complex value");
}

rcutils_ret_t
fastdds__dynamic_data_set_complex_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_member_id_t id,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * value)
{
  (void) serialization_support_impl;

  auto data_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  auto value_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(value->handle);
  if (!value_handle || !value_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  FASTDDS_CHECK_RET_FOR_NOT_OK_AND_RETURN_WITH_MSG(
    data_handle->ref_type
    ->set_complex_value(fastdds__size_t_to_uint32_t(id), value_handle->ref_type),
    "Could not set complex value");
}

rcutils_ret_t
fastdds__dynamic_data_insert_complex_value_copy(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * value,
  rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (void) serialization_support_impl;
  eprosima::fastdds::dds::MemberId tmp_id;

  auto data_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  auto value_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(value->handle);
  if (!value_handle || !value_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  tmp_id = data_handle->ref_type->get_item_count();

  FASTDDS_CHECK_RET_FOR_NOT_OK_WITH_MSG(
    data_handle->ref_type
    ->set_complex_value(tmp_id, value_handle->ref_type),
    "Could not insert complex value copy");
  *out_id = tmp_id;
  return RCUTILS_RET_OK;
}

rcutils_ret_t
fastdds__dynamic_data_insert_complex_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * value,
  rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (void) serialization_support_impl;
  eprosima::fastdds::dds::MemberId tmp_id;

  auto data_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(data_impl->handle);
  if (!data_handle || !data_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  auto value_handle =
    static_cast<fastdds__rosidl_dynamic_typesupport_dynamic_data_impl *>(value->handle);
  if (!value_handle || !value_handle->ref_type) {
    RCUTILS_SET_ERROR_MSG("Could not get handle to data impl");
    return RCUTILS_RET_INVALID_ARGUMENT;
  }

  tmp_id = data_handle->ref_type->get_item_count();

  FASTDDS_CHECK_RET_FOR_NOT_OK_WITH_MSG(
    data_handle->ref_type
    ->set_complex_value(tmp_id, value_handle->ref_type),
    "Could not insert complex value");
  *out_id = tmp_id;
  return RCUTILS_RET_OK;
}
