// Copyright 2014-2015 Open Source Robotics Foundation, Inc.
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

#ifndef ROSIDL_TYPESUPPORT_FASTRTPS_CPP__MESSAGE_TYPE_SUPPORT_H_
#define ROSIDL_TYPESUPPORT_FASTRTPS_CPP__MESSAGE_TYPE_SUPPORT_H_

#include <cstddef>

#include "fastcdr/Cdr.h"

#include "rosidl_runtime_c/message_type_support_struct.h"

/// Feature define to allow API version detection
#define ROSIDL_TYPESUPPORT_FASTRTPS_HAS_PLAIN_TYPES

#define ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE 0x00
#define ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE   0x01
#define ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE     0x03

// Holds generated methods related with keys
typedef struct message_type_support_key_callbacks_t
{
  /// Callback function to determine the maximum size needed for key serialization,
  /// which is used for key type support initialization.
  /**
   * \param [in,out] is_unbounded Whether the key has any unbounded member.
   * \return The maximum key serialized size, in bytes.
   */
  size_t (* max_serialized_size_key)(
    bool & is_unbounded);

  /// Callback function to get size of the key data
  /**
   * \param [in] untyped_ros_message Type erased pointer to message instance
   * \return The size of the serialized key in bytes.
   */
  size_t (* get_serialized_size_key)(
    const void * untyped_ros_message);

  /// Callback function for key serialization
  /**
   * \param[in] untyped_ros_message Type erased pointer to message instance.
   * \param [in,out] cdr Fast CDR serializer.
   * \return true if serialization succeeded, false otherwise.
   */
  bool (* cdr_serialize_key)(
    const void * untyped_ros_message,
    eprosima::fastcdr::Cdr & cdr);
} message_type_support_key_callbacks_t;

/// Encapsulates the callbacks for getting properties of this rosidl type.
/**
 * These callbacks are implemented in the generated sources.
 */
typedef struct message_type_support_callbacks_t
{
  /// The C++ namespace of this message.
  const char * message_namespace_;

  /// The typename of this message.
  const char * message_name_;

  /// Callback function for message serialization
  /**
   * \param[in] untyped_ros_message Type erased pointer to message instance.
   * \param [in,out] Serialized FastCDR data object.
   * \return true if serialization succeeded, false otherwise.
   */
  bool (* cdr_serialize)(
    const void * untyped_ros_message,
    eprosima::fastcdr::Cdr & cdr);

  /// Callback function for message deserialization
  /**
   * \param [in] Serialized FastCDR data object.
   * \param[out] untyped_ros_message Type erased pointer to message instance.
   * \return true if deserialization succeeded, false otherwise.
   */
  bool (* cdr_deserialize)(
    eprosima::fastcdr::Cdr & cdr,
    void * untyped_ros_message);

  /// Callback function to get size of data
  /**
   * \return The size of the serialized message in bytes.
   */
  uint32_t (* get_serialized_size)(const void *);

  /// Callback function to determine the maximum size needed for serialization, which is used for
  /// type support initialization.
  /**
   * \param[out] bounds_info Bounds information for the type.
   *                         May return one of the following values:
   *                         - \c ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE for POD types
   *                         - \c ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE for fully bounded types,
   *                           (i.e. not unbounded strings or sequences).
   *                         - \c ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE for unbounded types
   * \return The maximum serialized size, in bytes.
   */
  size_t (* max_serialized_size)(char & bounds_info);

  /// Pointer to the message_type_support_key_callbacks_t.
  /// Nullptr if the type is not keyed.
  message_type_support_key_callbacks_t * key_callbacks;
} message_type_support_callbacks_t;

#endif  // ROSIDL_TYPESUPPORT_FASTRTPS_CPP__MESSAGE_TYPE_SUPPORT_H_
