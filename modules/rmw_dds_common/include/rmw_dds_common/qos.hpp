// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#ifndef RMW_DDS_COMMON__QOS_HPP_
#define RMW_DDS_COMMON__QOS_HPP_

#include <functional>
#include <string>

#include "rmw/qos_profiles.h"
#include "rmw/topic_endpoint_info_array.h"
#include "rmw/types.h"

#include "rmw_dds_common/visibility_control.h"

namespace rmw_dds_common
{

/// Check if two QoS profiles are compatible
/**
 * Two QoS profiles are compatible if a publisher and subcription
 * using the QoS policies can communicate with each other.
 *
 * This implements the rmw API \ref rmw_qos_profile_check_compatible().
 * See \ref rmw_qos_profile_check_compatible() for more information.
 *
 * \param[in] publisher_qos: The QoS profile used for a publisher.
 * \param[in] subscription_qos: The QoS profile used for a subscription.
 * \param[out] compatibility: `RMW_QOS_COMPATIBILITY_OK` if the QoS profiles are compatible, or
 *   `RMW_QOS_COMPATIBILITY_WARNING` if the QoS profiles might be compatible, or
 *   `RMW_QOS_COMPATIBILITY_ERROR` if the QoS profiles are not compatible.
 * \param[out] reason: A detailed reason for a QoS incompatibility or potential incompatibility.
 *   Must be pre-allocated by the caller.
 *   This parameter is optional and may be set to `nullptr` if the reason information is not
 *   desired.
 * \param[in] reason_size: Size of the string buffer `reason`, if one is provided.
 *   If `reason` is `nullptr`, then this parameter must be zero.
 * \return `RMW_RET_OK` if the check was successful, or
 * \return `RMW_RET_INVALID_ARGUMENT` if `compatiblity` is `nullptr`, or
 * \return `RMW_RET_INVALID_ARGUMENT` if `reason` is `nullptr` and  `reason_size` is not zero, or
 * \return `RMW_RET_ERROR` if there is an unexpected error.
 */
RMW_DDS_COMMON_PUBLIC
rmw_ret_t
qos_profile_check_compatible(
  const rmw_qos_profile_t publisher_qos,
  const rmw_qos_profile_t subscription_qos,
  rmw_qos_compatibility_type_t * compatibility,
  char * reason,
  size_t reason_size);

/// Get the best available QoS policies for a subscription.
/**
 * Given zero or more publisher endpoints, update any BEST_AVAILABLE policies in a subscription QoS
 * profile such that is matches all publishers while maintaining the highest level of service.
 *
 * If reliability is BEST_AVAILABLE, then it will be set to RELIABLE if all publishers are
 * RELIABLE.
 * Otherwise, reliability will be set to BEST_EFFORT.
 *
 * If durability is BEST_AVAILABLE, then it will be set to TRANSIENT_LOCAL if all publishers are
 * TRANSIENT_LOCAL.
 * Otherwise, durability will be set to VOLATILE.
 *
 * If liveliness is BEST_AVAILABLE, then it will be set to MANUAL_BY_TOPIC if all publishers are
 * MANUAL_BY_TOPIC.
 * Otherwise, liveliness will be set to AUTOMATIC.
 *
 * If deadline is BEST_AVAILABLE, then it will be set to DEFAULT if all publishers are DEFAULT.
 * Otherwise, deadline will be set to the maximum deadline of all publishers.
 *
 * If liveliness lease duration is BEST_AVAILABLE, then it will be set to DEFAULT if all
 * publishers are DEFAULT.
 * Otherwise, liveliness lease duration will be set to the maximum deadline of all publishers.
 *
 * History, history depth, and lifespan policies are not changed by this function.
 *
 * \param[in] publishers_info: Endpoint information for publishers.
 * \param[out] subscription_profile: QoS profile that is compatible with the majority of
 *   the input publishers.
 * \return `RMW_RET_OK` if the operation was successful, or
 * \return `RMW_RET_INVALID_ARGUMENT` if `publishers_info` is `nullptr`, or
 * \return `RMW_RET_INVALID_ARGUMENT` if `subscription_profile` is `nullptr`, or
 * \return `RMW_RET_ERROR` if there is an unexpected error.
 */
RMW_DDS_COMMON_PUBLIC
rmw_ret_t
qos_profile_get_best_available_for_subscription(
  const rmw_topic_endpoint_info_array_t * publishers_info,
  rmw_qos_profile_t * subscription_profile);

/// Get the best available QoS policies for a publisher.
/**
 * Given zero or more subscription endpoints, update any BEST_AVAILABLE policies in a publisher QoS
 * profile such that is matches all subscriptions while maintaining the highest level of service.
 *
 * If reliability is BEST_AVAILABLE, then it will be set to RELIABLE.
 *
 * If durability is BEST_AVAILABLE, then it will be set to TRANSIENT_LOCAL.
 *
 * If liveliness is BEST_AVAILABLE, then it will be set to MANUAL_BY_TOPIC if at least one
 * subscription is MANUAL_BY_TOPIC.
 * Otherwise, liveliness will be set to AUTOMATIC.
 *
 * If deadline is BEST_AVAILABLE, then it will be set to DEFAULT if all subscriptions are DEFAULT.
 * Otherwise, deadline will be set to the minimum deadline of all subscriptions.
 *
 * If liveliness lease duration is BEST_AVAILABLE, then it will be set to DEFAULT if all
 * subscriptions are DEFAULT.
 * Otherwise, liveliness lease duration will be set to the mininum deadline of all subscriptions.
 *
 * History, history depth, and lifespan policies are not changed by this function.
 *
 * \param[in] subscriptions_info: Endpoint information for subscriptions.
 * \param[out] publisher_profile: QoS profile that is compatible with the majority of
 *   the input subscriptions.
 * \return `RMW_RET_OK` if the operation was successful, or
 * \return `RMW_RET_INVALID_ARGUMENT` if `subscriptions_info` is `nullptr`, or
 * \return `RMW_RET_INVALID_ARGUMENT` if `publisher_profile` is `nullptr`, or
 * \return `RMW_RET_ERROR` if there is an unexpected error.
 */
RMW_DDS_COMMON_PUBLIC
rmw_ret_t
qos_profile_get_best_available_for_publisher(
  const rmw_topic_endpoint_info_array_t * subscriptions_info,
  rmw_qos_profile_t * publisher_profile);

/// Signature matching rmw_get_publishers_info_by_topic and rmw_get_subscriptions_info_by_topic
using GetEndpointInfoByTopicFunction = std::function<rmw_ret_t(
      const rmw_node_t *,
      rcutils_allocator_t *,
      const char *,
      bool,
      rmw_topic_endpoint_info_array_t *)>;

/// Update a subscription QoS profile so that it is compatible with discovered publishers.
/**
 * If any policies in `qos_profile` are set to BEST_AVAILABLE, then `get_endpoint_info` will
 * be called to query endpoint information for adapting the QoS policies.
 *
 * For rules related to adapting 'best available' policies, see
 * \ref `qos_profile_get_best_available_for_subscription`.
 *
 * This function allocates memory with the node's context allocator.
 *
 * \param[in] node: Node used to query the graph.
 * \param[in] topic_name: Query info for publishers on this topic name.
 * \param[inout] qos_profile: Any policies that are set to 'best available' will by updated based
 *   on publisher endpoint QoS policies.
 * \param[in] get_endpoint_info: The function used to query for publisher endpoint information.
 *   I.e. an implementation of `rmw_get_publishers_info_by_topic`.
 * \return `RMW_RET_OK` if the operation was successful, or
 * \return `RMW_RET_INVALID_ARGUMENT` if `node` is `nullptr`, or
 * \return `RMW_RET_INVALID_ARGUMENT` if `topic_name` is `nullptr`, or
 * \return `RMW_RET_INVALID_ARGUMENT` if `qos_profile` is `nullptr`, or
 * \return `RMW_RET_INCORRECT_RMW_IMPLEMENTATION` if the `node` implementation
 *   identifier does not match this implementation, or
 * \return `RMW_RET_BAD_ALLOC` if memory allocation fails, or
 * \return `RMW_RET_ERROR` if there is an unexpected error.
 */
RMW_DDS_COMMON_PUBLIC
rmw_ret_t
qos_profile_get_best_available_for_topic_subscription(
  const rmw_node_t * node,
  const char * topic_name,
  rmw_qos_profile_t * qos_profile,
  const GetEndpointInfoByTopicFunction & get_endpoint_info);

/// Update a publisher QoS profile so that it is compatible with discovered subscriptions.
/**
 * If any policies in `qos_profile` are set to 'best available', then `get_endpoint_info` will
 * be called to query endpoint information for adapting the QoS policies.
 *
 * For rules related to adapting 'best available' policies, see
 * \ref `qos_profile_get_best_available_for_publisher`.
 *
 * This function allocates memory with the node's context allocator.
 *
 * \param[in] node: Node used to query the graph.
 * \param[in] topic_name: Query info for subscriptions on this topic name.
 * \param[inout] qos_profile: Any policies that are set to 'best available' will by updated based
 *   on subscription endpoint QoS policies.
 * \param[in] get_endpoint_info: The function used to query for subscription endpoint information.
 *   I.e. an implementation of `rmw_get_subscriptions_info_by_topic`.
 * \return `RMW_RET_OK` if the operation was successful, or
 * \return `RMW_RET_INVALID_ARGUMENT` if `node` is `nullptr`, or
 * \return `RMW_RET_INVALID_ARGUMENT` if `topic_name` is `nullptr`, or
 * \return `RMW_RET_INVALID_ARGUMENT` if `qos_profile` is `nullptr`, or
 * \return `RMW_RET_INCORRECT_RMW_IMPLEMENTATION` if the `node` implementation
 *   identifier does not match this implementation, or
 * \return `RMW_RET_BAD_ALLOC` if memory allocation fails, or
 * \return `RMW_RET_ERROR` if there is an unexpected error.
 */
RMW_DDS_COMMON_PUBLIC
rmw_ret_t
qos_profile_get_best_available_for_topic_publisher(
  const rmw_node_t * node,
  const char * topic_name,
  rmw_qos_profile_t * qos_profile,
  const GetEndpointInfoByTopicFunction & get_endpoint_info);

/// Update best available QoS policies for services and clients.
/**
 * Give QoS policies, return a new set of policies that have any BEST_AVAILABLE policies replaced
 * with default policies for services.
 *
 * See `rmw_qos_profile_services_default` for default policy values.
 *
 * \param[in] qos_profile: QoS profile to copy and update.
 * \return A copy of the input QoS profile with any BEST_AVAILABLE policies overwritten with
 *   default service policies.
 */
RMW_DDS_COMMON_PUBLIC
rmw_qos_profile_t
qos_profile_update_best_available_for_services(const rmw_qos_profile_t & qos_profile);

/// Parse USER_DATA "key=value;key=value;"" encoding, finding value of key "typehash"
/**
 * \param[in] user_data USER_DATA qos raw bytes
 * \param[in] user_data_size Length of user_data
 * \param[out] type_hash_out Filled with type hash data if found, or to zero value if key not found
 * \return RMW_RET_OK if key parsed successfully, or if key not found
 * \return RMW_RET_INVALID_ARGUMENT if user_data is null
 * \return RMW_RET_ERROR if typehash key found, but value could not be parsed
 */
RMW_DDS_COMMON_PUBLIC
rmw_ret_t
parse_type_hash_from_user_data(
  const uint8_t * user_data,
  size_t user_data_size,
  rosidl_type_hash_t & type_hash_out);

/// Encode type hash as "typehash=hash_string;" for use in USER_DATA QoS
/**
 * \param[in] type_hash Type hash value to encode
 * \param[out] string_out On success, will be set to "typehash=stringified_type_hash;"
 *   If type_hash's version is 0, string_out will be set to empty
 * \return RMW_RET_OK on success, including empty string for unset version
 * \return RMW_RET_BAD_ALLOC if memory allocation fails
 */
RMW_DDS_COMMON_PUBLIC
rmw_ret_t
encode_type_hash_for_user_data_qos(
  const rosidl_type_hash_t & type_hash,
  std::string & string_out);

}  // namespace rmw_dds_common

#endif  // RMW_DDS_COMMON__QOS_HPP_
