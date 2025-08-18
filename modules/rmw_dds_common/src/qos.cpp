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

#include "rmw_dds_common/qos.hpp"

#include <cstdarg>
#include <cstring>
#include <string>
#include <vector>

#include "rcpputils/scope_exit.hpp"
#include "rcutils/error_handling.h"
#include "rcutils/snprintf.h"
#include "rmw/error_handling.h"
#include "rmw/impl/cpp/key_value.hpp"
#include "rmw/get_topic_endpoint_info.h"
#include "rmw/qos_profiles.h"
#include "rmw/qos_string_conversions.h"

namespace rmw_dds_common
{

static bool
operator==(rmw_time_t t1, rmw_time_t t2)
{
  return t1.sec == t2.sec && t1.nsec == t2.nsec;
}

static bool
operator!=(rmw_time_t t1, rmw_time_t t2)
{
  return !(t1 == t2);
}

static bool
operator<(rmw_time_t t1, rmw_time_t t2)
{
  if (t1.sec < t2.sec) {
    return true;
  } else if (t1.sec == t2.sec && t1.nsec < t2.nsec) {
    return true;
  }
  return false;
}

static const rmw_time_t deadline_default = RMW_QOS_DEADLINE_DEFAULT;
static const rmw_time_t deadline_best_available = RMW_QOS_DEADLINE_BEST_AVAILABLE;
static const rmw_time_t liveliness_lease_duration_default =
  RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT;
static const rmw_time_t liveliness_lease_duration_best_available =
  RMW_QOS_LIVELINESS_LEASE_DURATION_BEST_AVAILABLE;

// Returns RMW_RET_OK if successful or no buffer was provided
// Returns RMW_RET_ERROR if there as an error copying the message to the buffer
static rmw_ret_t
_append_to_buffer(char * buffer, size_t buffer_size, const char * format, ...)
{
  // Only write if a buffer is provided
  if (!buffer || buffer_size == 0u) {
    return RMW_RET_OK;
  }
  // Determine available space left in buffer
  size_t offset = strnlen(buffer, buffer_size);
  size_t write_size = buffer_size - offset;
  std::va_list args;
  va_start(args, format);
  int snprintf_ret = rcutils_vsnprintf(buffer + offset, write_size, format, args);
  va_end(args);
  if (snprintf_ret < 0) {
    RMW_SET_ERROR_MSG("failed to append to character buffer");
    return RMW_RET_ERROR;
  }
  return RMW_RET_OK;
}

rmw_ret_t
qos_profile_check_compatible(
  const rmw_qos_profile_t publisher_qos,
  const rmw_qos_profile_t subscription_qos,
  rmw_qos_compatibility_type_t * compatibility,
  char * reason,
  size_t reason_size)
{
  if (!compatibility) {
    RMW_SET_ERROR_MSG("compatibility parameter is null");
    return RMW_RET_INVALID_ARGUMENT;
  }

  if (!reason && reason_size != 0u) {
    RMW_SET_ERROR_MSG("reason parameter is null, but reason_size parameter is not zero");
    return RMW_RET_INVALID_ARGUMENT;
  }

  // Presume profiles are compatible until proven otherwise
  *compatibility = RMW_QOS_COMPATIBILITY_OK;

  // Initialize reason buffer
  if (reason && reason_size != 0u) {
    reason[0] = '\0';
  }

  // Best effort publisher and reliable subscription
  if (publisher_qos.reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT &&
    subscription_qos.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE)
  {
    *compatibility = RMW_QOS_COMPATIBILITY_ERROR;
    rmw_ret_t append_ret = _append_to_buffer(
      reason,
      reason_size,
      "ERROR: Best effort publisher and reliable subscription;");
    if (RMW_RET_OK != append_ret) {
      return append_ret;
    }
  }

  // Volatile publisher and transient local subscription
  if (publisher_qos.durability == RMW_QOS_POLICY_DURABILITY_VOLATILE &&
    subscription_qos.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
  {
    *compatibility = RMW_QOS_COMPATIBILITY_ERROR;
    rmw_ret_t append_ret = _append_to_buffer(
      reason,
      reason_size,
      "ERROR: Volatile publisher and transient local subscription;");
    if (RMW_RET_OK != append_ret) {
      return append_ret;
    }
  }

  const rmw_time_t & pub_deadline = publisher_qos.deadline;
  const rmw_time_t & sub_deadline = subscription_qos.deadline;

  // No deadline for publisher and deadline for subscription
  if (pub_deadline == deadline_default && sub_deadline != deadline_default) {
    *compatibility = RMW_QOS_COMPATIBILITY_ERROR;
    rmw_ret_t ret = _append_to_buffer(
      reason,
      reason_size,
      "ERROR: Subscription has a deadline, but publisher does not;");
    if (RMW_RET_OK != ret) {
      return ret;
    }
  }

  // Subscription deadline is less than publisher deadline
  if (pub_deadline != deadline_default && sub_deadline != deadline_default) {
    if (sub_deadline < pub_deadline) {
      *compatibility = RMW_QOS_COMPATIBILITY_ERROR;
      rmw_ret_t append_ret = _append_to_buffer(
        reason,
        reason_size,
        "ERROR: Subscription deadline is less than publisher deadline;");
      if (RMW_RET_OK != append_ret) {
        return append_ret;
      }
    }
  }

  // Automatic liveliness for publisher and manual by topic for subscription
  if (publisher_qos.liveliness == RMW_QOS_POLICY_LIVELINESS_AUTOMATIC &&
    subscription_qos.liveliness == RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC)
  {
    *compatibility = RMW_QOS_COMPATIBILITY_ERROR;
    rmw_ret_t append_ret = _append_to_buffer(
      reason,
      reason_size,
      "ERROR: Publisher's liveliness is automatic and subscription's is manual by topic;");
    if (RMW_RET_OK != append_ret) {
      return append_ret;
    }
  }

  const rmw_time_t & pub_lease = publisher_qos.liveliness_lease_duration;
  const rmw_time_t & sub_lease = subscription_qos.liveliness_lease_duration;

  // No lease duration for publisher and lease duration for subscription
  if (pub_lease == liveliness_lease_duration_default &&
    sub_lease != liveliness_lease_duration_default)
  {
    *compatibility = RMW_QOS_COMPATIBILITY_ERROR;
    rmw_ret_t append_ret = _append_to_buffer(
      reason,
      reason_size,
      "ERROR: Subscription has a liveliness lease duration, but publisher does not;");
    if (RMW_RET_OK != append_ret) {
      return append_ret;
    }
  }

  // Subscription lease duration is less than publisher lease duration
  if (pub_lease != liveliness_lease_duration_default &&
    sub_lease != liveliness_lease_duration_default)
  {
    if (sub_lease < pub_lease) {
      *compatibility = RMW_QOS_COMPATIBILITY_ERROR;
      rmw_ret_t append_ret = _append_to_buffer(
        reason,
        reason_size,
        "ERROR: Subscription liveliness lease duration is less than publisher;");
      if (RMW_RET_OK != append_ret) {
        return append_ret;
      }
    }
  }

  // Only check for warnings if there are no errors
  if (RMW_QOS_COMPATIBILITY_OK == *compatibility) {
    // We don't know the policy if the value is "system default" or "unknown"
    const bool pub_reliability_unknown =
      publisher_qos.reliability == RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT ||
      publisher_qos.reliability == RMW_QOS_POLICY_RELIABILITY_UNKNOWN;
    const bool sub_reliability_unknown =
      subscription_qos.reliability == RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT ||
      subscription_qos.reliability == RMW_QOS_POLICY_RELIABILITY_UNKNOWN;
    const bool pub_durability_unknown =
      publisher_qos.durability == RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT ||
      publisher_qos.durability == RMW_QOS_POLICY_DURABILITY_UNKNOWN;
    const bool sub_durability_unknown =
      subscription_qos.durability == RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT ||
      subscription_qos.durability == RMW_QOS_POLICY_DURABILITY_UNKNOWN;
    const bool pub_liveliness_unknown =
      publisher_qos.liveliness == RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT ||
      publisher_qos.liveliness == RMW_QOS_POLICY_LIVELINESS_UNKNOWN;
    const bool sub_liveliness_unknown =
      subscription_qos.liveliness == RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT ||
      subscription_qos.liveliness == RMW_QOS_POLICY_LIVELINESS_UNKNOWN;

    const char * pub_reliability_str = rmw_qos_reliability_policy_to_str(publisher_qos.reliability);
    if (!pub_reliability_str) {
      pub_reliability_str = "unknown";
    }
    const char * sub_reliability_str = rmw_qos_reliability_policy_to_str(
      subscription_qos.reliability);
    if (!sub_reliability_str) {
      sub_reliability_str = "unknown";
    }
    const char * pub_durability_str = rmw_qos_durability_policy_to_str(publisher_qos.durability);
    if (!pub_durability_str) {
      pub_durability_str = "unknown";
    }
    const char * sub_durability_str = rmw_qos_durability_policy_to_str(subscription_qos.durability);
    if (!sub_durability_str) {
      sub_durability_str = "unknown";
    }
    const char * pub_liveliness_str = rmw_qos_liveliness_policy_to_str(publisher_qos.liveliness);
    if (!pub_liveliness_str) {
      pub_liveliness_str = "unknown";
    }
    const char * sub_liveliness_str = rmw_qos_liveliness_policy_to_str(subscription_qos.liveliness);
    if (!sub_liveliness_str) {
      sub_liveliness_str = "unknown";
    }

    // Reliability warnings
    if (pub_reliability_unknown && sub_reliability_unknown) {
      // Reliability for publisher and subscription is unknown
      *compatibility = RMW_QOS_COMPATIBILITY_WARNING;
      rmw_ret_t append_ret = _append_to_buffer(
        reason,
        reason_size,
        "WARNING: Publisher reliability is %s and subscription reliability is %s;",
        pub_reliability_str,
        sub_reliability_str);
      if (RMW_RET_OK != append_ret) {
        return append_ret;
      }
    } else if (pub_reliability_unknown &&  // NOLINT
      subscription_qos.reliability == RMW_QOS_POLICY_RELIABILITY_RELIABLE)
    {
      // Reliability for publisher is unknown and subscription is reliable
      *compatibility = RMW_QOS_COMPATIBILITY_WARNING;
      rmw_ret_t append_ret = _append_to_buffer(
        reason,
        reason_size,
        "WARNING: Reliable subscription, but publisher is %s;",
        pub_reliability_str);
      if (RMW_RET_OK != append_ret) {
        return append_ret;
      }
    } else if (publisher_qos.reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT &&  // NOLINT
      sub_reliability_unknown)
    {
      // Reliability for publisher is best effort and subscription is unknown
      *compatibility = RMW_QOS_COMPATIBILITY_WARNING;
      rmw_ret_t append_ret = _append_to_buffer(
        reason,
        reason_size,
        "WARNING: Best effort publisher, but subscription is %s;",
        sub_reliability_str);
      if (RMW_RET_OK != append_ret) {
        return append_ret;
      }
    }

    // Durability warnings
    if (pub_durability_unknown && sub_durability_unknown) {
      // Durability for publisher and subscription is unknown
      *compatibility = RMW_QOS_COMPATIBILITY_WARNING;
      rmw_ret_t append_ret = _append_to_buffer(
        reason,
        reason_size,
        "WARNING: Publisher durabilty is %s and subscription durability is %s;",
        pub_durability_str,
        sub_durability_str);
      if (RMW_RET_OK != append_ret) {
        return append_ret;
      }
    } else if (pub_durability_unknown &&  // NOLINT
      subscription_qos.durability == RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
    {
      // Durability for publisher is unknown and subscription is transient local
      *compatibility = RMW_QOS_COMPATIBILITY_WARNING;
      rmw_ret_t ret = _append_to_buffer(
        reason,
        reason_size,
        "WARNING: Transient local subscription, but publisher is %s;",
        pub_durability_str);
      if (RMW_RET_OK != ret) {
        return ret;
      }
    } else if (publisher_qos.durability == RMW_QOS_POLICY_DURABILITY_VOLATILE &&  // NOLINT
      sub_durability_unknown)
    {
      // Durability for publisher is volatile and subscription is unknown
      *compatibility = RMW_QOS_COMPATIBILITY_WARNING;
      rmw_ret_t ret = _append_to_buffer(
        reason,
        reason_size,
        "WARNING: Volatile publisher, but subscription is %s;",
        sub_durability_str);
      if (RMW_RET_OK != ret) {
        return ret;
      }
    }

    // Liveliness warnings
    if (pub_liveliness_unknown && sub_liveliness_unknown) {
      // Liveliness for publisher and subscription is unknown
      *compatibility = RMW_QOS_COMPATIBILITY_WARNING;
      rmw_ret_t append_ret = _append_to_buffer(
        reason,
        reason_size,
        "WARNING: Publisher liveliness is %s and subscription liveliness is %s;",
        pub_liveliness_str,
        sub_liveliness_str);
      if (RMW_RET_OK != append_ret) {
        return append_ret;
      }
    } else if (pub_liveliness_unknown &&  // NOLINT
      subscription_qos.liveliness == RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC)
    {
      // Unknown liveliness for publisher and manual by topic for subscription
      *compatibility = RMW_QOS_COMPATIBILITY_WARNING;
      rmw_ret_t ret = _append_to_buffer(
        reason,
        reason_size,
        "WARNING: Subscription's liveliness is manual by topic, but publisher's is %s;",
        pub_liveliness_str);
      if (RMW_RET_OK != ret) {
        return ret;
      }
    } else if (publisher_qos.liveliness == RMW_QOS_POLICY_LIVELINESS_AUTOMATIC &&  // NOLINT
      sub_liveliness_unknown)
    {
      // Automatic liveliness for publisher and unknown for subscription
      *compatibility = RMW_QOS_COMPATIBILITY_WARNING;
      rmw_ret_t ret = _append_to_buffer(
        reason,
        reason_size,
        "WARNING: Publisher's liveliness is automatic, but subscription's is %s;",
        sub_liveliness_str);
      if (RMW_RET_OK != ret) {
        return ret;
      }
    }
  }

  return RMW_RET_OK;
}

rmw_ret_t
qos_profile_get_best_available_for_subscription(
  const rmw_topic_endpoint_info_array_t * publishers_info,
  rmw_qos_profile_t * subscription_profile)
{
  if (!publishers_info) {
    RMW_SET_ERROR_MSG("publishers_info parameter is null");
    return RMW_RET_INVALID_ARGUMENT;
  }
  if (!subscription_profile) {
    RMW_SET_ERROR_MSG("subscription_profile parameter is null");
    return RMW_RET_INVALID_ARGUMENT;
  }

  // Only use "reliable" reliability if all publisher profiles are reliable
  // Only use "transient local" durability if all publisher profiles are transient local
  // Only use "manual by topic" liveliness if all publisher profiles are manual by topic
  // Use default deadline if all publishers have default deadline, otherwise use largest deadline
  // Use default lease duration if all publishers have default lease, otherwise use largest lease
  size_t number_of_reliable = 0u;
  size_t number_of_transient_local = 0u;
  size_t number_of_manual_by_topic = 0u;
  bool use_default_deadline = true;
  rmw_time_t largest_deadline = {0u, 0u};
  bool use_default_liveliness_lease_duration = true;
  rmw_time_t largest_liveliness_lease_duration = {0u, 0u};
  for (size_t i = 0u; i < publishers_info->size; ++i) {
    const rmw_qos_profile_t & profile = publishers_info->info_array[i].qos_profile;
    if (RMW_QOS_POLICY_RELIABILITY_RELIABLE == profile.reliability) {
      number_of_reliable++;
    }
    if (RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL == profile.durability) {
      number_of_transient_local++;
    }
    if (RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC == profile.liveliness) {
      number_of_manual_by_topic++;
    }
    if (profile.deadline != deadline_default) {
      use_default_deadline = false;
      if (largest_deadline < profile.deadline) {
        largest_deadline = profile.deadline;
      }
    }
    if (profile.liveliness_lease_duration != liveliness_lease_duration_default) {
      use_default_liveliness_lease_duration = false;
      if (largest_liveliness_lease_duration < profile.liveliness_lease_duration) {
        largest_liveliness_lease_duration = profile.liveliness_lease_duration;
      }
    }
  }

  if (RMW_QOS_POLICY_RELIABILITY_BEST_AVAILABLE == subscription_profile->reliability) {
    if (number_of_reliable == publishers_info->size) {
      subscription_profile->reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    } else {
      subscription_profile->reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    }
  }

  if (RMW_QOS_POLICY_DURABILITY_BEST_AVAILABLE == subscription_profile->durability) {
    if (number_of_transient_local == publishers_info->size) {
      subscription_profile->durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    } else {
      subscription_profile->durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    }
  }

  if (RMW_QOS_POLICY_LIVELINESS_BEST_AVAILABLE == subscription_profile->liveliness) {
    if (number_of_manual_by_topic == publishers_info->size) {
      subscription_profile->liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
    } else {
      subscription_profile->liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
    }
  }

  if (deadline_best_available == subscription_profile->deadline) {
    if (use_default_deadline) {
      subscription_profile->deadline = RMW_QOS_DEADLINE_DEFAULT;
    } else {
      subscription_profile->deadline = largest_deadline;
    }
  }

  if (liveliness_lease_duration_best_available == subscription_profile->liveliness_lease_duration) {
    if (use_default_liveliness_lease_duration) {
      subscription_profile->liveliness_lease_duration = RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT;
    } else {
      subscription_profile->liveliness_lease_duration = largest_liveliness_lease_duration;
    }
  }

  return RMW_RET_OK;
}

rmw_ret_t
qos_profile_get_best_available_for_publisher(
  const rmw_topic_endpoint_info_array_t * subscriptions_info,
  rmw_qos_profile_t * publisher_profile)
{
  if (!subscriptions_info) {
    RMW_SET_ERROR_MSG("subscriptions_info parameter is null");
    return RMW_RET_INVALID_ARGUMENT;
  }
  if (!publisher_profile) {
    RMW_SET_ERROR_MSG("publisher_profile parameter is null");
    return RMW_RET_INVALID_ARGUMENT;
  }

  // Always use "reliable" reliability and "transient_local" durability since both policies
  // are compatible with all subscriptions and have highest level of service
  if (RMW_QOS_POLICY_RELIABILITY_BEST_AVAILABLE == publisher_profile->reliability) {
    publisher_profile->reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  }
  if (RMW_QOS_POLICY_DURABILITY_BEST_AVAILABLE == publisher_profile->durability) {
    publisher_profile->durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  }

  // Only use "manual by topic" liveliness if at least one  subscription is using manual by topic
  // Use default deadline if all subscriptions have default deadline, otherwise use smallest
  // Use default lease duration if all subscriptions have default lease, otherwise use smallest
  bool use_manual_by_topic = false;
  bool use_default_deadline = true;
  rmw_time_t smallest_deadline = RMW_DURATION_INFINITE;
  bool use_default_liveliness_lease_duration = true;
  rmw_time_t smallest_liveliness_lease_duration = RMW_DURATION_INFINITE;
  for (size_t i = 0u; i < subscriptions_info->size; ++i) {
    const rmw_qos_profile_t & profile = subscriptions_info->info_array[i].qos_profile;
    if (RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC == profile.liveliness) {
      use_manual_by_topic = true;
    }
    if (profile.deadline != deadline_default) {
      use_default_deadline = false;
      if (profile.deadline < smallest_deadline) {
        smallest_deadline = profile.deadline;
      }
    }
    if (profile.liveliness_lease_duration != liveliness_lease_duration_default) {
      use_default_liveliness_lease_duration = false;
      if (profile.liveliness_lease_duration < smallest_liveliness_lease_duration) {
        smallest_liveliness_lease_duration = profile.liveliness_lease_duration;
      }
    }
  }

  if (RMW_QOS_POLICY_LIVELINESS_BEST_AVAILABLE == publisher_profile->liveliness) {
    if (use_manual_by_topic) {
      publisher_profile->liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
    } else {
      publisher_profile->liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
    }
  }

  if (deadline_best_available == publisher_profile->deadline) {
    if (use_default_deadline) {
      publisher_profile->deadline = RMW_QOS_DEADLINE_DEFAULT;
    } else {
      publisher_profile->deadline = smallest_deadline;
    }
  }

  if (liveliness_lease_duration_best_available == publisher_profile->liveliness_lease_duration) {
    if (use_default_liveliness_lease_duration) {
      publisher_profile->liveliness_lease_duration = RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT;
    } else {
      publisher_profile->liveliness_lease_duration = smallest_liveliness_lease_duration;
    }
  }

  return RMW_RET_OK;
}

static bool
_qos_profile_has_best_available_policy(const rmw_qos_profile_t & qos_profile)
{
  if (RMW_QOS_POLICY_RELIABILITY_BEST_AVAILABLE == qos_profile.reliability) {
    return true;
  }
  if (RMW_QOS_POLICY_DURABILITY_BEST_AVAILABLE == qos_profile.durability) {
    return true;
  }
  if (RMW_QOS_POLICY_LIVELINESS_BEST_AVAILABLE == qos_profile.liveliness) {
    return true;
  }
  if (deadline_best_available == qos_profile.deadline) {
    return true;
  }
  if (liveliness_lease_duration_best_available == qos_profile.liveliness_lease_duration) {
    return true;
  }
  return false;
}

rmw_ret_t
qos_profile_get_best_available_for_topic_subscription(
  const rmw_node_t * node,
  const char * topic_name,
  rmw_qos_profile_t * qos_profile,
  const GetEndpointInfoByTopicFunction & get_endpoint_info)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_profile, RMW_RET_INVALID_ARGUMENT);

  if (_qos_profile_has_best_available_policy(*qos_profile)) {
    rcutils_allocator_t & allocator = node->context->options.allocator;
    rmw_topic_endpoint_info_array_t publishers_info =
      rmw_get_zero_initialized_topic_endpoint_info_array();
    rmw_ret_t ret = get_endpoint_info(
      node, &allocator, topic_name, false, &publishers_info);
    if (RMW_RET_OK != ret) {
      return ret;
    }
    ret = qos_profile_get_best_available_for_subscription(
      &publishers_info, qos_profile);
    rmw_ret_t fini_ret = rmw_topic_endpoint_info_array_fini(&publishers_info, &allocator);
    if (RMW_RET_OK != fini_ret) {
      return fini_ret;
    }
    if (RMW_RET_OK != ret) {
      return ret;
    }
  }
  return RMW_RET_OK;
}

rmw_ret_t
qos_profile_get_best_available_for_topic_publisher(
  const rmw_node_t * node,
  const char * topic_name,
  rmw_qos_profile_t * qos_profile,
  const GetEndpointInfoByTopicFunction & get_endpoint_info)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_profile, RMW_RET_INVALID_ARGUMENT);

  if (_qos_profile_has_best_available_policy(*qos_profile)) {
    rcutils_allocator_t & allocator = node->context->options.allocator;
    rmw_topic_endpoint_info_array_t subscriptions_info =
      rmw_get_zero_initialized_topic_endpoint_info_array();
    rmw_ret_t ret = get_endpoint_info(
      node, &allocator, topic_name, false, &subscriptions_info);
    if (RMW_RET_OK != ret) {
      return ret;
    }
    ret = qos_profile_get_best_available_for_publisher(
      &subscriptions_info, qos_profile);
    rmw_ret_t fini_ret = rmw_topic_endpoint_info_array_fini(&subscriptions_info, &allocator);
    if (RMW_RET_OK != fini_ret) {
      return fini_ret;
    }
    if (RMW_RET_OK != ret) {
      return ret;
    }
  }
  return RMW_RET_OK;
}

rmw_qos_profile_t
qos_profile_update_best_available_for_services(const rmw_qos_profile_t & qos_profile)
{
  rmw_qos_profile_t result = qos_profile;
  if (RMW_QOS_POLICY_RELIABILITY_BEST_AVAILABLE == result.reliability) {
    result.reliability = rmw_qos_profile_services_default.reliability;
  }
  if (RMW_QOS_POLICY_DURABILITY_BEST_AVAILABLE == result.durability) {
    result.durability = rmw_qos_profile_services_default.durability;
  }
  if (RMW_QOS_POLICY_LIVELINESS_BEST_AVAILABLE == result.liveliness) {
    result.liveliness = rmw_qos_profile_services_default.liveliness;
  }
  if (deadline_best_available == result.deadline) {
    result.deadline = rmw_qos_profile_services_default.deadline;
  }
  if (liveliness_lease_duration_best_available == result.liveliness_lease_duration) {
    result.liveliness_lease_duration = rmw_qos_profile_services_default.liveliness_lease_duration;
  }
  return result;
}

rmw_ret_t
parse_type_hash_from_user_data(
  const uint8_t * user_data,
  size_t user_data_size,
  rosidl_type_hash_t & type_hash_out)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(user_data, RMW_RET_INVALID_ARGUMENT);
  std::vector<uint8_t> udvec(user_data, user_data + user_data_size);
  auto key_value = rmw::impl::cpp::parse_key_value(udvec);
  auto typehash_it = key_value.find("typehash");
  if (typehash_it == key_value.end()) {
    type_hash_out = rosidl_get_zero_initialized_type_hash();
    return RMW_RET_OK;
  }
  std::string type_hash_str(typehash_it->second.begin(), typehash_it->second.end());
  if (RCUTILS_RET_OK != rosidl_parse_type_hash_string(type_hash_str.c_str(), &type_hash_out)) {
    return RMW_RET_ERROR;
  }
  return RMW_RET_OK;
}

rmw_ret_t
encode_type_hash_for_user_data_qos(
  const rosidl_type_hash_t & type_hash,
  std::string & string_out)
{
  if (type_hash.version == ROSIDL_TYPE_HASH_VERSION_UNSET) {
    string_out.clear();
    return RMW_RET_OK;
  }
  auto allocator = rcutils_get_default_allocator();
  char * type_hash_c_str = nullptr;
  rcutils_ret_t stringify_ret = rosidl_stringify_type_hash(&type_hash, allocator, &type_hash_c_str);
  if (RCUTILS_RET_BAD_ALLOC == stringify_ret) {
    return RMW_RET_BAD_ALLOC;
  }
  if (RCUTILS_RET_OK != stringify_ret) {
    return RMW_RET_ERROR;
  }
  RCPPUTILS_SCOPE_EXIT(allocator.deallocate(type_hash_c_str, &allocator.state));
  string_out = "typehash=" + std::string(type_hash_c_str) + ";";
  return RMW_RET_OK;
}

}  // namespace rmw_dds_common
