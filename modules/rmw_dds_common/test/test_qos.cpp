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

#include <gtest/gtest.h>

#include <cstring>

#include "osrf_testing_tools_cpp/scope_exit.hpp"
#include "rcpputils/scope_exit.hpp"
#include "rmw/error_handling.h"
#include "rmw/qos_profiles.h"
#include "rmw/types.h"

#include "rmw_dds_common/qos.hpp"

static rmw_qos_profile_t
get_qos_profile_fixture()
{
  return {
    RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    5,
    RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    RMW_QOS_POLICY_DURABILITY_VOLATILE,
    RMW_QOS_DEADLINE_DEFAULT,
    RMW_QOS_LIFESPAN_DEFAULT,
    RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
    RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
    false
  };
}

TEST(test_qos, test_qos_profile_check_compatible_reliability)
{
  // Reliable pub, reliable sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    sub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_OK);
    EXPECT_EQ(0u, strnlen(reason, 1));
  }

  // Reliable pub, best effort sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    sub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_OK);
    EXPECT_EQ(0u, strnlen(reason, 1));
  }

  // Best effort pub, best effort sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    sub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_OK);
    EXPECT_EQ(0u, strnlen(reason, 1));
  }

  // Best effort pub, reliable sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    sub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_ERROR);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
}

TEST(test_qos, test_qos_profile_check_compatible_durability)
{
  // Volatile pub, volatile sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    sub_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_OK);
    EXPECT_EQ(0u, strnlen(reason, 1));
  }
  // Volatile pub, transient local sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    sub_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_ERROR);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Transient local pub, transient local sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    sub_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_OK);
    EXPECT_EQ(0u, strnlen(reason, 1));
  }
  // Transient local pub, volatile sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    sub_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_OK);
    EXPECT_EQ(0u, strnlen(reason, 1));
  }
}

TEST(test_qos, test_qos_profile_check_compatible_deadline)
{
  // No deadline pub, no deadline sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.deadline = RMW_QOS_DEADLINE_DEFAULT;
    sub_qos.deadline = RMW_QOS_DEADLINE_DEFAULT;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_OK);
    EXPECT_EQ(0u, strnlen(reason, 1));
  }

  // No deadline pub, deadline sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.deadline = RMW_QOS_DEADLINE_DEFAULT;
    sub_qos.deadline = {1, 0};
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_ERROR);
    EXPECT_LT(0u, strnlen(reason, 1));
  }

  // Deadline pub > deadline sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.deadline = {1, 1};
    sub_qos.deadline = {1, 0};
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_ERROR);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Deadline pub == deadline sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.deadline = {1, 1};
    sub_qos.deadline = {1, 1};
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_OK);
    EXPECT_EQ(0u, strnlen(reason, 1));
  }
  // Deadline pub < deadline sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.deadline = {1, 1};
    sub_qos.deadline = {2, 0};
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_OK);
    EXPECT_EQ(0u, strnlen(reason, 1));
  }
}

TEST(test_qos, test_qos_profile_check_compatible_liveliness)
{
  // Automatic pub, automatic sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
    sub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_OK);
    EXPECT_EQ(0u, strnlen(reason, 1));
  }
  // Automatic pub, manual sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
    sub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_ERROR);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Manual pub, manual sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
    sub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_OK);
    EXPECT_EQ(0u, strnlen(reason, 1));
  }
  // Manual pub, automatic sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
    sub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_OK);
    EXPECT_EQ(0u, strnlen(reason, 1));
  }
}

TEST(test_qos, test_qos_profile_check_compatible_liveliness_lease_duration)
{
  // No duration pub, no duration sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.liveliness_lease_duration = RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT;
    sub_qos.liveliness_lease_duration = RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_OK);
    EXPECT_EQ(0u, strnlen(reason, 1));
  }
  // No duration pub, some duration sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.liveliness_lease_duration = RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT;
    sub_qos.liveliness_lease_duration = {1, 0};
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_ERROR);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Lease duration pub == lease duration sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.liveliness_lease_duration = {1, 0};
    sub_qos.liveliness_lease_duration = {1, 0};
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_OK);
    EXPECT_EQ(0u, strnlen(reason, 1));
  }
  // Lease duration pub > lease duration sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.liveliness_lease_duration = {1, 1};
    sub_qos.liveliness_lease_duration = {1, 0};
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_ERROR);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Lease duration pub < lease duration sub
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.liveliness_lease_duration = {1, 1};
    sub_qos.liveliness_lease_duration = {2, 1};
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_OK);
    EXPECT_EQ(0u, strnlen(reason, 1));
  }
}

TEST(test_qos, test_qos_profile_check_compatible_system_default)
{
  // Some policies set to "system default" should cause a warning

  // Pub best effort, sub system default
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    sub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Pub system default, sub reliable
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
    sub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Both reliability system default
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
    sub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Pub volatile, sub system default
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    sub_qos.durability = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Pub system default, sub transient local
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.durability = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
    sub_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Both durability system default
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.durability = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
    sub_qos.durability = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Pub automatic, sub system default
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
    sub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Pub system default, sub manual by topic
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT;
    sub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Both liveliness system default
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT;
    sub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
}

TEST(test_qos, test_qos_profile_check_compatible_unknown)
{
  // Some policies set to "unknown" should cause a warning

  // Pub best effort, sub unknown
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    sub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_UNKNOWN;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Pub unknown, sub reliable
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_UNKNOWN;
    sub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Both reliability unknown
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_UNKNOWN;
    sub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_UNKNOWN;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Pub volatile, sub unknown
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
    sub_qos.durability = RMW_QOS_POLICY_DURABILITY_UNKNOWN;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Pub unknown, sub transient local
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.durability = RMW_QOS_POLICY_DURABILITY_UNKNOWN;
    sub_qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Both durability unknown
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.durability = RMW_QOS_POLICY_DURABILITY_UNKNOWN;
    sub_qos.durability = RMW_QOS_POLICY_DURABILITY_UNKNOWN;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Pub automatic, sub unknown
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
    sub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_UNKNOWN;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Pub unknown, sub manual by topic
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_UNKNOWN;
    sub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
  // Both liveliness unknown
  {
    rmw_qos_compatibility_type_t compatible;
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_UNKNOWN;
    sub_qos.liveliness = RMW_QOS_POLICY_LIVELINESS_UNKNOWN;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_WARNING);
    EXPECT_LT(0u, strnlen(reason, 1));
  }
}

TEST(test_qos, test_qos_profile_check_compatible_no_reason)
{
  // Compatible
  {
    rmw_qos_compatibility_type_t compatible;
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, nullptr, 0u);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_OK);
  }
  // Incompatible
  {
    rmw_qos_compatibility_type_t compatible;
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    pub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    sub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, nullptr, 0u);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_ERROR);
  }
}

TEST(test_qos, test_qos_profile_check_compatible_reason_buffer_too_small)
{
  // We expect a message larger than 10 characters
  char reason[10];
  size_t reason_size = 10u;
  rmw_qos_compatibility_type_t compatible;
  rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
  rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
  pub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  sub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
    pub_qos, sub_qos, &compatible, reason, reason_size);
  EXPECT_EQ(ret, RMW_RET_OK);
  EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_ERROR);
  // Expect the first 10 characters including the terminating null character
  EXPECT_STREQ(reason, "ERROR: Be");
}

TEST(test_qos, test_qos_profile_check_compatible_reason_buffer_size_zero)
{
  char reason[10] = "untouched";
  // With reason size zero, we don't expect any message to be written
  size_t reason_size = 0u;
  rmw_qos_compatibility_type_t compatible;
  rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
  rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
  pub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  sub_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
    pub_qos, sub_qos, &compatible, reason, reason_size);
  EXPECT_EQ(ret, RMW_RET_OK);
  EXPECT_EQ(compatible, RMW_QOS_COMPATIBILITY_ERROR);
  EXPECT_STREQ(reason, "untouched");
}

TEST(test_qos, test_qos_profile_check_compatible_invalid)
{
  // Null compatible parameter
  {
    const size_t reason_size = 2048u;
    char reason[2048];
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, nullptr, reason, reason_size);
    EXPECT_EQ(ret, RMW_RET_INVALID_ARGUMENT);
    rmw_reset_error();
  }
  // Null reason, but non-zero reason_size
  {
    rmw_qos_compatibility_type_t compatible;
    rmw_qos_profile_t pub_qos = get_qos_profile_fixture();
    rmw_qos_profile_t sub_qos = get_qos_profile_fixture();
    rmw_ret_t ret = rmw_dds_common::qos_profile_check_compatible(
      pub_qos, sub_qos, &compatible, nullptr, 3u);
    EXPECT_EQ(ret, RMW_RET_INVALID_ARGUMENT);
    rmw_reset_error();
  }
}

TEST(test_qos, test_qos_profile_get_best_available_for_subscription)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  // Zero publisher endpoints
  {
    rmw_topic_endpoint_info_array_t zeroed_publishers_info =
      rmw_get_zero_initialized_topic_endpoint_info_array();
    rmw_qos_profile_t subscription_profile = rmw_qos_profile_best_available;
    rmw_ret_t ret = rmw_dds_common::qos_profile_get_best_available_for_subscription(
      &zeroed_publishers_info, &subscription_profile);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(subscription_profile.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(subscription_profile.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    EXPECT_EQ(subscription_profile.liveliness, RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
    EXPECT_TRUE(
      rmw_time_equal(
        subscription_profile.liveliness_lease_duration,
        RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT));
    EXPECT_TRUE(rmw_time_equal(subscription_profile.deadline, RMW_QOS_DEADLINE_DEFAULT));
  }
  // One publisher endpoint
  {
    rmw_topic_endpoint_info_array_t publishers_info =
      rmw_get_zero_initialized_topic_endpoint_info_array();
    rmw_ret_t init_ret = rmw_topic_endpoint_info_array_init_with_size(
      &publishers_info, 1u, &allocator);
    ASSERT_EQ(init_ret, RMW_RET_OK);
    OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
    {
      EXPECT_EQ(rmw_topic_endpoint_info_array_fini(&publishers_info, &allocator), RMW_RET_OK);
    });

    publishers_info.info_array[0].qos_profile = {
      RMW_QOS_POLICY_HISTORY_KEEP_ALL,
      1,
      RMW_QOS_POLICY_RELIABILITY_RELIABLE,
      RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
      RMW_QOS_DEADLINE_DEFAULT,
      RMW_QOS_LIFESPAN_DEFAULT,
      RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC,
      RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
      false
    };
    rmw_qos_profile_t subscription_profile = rmw_qos_profile_best_available;
    rmw_ret_t ret = rmw_dds_common::qos_profile_get_best_available_for_subscription(
      &publishers_info, &subscription_profile);

    EXPECT_EQ(ret, RMW_RET_OK);
    // Expect changes to match publisher QoS
    const rmw_qos_profile_t & publisher_profile = publishers_info.info_array[0].qos_profile;
    EXPECT_EQ(subscription_profile.reliability, publisher_profile.reliability);
    EXPECT_EQ(subscription_profile.durability, publisher_profile.durability);
    EXPECT_EQ(subscription_profile.liveliness, publisher_profile.liveliness);
    EXPECT_TRUE(
      rmw_time_equal(
        subscription_profile.liveliness_lease_duration,
        publisher_profile.liveliness_lease_duration));
    EXPECT_TRUE(rmw_time_equal(subscription_profile.deadline, publisher_profile.deadline));
  }
  // More than one publisher endpoint
  {
    rmw_topic_endpoint_info_array_t publishers_info =
      rmw_get_zero_initialized_topic_endpoint_info_array();
    rmw_ret_t init_ret = rmw_topic_endpoint_info_array_init_with_size(
      &publishers_info, 3u, &allocator);
    ASSERT_EQ(init_ret, RMW_RET_OK);
    OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
    {
      EXPECT_EQ(rmw_topic_endpoint_info_array_fini(&publishers_info, &allocator), RMW_RET_OK);
    });

    publishers_info.info_array[0].qos_profile = {
      RMW_QOS_POLICY_HISTORY_KEEP_ALL,
      1,
      RMW_QOS_POLICY_RELIABILITY_RELIABLE,
      RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
      RMW_QOS_DEADLINE_DEFAULT,
      RMW_QOS_LIFESPAN_DEFAULT,
      RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,  // should result in "automatic" for subscription
      {1u, 0u},
      false
    };
    publishers_info.info_array[1].qos_profile = {
      RMW_QOS_POLICY_HISTORY_KEEP_ALL,
      1,
      RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,  // should result in "best effort" for subscription
      RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
      {3u, 0u},  // this deadline should appear in subscription QoS because it is the largest
      RMW_QOS_LIFESPAN_DEFAULT,
      RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC,
      RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
      false
    };
    publishers_info.info_array[2].qos_profile = {
      RMW_QOS_POLICY_HISTORY_KEEP_ALL,
      1,
      RMW_QOS_POLICY_RELIABILITY_RELIABLE,
      RMW_QOS_POLICY_DURABILITY_VOLATILE,  // should result in "volatile" for subscription
      {2u, 0u},
      RMW_QOS_LIFESPAN_DEFAULT,
      RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC,
      {2u, 0u},  // should appear in subscription QoS because it is the largest
      false
    };
    rmw_qos_profile_t subscription_profile = rmw_qos_profile_best_available;
    rmw_ret_t ret = rmw_dds_common::qos_profile_get_best_available_for_subscription(
      &publishers_info, &subscription_profile);

    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(subscription_profile.reliability, RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    EXPECT_EQ(subscription_profile.durability, RMW_QOS_POLICY_DURABILITY_VOLATILE);
    EXPECT_EQ(subscription_profile.liveliness, RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
    EXPECT_TRUE(rmw_time_equal(subscription_profile.liveliness_lease_duration, {2u, 0u}));
    EXPECT_TRUE(rmw_time_equal(subscription_profile.deadline, {3u, 0u}));
  }
}

TEST(test_qos, test_qos_profile_get_best_available_for_subscription_invalid_arguments)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rmw_topic_endpoint_info_array_t publishers_info =
    rmw_get_zero_initialized_topic_endpoint_info_array();
  rmw_ret_t init_ret = rmw_topic_endpoint_info_array_init_with_size(
    &publishers_info, 1u, &allocator);
  ASSERT_EQ(init_ret, RMW_RET_OK);
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
  {
    EXPECT_EQ(rmw_topic_endpoint_info_array_fini(&publishers_info, &allocator), RMW_RET_OK);
  });
  rmw_qos_profile_t subscription_profile = rmw_qos_profile_best_available;

  // NULL publishers_info
  {
    rmw_ret_t ret = rmw_dds_common::qos_profile_get_best_available_for_subscription(
      nullptr, &subscription_profile);
    EXPECT_EQ(ret, RMW_RET_INVALID_ARGUMENT);
    rmw_reset_error();
  }
  // NULL subscription profile
  {
    rmw_ret_t ret = rmw_dds_common::qos_profile_get_best_available_for_subscription(
      &publishers_info, nullptr);
    EXPECT_EQ(ret, RMW_RET_INVALID_ARGUMENT);
    rmw_reset_error();
  }
}

TEST(test_qos, test_qos_profile_get_best_available_for_publisher)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  // Zero subscription endpoints
  {
    rmw_topic_endpoint_info_array_t zeroed_subscriptions_info =
      rmw_get_zero_initialized_topic_endpoint_info_array();
    rmw_qos_profile_t publisher_profile = rmw_qos_profile_best_available;
    rmw_ret_t ret = rmw_dds_common::qos_profile_get_best_available_for_publisher(
      &zeroed_subscriptions_info, &publisher_profile);
    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(publisher_profile.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(publisher_profile.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    EXPECT_EQ(publisher_profile.liveliness, RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
    EXPECT_TRUE(
      rmw_time_equal(
        publisher_profile.liveliness_lease_duration,
        RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT));
    EXPECT_TRUE(rmw_time_equal(publisher_profile.deadline, RMW_QOS_DEADLINE_DEFAULT));
  }
  // One subscription endpoint
  {
    rmw_topic_endpoint_info_array_t subscriptions_info =
      rmw_get_zero_initialized_topic_endpoint_info_array();
    rmw_ret_t init_ret = rmw_topic_endpoint_info_array_init_with_size(
      &subscriptions_info, 1u, &allocator);
    ASSERT_EQ(init_ret, RMW_RET_OK);
    OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
    {
      EXPECT_EQ(rmw_topic_endpoint_info_array_fini(&subscriptions_info, &allocator), RMW_RET_OK);
    });

    subscriptions_info.info_array[0].qos_profile = {
      RMW_QOS_POLICY_HISTORY_KEEP_ALL,
      1,
      RMW_QOS_POLICY_RELIABILITY_RELIABLE,
      RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
      RMW_QOS_DEADLINE_DEFAULT,
      RMW_QOS_LIFESPAN_DEFAULT,
      RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC,
      RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
      false
    };
    rmw_qos_profile_t publisher_profile = rmw_qos_profile_best_available;
    rmw_ret_t ret = rmw_dds_common::qos_profile_get_best_available_for_publisher(
      &subscriptions_info, &publisher_profile);

    EXPECT_EQ(ret, RMW_RET_OK);
    // Expect changes to match subscription QoS
    const rmw_qos_profile_t & subscription_profile = subscriptions_info.info_array[0].qos_profile;
    EXPECT_EQ(publisher_profile.reliability, subscription_profile.reliability);
    EXPECT_EQ(publisher_profile.durability, subscription_profile.durability);
    EXPECT_EQ(publisher_profile.liveliness, subscription_profile.liveliness);
    EXPECT_TRUE(
      rmw_time_equal(
        publisher_profile.liveliness_lease_duration,
        subscription_profile.liveliness_lease_duration));
    EXPECT_TRUE(rmw_time_equal(publisher_profile.deadline, subscription_profile.deadline));
  }
  // More than one subscription endpoint
  {
    rmw_topic_endpoint_info_array_t subscriptions_info =
      rmw_get_zero_initialized_topic_endpoint_info_array();
    rmw_ret_t init_ret = rmw_topic_endpoint_info_array_init_with_size(
      &subscriptions_info, 3u, &allocator);
    ASSERT_EQ(init_ret, RMW_RET_OK);
    OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
    {
      EXPECT_EQ(rmw_topic_endpoint_info_array_fini(&subscriptions_info, &allocator), RMW_RET_OK);
    });

    subscriptions_info.info_array[0].qos_profile = {
      RMW_QOS_POLICY_HISTORY_KEEP_ALL,
      1,
      RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
      RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,  // should result in "transient local" for pub
      RMW_QOS_DEADLINE_DEFAULT,
      RMW_QOS_LIFESPAN_DEFAULT,
      RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
      {1u, 0u},  // should appear in publisher QoS because it is the smallest
      false
    };
    subscriptions_info.info_array[1].qos_profile = {
      RMW_QOS_POLICY_HISTORY_KEEP_ALL,
      1,
      RMW_QOS_POLICY_RELIABILITY_RELIABLE,  // should result in "reliable" for publisher
      RMW_QOS_POLICY_DURABILITY_VOLATILE,
      {3u, 0u},
      RMW_QOS_LIFESPAN_DEFAULT,
      RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC,  // should result in "manual by topic" for pub
      RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
      false
    };
    subscriptions_info.info_array[2].qos_profile = {
      RMW_QOS_POLICY_HISTORY_KEEP_ALL,
      1,
      RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
      RMW_QOS_POLICY_DURABILITY_VOLATILE,
      {2u, 0u},  // this deadline should appear in publisher QoS because it is the smallest
      RMW_QOS_LIFESPAN_DEFAULT,
      RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
      {2u, 0u},
      false
    };
    rmw_qos_profile_t publisher_profile = rmw_qos_profile_best_available;
    rmw_ret_t ret = rmw_dds_common::qos_profile_get_best_available_for_publisher(
      &subscriptions_info, &publisher_profile);

    EXPECT_EQ(ret, RMW_RET_OK);
    EXPECT_EQ(publisher_profile.reliability, RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    EXPECT_EQ(publisher_profile.durability, RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    EXPECT_EQ(publisher_profile.liveliness, RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
    EXPECT_TRUE(rmw_time_equal(publisher_profile.liveliness_lease_duration, {1u, 0u}));
    EXPECT_TRUE(rmw_time_equal(publisher_profile.deadline, {2u, 0u}));
  }
}

TEST(test_qos, test_qos_profile_get_best_available_for_publisher_invalid_arguments)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rmw_topic_endpoint_info_array_t subscriptions_info =
    rmw_get_zero_initialized_topic_endpoint_info_array();
  rmw_ret_t init_ret = rmw_topic_endpoint_info_array_init_with_size(
    &subscriptions_info, 1u, &allocator);
  ASSERT_EQ(init_ret, RMW_RET_OK);
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
  {
    EXPECT_EQ(rmw_topic_endpoint_info_array_fini(&subscriptions_info, &allocator), RMW_RET_OK);
  });
  rmw_qos_profile_t publisher_profile = rmw_qos_profile_best_available;

  // NULL subscriptions_info
  {
    rmw_ret_t ret = rmw_dds_common::qos_profile_get_best_available_for_publisher(
      nullptr, &publisher_profile);
    EXPECT_EQ(ret, RMW_RET_INVALID_ARGUMENT);
    rmw_reset_error();
  }
  // NULL publisher profile
  {
    rmw_ret_t ret = rmw_dds_common::qos_profile_get_best_available_for_subscription(
      &subscriptions_info, nullptr);
    EXPECT_EQ(ret, RMW_RET_INVALID_ARGUMENT);
    rmw_reset_error();
  }
}

TEST(test_qos, test_qos_profile_update_best_available_for_services)
{
  rmw_qos_profile_t input_profile = rmw_qos_profile_best_available;
  rmw_qos_profile_t output_profile =
    rmw_dds_common::qos_profile_update_best_available_for_services(input_profile);
  EXPECT_EQ(rmw_qos_profile_services_default.reliability, output_profile.reliability);
  EXPECT_EQ(rmw_qos_profile_services_default.durability, output_profile.durability);
  EXPECT_EQ(rmw_qos_profile_services_default.liveliness, output_profile.liveliness);
  EXPECT_TRUE(rmw_time_equal(rmw_qos_profile_services_default.deadline, output_profile.deadline));
  EXPECT_TRUE(
    rmw_time_equal(
      rmw_qos_profile_services_default.liveliness_lease_duration,
      output_profile.liveliness_lease_duration));
}

TEST(test_qos, test_parse_type_hash_from_user_data)
{
  const auto zero_val = rosidl_get_zero_initialized_type_hash();

  std::string bad_value = "something that isn't key equals value semicolon";
  rosidl_type_hash_t result_type_hash;
  rmw_ret_t ret = rmw_dds_common::parse_type_hash_from_user_data(
    reinterpret_cast<uint8_t *>(bad_value.data()), bad_value.size(), result_type_hash);
  EXPECT_EQ(ret, RMW_RET_OK);
  EXPECT_EQ(0, memcmp(&result_type_hash, &zero_val, sizeof(rosidl_type_hash_t)))
    << "Non-ros 2 user_data should result in zero hash struct.";

  std::string no_key = "key1=value1;key2=value2;key3=value3;";
  ret = rmw_dds_common::parse_type_hash_from_user_data(
    reinterpret_cast<uint8_t *>(no_key.data()), no_key.size(), result_type_hash);
  EXPECT_EQ(ret, RMW_RET_OK);
  EXPECT_EQ(0, memcmp(&result_type_hash, &zero_val, sizeof(rosidl_type_hash_t)))
    << "No typehash key should result in zero hash struct.";

  rosidl_type_hash_t input_type_hash;
  input_type_hash.version = 1;
  for (uint8_t i = 0; i < ROSIDL_TYPE_HASH_SIZE; i++) {
    input_type_hash.value[i] = i;
  }
  char * type_hash_c_str;
  auto allocator = rcutils_get_default_allocator();
  ret = rosidl_stringify_type_hash(&input_type_hash, allocator, &type_hash_c_str);
  ASSERT_EQ(ret, RCUTILS_RET_OK);
  RCPPUTILS_SCOPE_EXIT(allocator.deallocate(type_hash_c_str, &allocator.state));
  std::string type_hash_string(type_hash_c_str);
  std::string good_data = "foo=bar;typehash=" + type_hash_string + ";key=value;";
  ret = rmw_dds_common::parse_type_hash_from_user_data(
    reinterpret_cast<uint8_t *>(good_data.data()), good_data.size(), result_type_hash);
  EXPECT_EQ(ret, RMW_RET_OK);
  EXPECT_EQ(0, memcmp(&result_type_hash, &input_type_hash, sizeof(rosidl_type_hash_t)));
}

TEST(test_qos, test_encode_type_hash_for_user_data_qos)
{
  rosidl_type_hash_t test_hash = rosidl_get_zero_initialized_type_hash();
  std::string hash_string;
  rmw_ret_t ret = rmw_dds_common::encode_type_hash_for_user_data_qos(test_hash, hash_string);
  EXPECT_EQ(ret, RMW_RET_OK);
  EXPECT_EQ(hash_string, "");

  test_hash.version = 1;
  for (uint8_t i = 0; i < ROSIDL_TYPE_HASH_SIZE; i++) {
    test_hash.value[i] = i;
  }
  hash_string.clear();
  ret = rmw_dds_common::encode_type_hash_for_user_data_qos(test_hash, hash_string);
  EXPECT_EQ(ret, RMW_RET_OK);
  EXPECT_EQ(
    hash_string,
    "typehash=RIHS01_000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f;");
}
