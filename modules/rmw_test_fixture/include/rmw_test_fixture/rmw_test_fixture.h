// Copyright 2025 Open Source Robotics Foundation, Inc.
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

#ifndef RMW_TEST_FIXTURE__RMW_TEST_FIXTURE_H_
#define RMW_TEST_FIXTURE__RMW_TEST_FIXTURE_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <rmw/macros.h>
#include <rmw/types.h>

#include "rmw_test_fixture/visibility_control.h"

/// Begin isolating ROS communication in this process.
/**
 * Perform neccessary changes to the process and/or environment so that any
 * attempted ROS communication will be isolated from other processes.
 *
 * \return `RMW_RET_OK` if successful, or
 * \return `RMW_RET_ERROR` if an unexpected error occurs.
 */
RMW_TEST_FIXTURE_PUBLIC
RMW_WARN_UNUSED
rmw_ret_t
rmw_test_isolation_start();

/// Cease isolation of ROS communication in this process.
/**
 * Restore the process and/or environment to the previous state and deallocate
 * any resources previously allocated by rmw_test_isolation_start().
 *
 * \return `RMW_RET_OK` if successful, or
 * \return `RMW_RET_ERROR` if an unexpected error occurs.
 */
RMW_TEST_FIXTURE_PUBLIC
rmw_ret_t
rmw_test_isolation_stop();

#ifdef __cplusplus
}
#endif

#endif  // RMW_TEST_FIXTURE__RMW_TEST_FIXTURE_H_
