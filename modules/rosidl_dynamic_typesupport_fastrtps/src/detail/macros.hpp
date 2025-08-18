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

#ifndef DETAIL__MACROS_HPP_
#define DETAIL__MACROS_HPP_

#include <fastdds/dds/core/ReturnCode.hpp>
#include <rcutils/error_handling.h>
#include <rcutils/types/rcutils_ret.h>

#include "utils.hpp"

/// Check ReturnCode_t, and return its equivalent rcutils_ret_t if not ok, with an error message.
/**
 * If the statement returns a ReturnCode_t other than RETCODE_OK, set the error message and return
 * the return type, converted to the equivalent rcutils_ret_t.
 *
 * \param[in] statement_with_return_code The statement to test.
 * \param[in] msg The error message if the statement returns a non-ok return code.
 */
#define FASTDDS_CHECK_RET_FOR_NOT_OK_WITH_MSG(statement_with_return_code, msg) \
  do \
  { \
    eprosima::fastdds::dds::ReturnCode_t macro_ret_ = statement_with_return_code; \
    if (macro_ret_ != eprosima::fastdds::dds::RETCODE_OK) \
    { \
      RCUTILS_SET_ERROR_MSG(msg); \
      return fastdds__convert_fastdds_ret_to_rcl_ret(macro_ret_); \
    } \
  } while (0)

/// Return ReturnCode_t, converted to its equivalent rcutils_ret_t, setting error msg as needed.
/**
 *
 * \param[in] statement_with_return_code The statement to test.
 * \param[in] msg The error message if the statement returns a non-ok return code.
 */
#define FASTDDS_CHECK_RET_FOR_NOT_OK_AND_RETURN_WITH_MSG(statement_with_return_code, msg) \
  do \
  { \
    eprosima::fastdds::dds::ReturnCode_t macro_ret_ = statement_with_return_code; \
    if (macro_ret_ != eprosima::fastdds::dds::RETCODE_OK) \
    { \
      RCUTILS_SET_ERROR_MSG(msg); \
      return fastdds__convert_fastdds_ret_to_rcl_ret(macro_ret_); \
    } \
    else \
    { \
      return RCUTILS_RET_OK; \
    } \
  } while (0)

#define FASTDDS_STRINGIFY(x) FASTDDS_STRINGIFY2(x)
#define FASTDDS_STRINGIFY2(x) #x

#define FASTDDS_EXPAND(x) FASTDDS_EXPAND2(x)
#define FASTDDS_EXPAND2(x) x

#endif // DETAIL__MACROS_HPP_
