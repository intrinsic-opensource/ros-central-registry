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

#ifndef DETAIL__UTILS_HPP_
#define DETAIL__UTILS_HPP_

#include <cstdint>
#include <fastdds/dds/core/ReturnCode.hpp>
#include <rosidl_dynamic_typesupport_fastrtps/visibility_control.h>
#include <rcutils/types/rcutils_ret.h>
#include <string>


/// Convert size_t to uint32_t, with limit checking
/// `in` must be smaller than std::numeric_limits<uint32_t>::max()
ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
uint32_t
fastdds__size_t_to_uint32_t(size_t in);

/// u16string copy
/// `dest` and `src` cannot be NULL, this method will return NULL in those cases
ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
char16_t *
fastdds__ucsncpy(char16_t * dest, const char16_t * src, size_t n);

/// Convert u16string to wstring
ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
std::wstring
fastdds__u16string_to_wstring(const std::u16string & u16str);

/// Convert wstring to u16string
ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
std::u16string
fastdds__wstring_to_u16string(const std::wstring & wstr);

/// Substring replace
ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
std::string
fastdds__replace_string(std::string str, const std::string & from, const std::string & to);

/// Convert fastdds return types to rcl
ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS_PUBLIC
rcutils_ret_t
fastdds__convert_fastdds_ret_to_rcl_ret(eprosima::fastdds::dds::ReturnCode_t fastdds_ret);


#endif  // DETAIL__UTILS_HPP_
