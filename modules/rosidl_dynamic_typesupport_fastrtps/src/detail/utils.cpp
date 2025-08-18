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

#include "utils.hpp"

#include <fastdds/dds/core/ReturnCode.hpp>

#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>


uint32_t
fastdds__size_t_to_uint32_t(size_t in)
{
  if (in > std::numeric_limits<uint32_t>::max()) {
    std::cerr << "Passed size_t will overflow when narrowed to uint32_t!" << std::endl;
    std::abort();
  }
  return static_cast<uint32_t>(in);
}

char16_t *
fastdds__ucsncpy(char16_t * dest, const char16_t * src, size_t n)
{
  if (src == NULL || dest == NULL) {
    return NULL;
  }
  char16_t * out = dest;
  while (*src && n--) {
    *dest = *src;
    dest++;
    src++;
  }
  *dest = '\0';
  return out;
}

std::wstring
fastdds__u16string_to_wstring(const std::u16string & u16str)
{
  std::wstring wstr;
  wstr.resize(u16str.size());
  for (size_t i = 0; i < u16str.size(); ++i) {
    wstr[i] = static_cast<wchar_t>(u16str[i]);
  }
  return wstr;
}

std::u16string
fastdds__wstring_to_u16string(const std::wstring & wstr)
{
  std::u16string u16str;
  u16str.resize(wstr.size());
  for (size_t i = 0; i < wstr.size(); ++i) {
    u16str[i] = static_cast<wchar_t>(wstr[i]);
  }
  return u16str;
}

std::string
fastdds__replace_string(std::string str, const std::string & from, const std::string & to)
{
  size_t pos = 0;
  while ((pos = str.find(from, pos)) != std::string::npos) {
    str.replace(pos, from.length(), to);
    pos += to.length();
  }
  return str;
}

rcutils_ret_t
fastdds__convert_fastdds_ret_to_rcl_ret(eprosima::fastdds::dds::ReturnCode_t fastdds_ret)
{
  switch (fastdds_ret) {
    case eprosima::fastdds::dds::RETCODE_OK:
      return RCUTILS_RET_OK;
    case eprosima::fastdds::dds::RETCODE_BAD_PARAMETER:
      return RCUTILS_RET_INVALID_ARGUMENT;
    case eprosima::fastdds::dds::RETCODE_OUT_OF_RESOURCES:
      return RCUTILS_RET_NOT_ENOUGH_SPACE;
    case eprosima::fastdds::dds::RETCODE_NO_DATA:
      return RCUTILS_RET_NOT_FOUND;

    case eprosima::fastdds::dds::RETCODE_ERROR:
    case eprosima::fastdds::dds::RETCODE_UNSUPPORTED:
    case eprosima::fastdds::dds::RETCODE_PRECONDITION_NOT_MET:
    case eprosima::fastdds::dds::RETCODE_NOT_ENABLED:
    case eprosima::fastdds::dds::RETCODE_ILLEGAL_OPERATION:
    case eprosima::fastdds::dds::RETCODE_NOT_ALLOWED_BY_SECURITY:
    case eprosima::fastdds::dds::RETCODE_IMMUTABLE_POLICY:
    case eprosima::fastdds::dds::RETCODE_INCONSISTENT_POLICY:
    case eprosima::fastdds::dds::RETCODE_ALREADY_DELETED:
    case eprosima::fastdds::dds::RETCODE_TIMEOUT:
    default:
      return RCUTILS_RET_ERROR;
  }
}
