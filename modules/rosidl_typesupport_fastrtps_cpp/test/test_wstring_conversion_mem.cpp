// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include <string>

#include "gtest/gtest.h"

#include "fastcdr/Cdr.h"
#include "osrf_testing_tools_cpp/memory_tools/memory_tools.hpp"
#include "osrf_testing_tools_cpp/scope_exit.hpp"

#include "rosidl_typesupport_fastrtps_cpp/serialization_helpers.hpp"

using rosidl_typesupport_fastrtps_cpp::cdr_deserialize;

TEST(test_wstring_conversion, u16string_resize_failure)
{
  namespace fastcdr = eprosima::fastcdr;

  osrf_testing_tools_cpp::memory_tools::initialize();
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
  {
    osrf_testing_tools_cpp::memory_tools::uninitialize();
  });

  std::wstring wstring(L"¡Hola, Mundo!");
  char raw_buffer[1024];
  {
    fastcdr::FastBuffer buffer(raw_buffer, sizeof(raw_buffer));
    fastcdr::Cdr cdr(buffer, fastcdr::Cdr::DEFAULT_ENDIAN, fastcdr::CdrVersion::XCDRv1);
    cdr.set_encoding_flag(fastcdr::EncodingAlgorithmFlag::PLAIN_CDR);
    cdr << wstring;
  }

  std::u16string u16string(1, 1);

  // Force malloc/realloc to fail
  auto on_unexpected_operation =
    []() {
      throw std::exception();
    };
  osrf_testing_tools_cpp::memory_tools::on_unexpected_malloc(on_unexpected_operation);
  osrf_testing_tools_cpp::memory_tools::on_unexpected_realloc(on_unexpected_operation);
  osrf_testing_tools_cpp::memory_tools::enable_monitoring();

  fastcdr::FastBuffer buffer(raw_buffer, sizeof(raw_buffer));
  fastcdr::Cdr cdr(buffer, fastcdr::Cdr::DEFAULT_ENDIAN, fastcdr::CdrVersion::XCDRv1);
  cdr.set_encoding_flag(fastcdr::EncodingAlgorithmFlag::PLAIN_CDR);
  EXPECT_NO_MEMORY_OPERATIONS(
  {
    EXPECT_FALSE(cdr_deserialize(cdr, u16string));
  });
}
