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
#include "rosidl_runtime_c/u16string_functions.h"
#include "osrf_testing_tools_cpp/scope_exit.hpp"

#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"

using rosidl_typesupport_fastrtps_c::cdr_serialize;
using rosidl_typesupport_fastrtps_c::cdr_deserialize;

void test_ser_des(const rosidl_runtime_c__U16String & input)
{
  namespace fastcdr = eprosima::fastcdr;
  char raw_buffer[1024];

  {
    fastcdr::FastBuffer buffer(raw_buffer, sizeof(raw_buffer));
    fastcdr::Cdr serializer(buffer, fastcdr::Cdr::DEFAULT_ENDIAN, fastcdr::CdrVersion::XCDRv1);
    serializer.set_encoding_flag(fastcdr::EncodingAlgorithmFlag::PLAIN_CDR);

    cdr_serialize(serializer, input);
  }

  fastcdr::FastBuffer buffer(raw_buffer, sizeof(raw_buffer));
  fastcdr::Cdr deserializer(buffer, fastcdr::Cdr::DEFAULT_ENDIAN, fastcdr::CdrVersion::XCDRv1);
  deserializer.set_encoding_flag(fastcdr::EncodingAlgorithmFlag::PLAIN_CDR);

  rosidl_runtime_c__U16String output;
  ASSERT_TRUE(rosidl_runtime_c__U16String__init(&output));
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
  {
    rosidl_runtime_c__U16String__fini(&output);
  });

  ASSERT_TRUE(cdr_deserialize(deserializer, output));
  EXPECT_EQ(input.size, output.size);
  EXPECT_EQ(0, memcmp(input.data, output.data, input.size));
}

TEST(test_wstring_conversion, serialize_deserialize)
{
  std::wstring actual;
  rosidl_runtime_c__U16String input;
  ASSERT_TRUE(rosidl_runtime_c__U16String__init(&input));
  OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT(
  {
    rosidl_runtime_c__U16String__fini(&input);
  });

  // Default string
  test_ser_des(input);

  // Empty string
  ASSERT_TRUE(rosidl_runtime_c__U16String__assign(&input, (const uint16_t *)u""));
  test_ser_des(input);

  // Non-empty string
  ASSERT_TRUE(rosidl_runtime_c__U16String__assign(&input, (const uint16_t *)u"¡Hola, Mundo!"));
  test_ser_des(input);
}
