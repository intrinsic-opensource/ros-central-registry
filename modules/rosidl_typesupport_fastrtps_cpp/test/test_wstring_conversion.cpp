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
#include "osrf_testing_tools_cpp/scope_exit.hpp"

#include "rosidl_typesupport_fastrtps_cpp/serialization_helpers.hpp"

using rosidl_typesupport_fastrtps_cpp::cdr_serialize;
using rosidl_typesupport_fastrtps_cpp::cdr_deserialize;

void test_ser_des(const std::u16string & input)
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

  std::u16string output;
  ASSERT_TRUE(cdr_deserialize(deserializer, output));
  EXPECT_EQ(input, output);
}

TEST(test_wstring_conversion, serialize_deserialize)
{
  // Default string
  test_ser_des(std::u16string());

  // Empty string
  test_ser_des(std::u16string(u""));

  // Non-empty string
  test_ser_des(std::u16string(u"¡Hola, Mundo!"));
}
