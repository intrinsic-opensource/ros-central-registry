// Copyright 2024 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#ifndef ROSIDL_TYPESUPPORT_FASTRTPS_CPP__SERIALIZATION_HELPERS_HPP_
#define ROSIDL_TYPESUPPORT_FASTRTPS_CPP__SERIALIZATION_HELPERS_HPP_

#include <rosidl_typesupport_fastrtps_cpp/visibility_control.h>

#include <fastcdr/Cdr.h>
#include <fastcdr/exceptions/BadParamException.h>

#include <limits>
#include <stdexcept>
#include <string>

namespace rosidl_typesupport_fastrtps_cpp
{

inline void cdr_serialize(
  eprosima::fastcdr::Cdr & cdr,
  const std::u16string & u16str)
{
  if (u16str.size() > std::numeric_limits<uint32_t>::max()) {
    throw std::overflow_error("String length exceeds does not fit in CDR serialization format");
  }
  auto len = static_cast<uint32_t>(u16str.size());
  cdr << len;
  for (uint32_t i = 0; i < len; ++i) {
    // We are serializing a uint32_t for interoperability with other DDS-based implementations.
    // We might change this to a uint16_t in the future if we don't mind breaking backward
    // compatibility and we make the change for all the supported DDS implementations at the same
    // time.
    uint32_t c = static_cast<uint32_t>(u16str[i]);
    cdr << c;
  }
}

inline bool cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  std::u16string & u16str)
{
  uint32_t len;
  cdr >> len;
  try {
    u16str.resize(len);
  } catch (...) {
    return false;
  }
  for (uint32_t i = 0; i < len; ++i) {
    // We are serializing a uint32_t for interoperability with other DDS-based implementations.
    // If we change this to a uint16_t in the future, we could remove the check below, since all
    // serialized values would fit in the destination type.
    uint32_t c;
    cdr >> c;
    if (c > std::numeric_limits<std::u16string::value_type>::max()) {
      throw eprosima::fastcdr::exception::BadParamException(
              "Character value exceeds maximum value");
    }
    u16str[i] = static_cast<std::u16string::value_type>(c);
  }

  return true;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#endif  // ROSIDL_TYPESUPPORT_FASTRTPS_CPP__SERIALIZATION_HELPERS_HPP_
