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

#ifndef RCPPUTILS__PROCESS_HPP_
#define RCPPUTILS__PROCESS_HPP_

#include <string>

#include "rcpputils/visibility_control.hpp"

namespace rcpputils
{

/// Retrieve the current executable name.
/**
 * This function portably retrieves the current program name and returns
 * a copy of it.
 *
 * This function is thread-safe.
 *
 * \return The program name.
 * \throws std::runtime_error on error
 */
RCPPUTILS_PUBLIC
std::string get_executable_name();

}  // namespace rcpputils

#endif  // RCPPUTILS__PROCESS_HPP_
