// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include "ament_index_cpp/get_search_paths.hpp"

#include <sys/stat.h>

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <list>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#ifdef _WIN32
#define stat _stat
#endif

#include <stdio.h>
extern char **environ;

namespace ament_index_cpp
{

// Try to get the value of an environment variable, and optionally return the result.
std::optional<std::string> get_env_variable(const char * env_var) {
  char * ament_prefix_path = nullptr;
#ifndef _WIN32
  ament_prefix_path = getenv(env_var);
#else
  size_t ament_prefix_path_size;
  _dupenv_s(&ament_prefix_path, &ament_prefix_path_size, env_var);
#endif
  if (!ament_prefix_path || std::string(ament_prefix_path).empty()) {
      return {};
  }
  return std::string(ament_prefix_path);
}

std::list<std::string>
get_search_paths()
{
  // In a Bazel context AMENT_PREFIX_PATH should not be set. If this is the case, then
  // we need to poke for a MANIFEST file, and use the current work directory as a
  // stand-in search path for package data.
  std::optional<std::string> ament_prefix_path = get_env_variable("AMENT_PREFIX_PATH");
  if (!ament_prefix_path) {
    const std::filesystem::path p1 = std::filesystem::current_path();
    const std::filesystem::path p2 = "../MANIFEST";
    if (std::filesystem::exists(p1 / p2)) {
      return std::list<std::string>{p1};
    }
    throw std::runtime_error("Environment variable 'AMENT_PREFIX_PATH' is not set or empty");
  }

  // split at token into separate paths
  std::list<std::string> paths;
  std::stringstream ss(*ament_prefix_path);
  std::string tok;
#ifndef _WIN32
  char delim = ':';
#else
  char delim = ';';
#endif
  while (getline(ss, tok, delim)) {
    if (tok.empty()) {
      continue;
    }
    // skip non existing directories
    struct stat s;
    if (stat(tok.c_str(), &s)) {
      continue;
    }
    if ((s.st_mode & S_IFMT) == S_IFDIR) {
      paths.push_back(tok);
    }
  }

#ifdef _WIN32
  if (ament_prefix_path) {
    free(ament_prefix_path);
  }
#endif

  return paths;
}

}  // namespace ament_index_cpp
