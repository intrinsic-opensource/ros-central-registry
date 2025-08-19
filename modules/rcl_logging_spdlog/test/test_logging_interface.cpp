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

#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>
#include <string>

#include "gmock/gmock.h"

#include "rcl_logging_interface/rcl_logging_interface.h"

#include "rcpputils/env.hpp"
#include "rcpputils/filesystem_helper.hpp"
#include "rcpputils/scope_exit.hpp"

#include "rcutils/allocator.h"
#include "rcutils/error_handling.h"
#include "rcutils/logging.h"
#include "rcutils/process.h"
#include "rcutils/strdup.h"
#include "rcutils/testing/fault_injection.h"

static constexpr int logger_levels[] =
{
  RCUTILS_LOG_SEVERITY_UNSET,
  RCUTILS_LOG_SEVERITY_DEBUG,
  RCUTILS_LOG_SEVERITY_INFO,
  RCUTILS_LOG_SEVERITY_WARN,
  RCUTILS_LOG_SEVERITY_ERROR,
  RCUTILS_LOG_SEVERITY_FATAL,
};

// This is a helper class that resets an environment
// variable when leaving scope
class RestoreEnvVar final
{
public:
  explicit RestoreEnvVar(const std::string & name)
  : name_(name),
    value_(rcpputils::get_env_var(name.c_str()))
  {
  }

  ~RestoreEnvVar()
  {
    if (!rcpputils::set_env_var(name_.c_str(), value_.c_str())) {
      std::cerr << "Failed to restore value of environment variable: " << name_ << std::endl;
    }
  }

private:
  const std::string name_;
  const std::string value_;
};

class AllocatorTest : public ::testing::Test
{
public:
  AllocatorTest()
  : allocator(rcutils_get_default_allocator()),
    bad_allocator(get_bad_allocator()),
    invalid_allocator(rcutils_get_zero_initialized_allocator())
  {
  }

  rcutils_allocator_t allocator;
  rcutils_allocator_t bad_allocator;
  rcutils_allocator_t invalid_allocator;

private:
  static rcutils_allocator_t get_bad_allocator()
  {
    rcutils_allocator_t bad_allocator = rcutils_get_default_allocator();
    bad_allocator.allocate = AllocatorTest::bad_malloc;
    bad_allocator.reallocate = AllocatorTest::bad_realloc;
    return bad_allocator;
  }

  static void * bad_malloc(size_t, void *)
  {
    return nullptr;
  }

  static void * bad_realloc(void *, size_t, void *)
  {
    return nullptr;
  }
};

class LoggingTest : public ::testing::Test
{
public:
  void SetUp()
  {
    allocator = rcutils_get_default_allocator();
    orig_ros_log_dir_value_ = rcpputils::get_env_var("ROS_LOG_DIR");
    std::filesystem::path log_dir = rcpputils::fs::create_temporary_directory(
      "rcl_logging_spdlog");

    local_log_dir_ = log_dir.string();

    rcpputils::set_env_var("ROS_LOG_DIR", local_log_dir_.c_str());
  }

  void TearDown()
  {
    rcpputils::set_env_var("ROS_LOG_DIR", orig_ros_log_dir_value_.c_str());
    if (std::filesystem::remove_all(local_log_dir_) == 0) {
      std::cerr << "Failed to remove temporary directory\n";
    }
  }

  std::filesystem::path find_single_log(const char * prefix)
  {
    char * rcl_log_dir = nullptr;
    rcl_logging_ret_t dir_ret = rcl_logging_get_logging_directory(allocator, &rcl_log_dir);
    if (dir_ret != RCL_LOGGING_RET_OK) {
      throw std::runtime_error("Failed to get logging directory");
    }
    RCPPUTILS_SCOPE_EXIT(
    {
      allocator.deallocate(rcl_log_dir, allocator.state);
    });
    std::filesystem::path log_dir(rcl_log_dir);
    std::string expected_prefix = get_expected_log_prefix(prefix);

    std::filesystem::path found;
    std::filesystem::file_time_type found_last_write;
    for (const std::filesystem::directory_entry & dir_entry :
      std::filesystem::directory_iterator{log_dir})
    {
      // If the start of the filename matches the expected_prefix, and this is the newest file
      // starting with that prefix, hold onto it to return later.
      if (dir_entry.path().filename().string().rfind(expected_prefix, 0) == 0) {
        if (found.string().empty() || dir_entry.last_write_time() > found_last_write) {
          found = dir_entry.path();
          found_last_write = dir_entry.last_write_time();
          // Even though we found the file, we have to keep looking in case there
          // is another file with the same prefix but a newer timestamp.
        }
      }
    }

    return found;
  }

  rcutils_allocator_t allocator;

private:
  std::string get_expected_log_prefix(const char * name)
  {
    char * exe_name;
    if (name == nullptr || name[0] == '\0') {
      exe_name = rcutils_get_executable_name(allocator);
    } else {
      exe_name = rcutils_strdup(name, allocator);
    }
    if (nullptr == exe_name) {
      throw std::runtime_error("Failed to determine executable name");
    }
    std::stringstream prefix;
    prefix << exe_name << "_" << rcutils_get_pid() << "_";
    allocator.deallocate(exe_name, allocator.state);
    return prefix.str();
  }

  std::string orig_ros_log_dir_value_;
  std::string local_log_dir_;
};

TEST_F(AllocatorTest, init_invalid)
{
  // Config files are not supported by spdlog
  EXPECT_EQ(
    RCL_LOGGING_RET_ERROR,
    rcl_logging_external_initialize(nullptr, "anything", allocator));
  rcutils_reset_error();
  EXPECT_EQ(
    RCL_LOGGING_RET_ERROR,
    rcl_logging_external_initialize("anything", nullptr, bad_allocator));
  rcutils_reset_error();
  EXPECT_EQ(
    RCL_LOGGING_RET_INVALID_ARGUMENT,
    rcl_logging_external_initialize(nullptr, nullptr, invalid_allocator));
  rcutils_reset_error();
}

TEST_F(AllocatorTest, init_failure)
{
  RestoreEnvVar home_var("HOME");
  RestoreEnvVar userprofile_var("USERPROFILE");

  // No home directory to write log to
  ASSERT_TRUE(rcpputils::set_env_var("HOME", nullptr));
  ASSERT_TRUE(rcpputils::set_env_var("USERPROFILE", nullptr));
  EXPECT_EQ(RCL_LOGGING_RET_ERROR, rcl_logging_external_initialize(nullptr, nullptr, allocator));
  rcutils_reset_error();

  // Force failure to create directories
  std::filesystem::path fake_home = std::filesystem::current_path() / "fake_home_dir";
  ASSERT_TRUE(std::filesystem::create_directories(fake_home));
  ASSERT_TRUE(rcpputils::set_env_var("HOME", fake_home.string().c_str()));

  // ...fail to create .ros dir
  std::filesystem::path ros_dir = fake_home / ".ros";
  std::fstream(ros_dir.string(), std::ios_base::out).close();
  EXPECT_EQ(RCL_LOGGING_RET_ERROR, rcl_logging_external_initialize(nullptr, nullptr, allocator));
  rcutils_reset_error();
  ASSERT_TRUE(std::filesystem::remove(ros_dir));

  // ...fail to create .ros/log dir
  ASSERT_TRUE(std::filesystem::create_directories(ros_dir));
  std::filesystem::path ros_log_dir = ros_dir / "log";
  std::fstream(ros_log_dir.string(), std::ios_base::out).close();
  EXPECT_EQ(RCL_LOGGING_RET_ERROR, rcl_logging_external_initialize(nullptr, nullptr, allocator));
  rcutils_reset_error();
  ASSERT_TRUE(std::filesystem::remove(ros_log_dir));
  ASSERT_TRUE(std::filesystem::remove(ros_dir));

  ASSERT_TRUE(std::filesystem::remove(fake_home));
}

TEST_F(LoggingTest, log_file_name_prefix)
{
  std::string log_file_path;
  // executable name in default
  {
    ASSERT_EQ(RCL_LOGGING_RET_OK, rcl_logging_external_initialize(nullptr, nullptr, allocator));
    EXPECT_EQ(RCL_LOGGING_RET_OK, rcl_logging_external_shutdown());
    EXPECT_NO_THROW(log_file_path = find_single_log(nullptr).string());
  }
  // falls back to executable name if not nullptr, but empty
  {
    ASSERT_EQ(RCL_LOGGING_RET_OK, rcl_logging_external_initialize("", nullptr, allocator));
    EXPECT_EQ(RCL_LOGGING_RET_OK, rcl_logging_external_shutdown());
    EXPECT_NO_THROW(log_file_path = find_single_log(nullptr).string());
  }
  // specified by user application
  {
    ASSERT_EQ(RCL_LOGGING_RET_OK, rcl_logging_external_initialize("logger", nullptr, allocator));
    EXPECT_EQ(RCL_LOGGING_RET_OK, rcl_logging_external_shutdown());
    EXPECT_NO_THROW(log_file_path = find_single_log("logger").string());
  }
}

TEST_F(LoggingTest, init_old_flushing_behavior)
{
  RestoreEnvVar env_var("RCL_LOGGING_SPDLOG_EXPERIMENTAL_OLD_FLUSHING_BEHAVIOR");
  rcpputils::set_env_var("RCL_LOGGING_SPDLOG_EXPERIMENTAL_OLD_FLUSHING_BEHAVIOR", "1");

  ASSERT_EQ(RCL_LOGGING_RET_OK, rcl_logging_external_initialize(nullptr, nullptr, allocator));

  std::stringstream expected_log;
  for (int level : logger_levels) {
    EXPECT_EQ(RCL_LOGGING_RET_OK, rcl_logging_external_set_logger_level(nullptr, level));

    for (int severity : logger_levels) {
      std::stringstream ss;
      ss << "Message of severity " << severity << " at level " << level;
      rcl_logging_external_log(severity, nullptr, ss.str().c_str());

      if (severity >= level) {
        expected_log << ss.str() << std::endl;
      } else if (severity == 0 && level == 10) {
        // This is a special case - not sure what the right behavior is
        expected_log << ss.str() << std::endl;
      }
    }
  }

  EXPECT_EQ(RCL_LOGGING_RET_OK, rcl_logging_external_shutdown());

  std::string log_file_path = find_single_log(nullptr).string();
  std::ifstream log_file(log_file_path);
  std::stringstream actual_log;
  actual_log << log_file.rdbuf();
  EXPECT_EQ(
    expected_log.str(),
    actual_log.str()) << "Unexpected log contents in " << log_file_path;
}

TEST_F(LoggingTest, init_explicit_new_flush_behavior)
{
  RestoreEnvVar env_var("RCL_LOGGING_SPDLOG_EXPERIMENTAL_OLD_FLUSHING_BEHAVIOR");
  rcpputils::set_env_var("RCL_LOGGING_SPDLOG_EXPERIMENTAL_OLD_FLUSHING_BEHAVIOR", "0");

  ASSERT_EQ(RCL_LOGGING_RET_OK, rcl_logging_external_initialize(nullptr, nullptr, allocator));

  std::stringstream expected_log;
  for (int level : logger_levels) {
    EXPECT_EQ(RCL_LOGGING_RET_OK, rcl_logging_external_set_logger_level(nullptr, level));

    for (int severity : logger_levels) {
      std::stringstream ss;
      ss << "Message of severity " << severity << " at level " << level;
      rcl_logging_external_log(severity, nullptr, ss.str().c_str());

      if (severity >= level) {
        expected_log << ss.str() << std::endl;
      } else if (severity == 0 && level == 10) {
        // This is a special case - not sure what the right behavior is
        expected_log << ss.str() << std::endl;
      }
    }
  }

  EXPECT_EQ(RCL_LOGGING_RET_OK, rcl_logging_external_shutdown());

  std::string log_file_path = find_single_log(nullptr).string();
  std::ifstream log_file(log_file_path);
  std::stringstream actual_log;
  actual_log << log_file.rdbuf();
  EXPECT_EQ(
    expected_log.str(),
    actual_log.str()) << "Unexpected log contents in " << log_file_path;
}

TEST_F(LoggingTest, init_invalid_flush_setting)
{
  RestoreEnvVar env_var("RCL_LOGGING_SPDLOG_EXPERIMENTAL_OLD_FLUSHING_BEHAVIOR");
  rcpputils::set_env_var("RCL_LOGGING_SPDLOG_EXPERIMENTAL_OLD_FLUSHING_BEHAVIOR", "invalid");

  ASSERT_EQ(RCL_LOGGING_RET_ERROR, rcl_logging_external_initialize(nullptr, nullptr, allocator));
  std::string error_state_str = rcutils_get_error_string().str;
  using ::testing::HasSubstr;
  ASSERT_THAT(
    error_state_str,
    HasSubstr("unrecognized value:"));
  rcutils_reset_error();
}

TEST_F(LoggingTest, full_cycle)
{
  ASSERT_EQ(RCL_LOGGING_RET_OK, rcl_logging_external_initialize(nullptr, nullptr, allocator));

  // Make sure we can call initialize more than once
  ASSERT_EQ(RCL_LOGGING_RET_OK, rcl_logging_external_initialize(nullptr, nullptr, allocator));

  std::stringstream expected_log;
  for (int level : logger_levels) {
    EXPECT_EQ(RCL_LOGGING_RET_OK, rcl_logging_external_set_logger_level(nullptr, level));

    for (int severity : logger_levels) {
      std::stringstream ss;
      ss << "Message of severity " << severity << " at level " << level;
      rcl_logging_external_log(severity, nullptr, ss.str().c_str());

      if (severity >= level) {
        expected_log << ss.str() << std::endl;
      } else if (severity == 0 && level == 10) {
        // This is a special case - not sure what the right behavior is
        expected_log << ss.str() << std::endl;
      }
    }
  }

  EXPECT_EQ(RCL_LOGGING_RET_OK, rcl_logging_external_shutdown());

  std::string log_file_path = find_single_log(nullptr).string();
  std::ifstream log_file(log_file_path);
  std::stringstream actual_log;
  actual_log << log_file.rdbuf();
  EXPECT_EQ(
    expected_log.str(),
    actual_log.str()) << "Unexpected log contents in " << log_file_path;
}

TEST_F(LoggingTest, init_fini_maybe_fail_test)
{
  RCUTILS_FAULT_INJECTION_TEST(
  {
    if (RCL_LOGGING_RET_OK == rcl_logging_external_initialize(nullptr, nullptr, allocator)) {
      EXPECT_EQ(RCL_LOGGING_RET_OK, rcl_logging_external_shutdown());
    } else {
      EXPECT_TRUE(rcutils_error_is_set());
      rcutils_reset_error();
    }
  });
}
