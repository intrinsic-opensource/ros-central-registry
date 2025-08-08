cmake_minimum_required(VERSION 3.5)
project(unique_identifier_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # we dont use add_compile_options with pedantic in message packages
  # because the Python C extensions dont comply with it
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
# Depend on "core" generators instead of "default" generators
# because ROS actions depend on this package
find_package(rosidl_core_generators REQUIRED)

set(msg_files
  "msg/UUID.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files})

ament_export_dependencies(rosidl_core_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(
  FILES mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME})
