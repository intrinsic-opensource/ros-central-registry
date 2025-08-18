cmake_minimum_required(VERSION 3.20)
project(rosidl_dynamic_typesupport)
set(CMAKE_VERBOSE_MAKEFILE ON)


# COMPILER SETTINGS ================================================================================
# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_VERBOSE_MAKEFILE ON)


# DEPS =============================================================================================
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros_core REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosidl_runtime_c REQUIRED)


# TARGETS ==========================================================================================
add_library(${PROJECT_NAME}
  "src/api/serialization_support.c"
  "src/api/dynamic_data.c"
  "src/api/dynamic_type.c"

  "src/dynamic_message_type_support_struct.c"
  "src/identifier.c"
)
if(WIN32)
  target_compile_definitions(${PROJECT_NAME}
    PRIVATE "ROSIDL_DYNAMIC_TYPESUPPORT_BUILDING_DLL")
endif()
target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rcutils::rcutils
  rosidl_runtime_c::rosidl_runtime_c
)


# INSTALL AND EXPORT ===============================================================================
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}-export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME}-export HAS_LIBRARY_TARGET)
ament_export_dependencies(rcutils)
ament_export_dependencies(rosidl_runtime_c)

ament_package()
