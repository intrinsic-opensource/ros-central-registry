# Copyright 2014-2018 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

find_package(rmw REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(rosidl_typesupport_interface REQUIRED)

# By default, without the settings below, find_package(Python3) will attempt
# to find the newest python version it can, and additionally will find the
# most specific version.  For instance, on a system that has
# /usr/bin/python3.10, /usr/bin/python3.11, and /usr/bin/python3, it will find
# /usr/bin/python3.11, even if /usr/bin/python3 points to /usr/bin/python3.10.
# The behavior we want is to prefer the "system" installed version unless the
# user specifically tells us othewise through the Python3_EXECUTABLE hint.
# Setting CMP0094 to NEW means that the search will stop after the first
# python version is found.  Setting Python3_FIND_UNVERSIONED_NAMES means that
# the search will prefer /usr/bin/python3 over /usr/bin/python3.11.  And that
# latter functionality is only available in CMake 3.20 or later, so we need
# at least that version.
cmake_minimum_required(VERSION 3.20)
cmake_policy(SET CMP0094 NEW)
set(Python3_FIND_UNVERSIONED_NAMES FIRST)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)

# Get a list of typesupport implementations from valid rmw implementations.
rosidl_generator_py_get_typesupports(_typesupport_impls)

if(_typesupport_impls STREQUAL "")
  message(WARNING "No valid typesupport for Python generator. Python messages will not be generated.")
  return()
endif()

set(_output_path
  "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py/${PROJECT_NAME}")
set(_generated_extension_files "")
set(_generated_py_files "")
set(_generated_c_files "")

foreach(_typesupport_impl ${_typesupport_impls})
  set(_generated_extension_${_typesupport_impl}_files "")
endforeach()

foreach(_abs_idl_file ${rosidl_generate_interfaces_ABS_IDL_FILES})
  get_filename_component(_parent_folder "${_abs_idl_file}" DIRECTORY)
  get_filename_component(_parent_folder "${_parent_folder}" NAME)
  get_filename_component(_idl_name "${_abs_idl_file}" NAME_WE)
  string_camel_case_to_lower_case_underscore("${_idl_name}" _module_name)
  list(APPEND _generated_py_files
    "${_output_path}/${_parent_folder}/_${_module_name}.py")
  list(APPEND _generated_c_files
    "${_output_path}/${_parent_folder}/_${_module_name}_s.c")
endforeach()

file(MAKE_DIRECTORY "${_output_path}")
file(WRITE "${_output_path}/__init__.py" "")

# collect relative paths of directories containing to-be-installed Python modules
# add __init__.py files where necessary
set(_generated_py_dirs "")
foreach(_generated_py_file ${_generated_py_files})
  get_filename_component(_parent_folder "${_generated_py_file}" DIRECTORY)
  set(_init_module "${_parent_folder}/__init__.py")
  list(FIND _generated_py_files "${_init_module}" _index)
  if(_index EQUAL -1)
    list(APPEND _generated_py_files "${_init_module}")

    string(LENGTH "${_output_path}" _length)
    math(EXPR _index "${_length} + 1")
    string(SUBSTRING "${_parent_folder}" ${_index} -1 _relative_directory)
    list(APPEND _generated_py_dirs "${_relative_directory}")
  endif()
endforeach()

if(NOT _generated_c_files STREQUAL "")
  foreach(_typesupport_impl ${_typesupport_impls})
    list(APPEND _generated_extension_${_typesupport_impl}_files "${_output_path}/_${PROJECT_NAME}_s.ep.${_typesupport_impl}.c")
    list(APPEND _generated_extension_files "${_generated_extension_${_typesupport_impl}_files}")
  endforeach()
endif()
set(_dependency_files "")
set(_dependencies "")
foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  foreach(_idl_file ${${_pkg_name}_IDL_FILES})
    set(_abs_idl_file "${${_pkg_name}_DIR}/../${_idl_file}")
    normalize_path(_abs_idl_file "${_abs_idl_file}")
    list(APPEND _dependency_files "${_abs_idl_file}")
    list(APPEND _dependencies "${_pkg_name}:${_abs_idl_file}")
  endforeach()
endforeach()

set(target_dependencies
  "${rosidl_generator_py_BIN}"
  ${rosidl_generator_py_GENERATOR_FILES}
  "${rosidl_generator_py_TEMPLATE_DIR}/_action_pkg_typesupport_entry_point.c.em"
  "${rosidl_generator_py_TEMPLATE_DIR}/_action.py.em"
  "${rosidl_generator_py_TEMPLATE_DIR}/_idl_pkg_typesupport_entry_point.c.em"
  "${rosidl_generator_py_TEMPLATE_DIR}/_idl_support.c.em"
  "${rosidl_generator_py_TEMPLATE_DIR}/_idl.py.em"
  "${rosidl_generator_py_TEMPLATE_DIR}/_msg_pkg_typesupport_entry_point.c.em"
  "${rosidl_generator_py_TEMPLATE_DIR}/_msg_support.c.em"
  "${rosidl_generator_py_TEMPLATE_DIR}/_msg.py.em"
  "${rosidl_generator_py_TEMPLATE_DIR}/_srv_pkg_typesupport_entry_point.c.em"
  "${rosidl_generator_py_TEMPLATE_DIR}/_srv.py.em"
  ${rosidl_generate_interfaces_ABS_IDL_FILES}
  ${_dependency_files})
foreach(dep ${target_dependencies})
  if(NOT EXISTS "${dep}")
    message(FATAL_ERROR "Target dependency '${dep}' does not exist")
  endif()
endforeach()

set(generator_arguments_file "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py__arguments.json")
rosidl_write_generator_arguments(
  "${generator_arguments_file}"
  PACKAGE_NAME "${PROJECT_NAME}"
  IDL_TUPLES "${rosidl_generate_interfaces_IDL_TUPLES}"
  ROS_INTERFACE_DEPENDENCIES "${_dependencies}"
  OUTPUT_DIR "${_output_path}"
  TEMPLATE_DIR "${rosidl_generator_py_TEMPLATE_DIR}"
  TARGET_DEPENDENCIES ${target_dependencies}
)

if(NOT rosidl_generate_interfaces_SKIP_INSTALL)
  ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR "${_output_path}")
endif()

set(_target_suffix "__py")

# move custom command into a subdirectory to avoid multiple invocations on Windows
set(_subdir "${CMAKE_CURRENT_BINARY_DIR}/${rosidl_generate_interfaces_TARGET}${_target_suffix}")
file(MAKE_DIRECTORY "${_subdir}")
file(READ "${rosidl_generator_py_DIR}/custom_command.cmake" _custom_command)
file(WRITE "${_subdir}/CMakeLists.txt" "${_custom_command}")
add_subdirectory("${_subdir}" ${rosidl_generate_interfaces_TARGET}${_target_suffix})
set_property(
  SOURCE
  ${_generated_extension_files} ${_generated_py_files} ${_generated_c_files}
  PROPERTY GENERATED 1)

set(_target_name_lib "${rosidl_generate_interfaces_TARGET}__rosidl_generator_py")
add_library(${_target_name_lib} SHARED ${_generated_c_files})
target_link_libraries(${_target_name_lib} PRIVATE
  ${rosidl_generate_interfaces_TARGET}__rosidl_generator_c)
add_dependencies(
  ${_target_name_lib}
  ${rosidl_generate_interfaces_TARGET}${_target_suffix}
  ${rosidl_generate_interfaces_TARGET}__rosidl_typesupport_c
)

target_link_libraries(
  ${_target_name_lib} PRIVATE
  Python3::NumPy
  Python3::Python
)
target_include_directories(${_target_name_lib}
  PRIVATE
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_c
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py
)

set(_extension_compile_flags "")
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(_extension_compile_flags -Wall -Wextra)
endif()

rosidl_get_typesupport_target(c_typesupport_target "${rosidl_generate_interfaces_TARGET}" "rosidl_typesupport_c")
target_link_libraries(${_target_name_lib} PRIVATE ${c_typesupport_target})

foreach(_typesupport_impl ${_typesupport_impls})
  find_package(${_typesupport_impl} REQUIRED)
  # a typesupport package might not be able to generated a target anymore
  # (e.g. if an underlying vendor package isn't available in an overlay)
  if(NOT TARGET ${rosidl_generate_interfaces_TARGET}__${_typesupport_impl})
    continue()
  endif()

  set(_target_name "${PROJECT_NAME}_s__${_typesupport_impl}")

  python3_add_library(${_target_name} MODULE
    ${_generated_extension_${_typesupport_impl}_files}
  )
  add_dependencies(
    ${_target_name}
    ${rosidl_generate_interfaces_TARGET}${_target_suffix}
    ${rosidl_generate_interfaces_TARGET}__rosidl_typesupport_c
  )

  if(WIN32 AND CMAKE_BUILD_TYPE STREQUAL "Debug")
    set_target_properties(${_target_name} PROPERTIES DEBUG_POSTFIX "_d")
  endif()
  # target_compile_options(${_target_name} PRIVATE ${_extension_compile_flags})
  # TODO(sloretz) use target_compile_options when python extension passes -Wpedantic
  set_target_properties(${_target_name} PROPERTIES COMPILE_OPTIONS "${_extension_compile_flags}")

  set_target_properties(${_target_name} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${_output_path}
    RUNTIME_OUTPUT_DIRECTORY ${_output_path})

  target_link_libraries(
    ${_target_name} PRIVATE
    ${_target_name_lib}
    ${rosidl_generate_interfaces_TARGET}__${_typesupport_impl}
    ${c_typesupport_target}
    rosidl_runtime_c::rosidl_runtime_c
    rosidl_typesupport_c::rosidl_typesupport_c
    rosidl_typesupport_interface::rosidl_typesupport_interface
  )

  target_include_directories(${_target_name}
    PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_c
    ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py
  )

  foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
    target_link_libraries(${_target_name} PRIVATE ${${_pkg_name}__TARGETS})
  endforeach()

  add_dependencies(${_target_name}
    ${rosidl_generate_interfaces_TARGET}__${_typesupport_impl}
  )

  if(NOT rosidl_generate_interfaces_SKIP_INSTALL)
    # PYTHON_INSTALL_DIR is defined by ament_cmake_python
    install(TARGETS ${_target_name}
      DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}")
  endif()
endforeach()


# Depend on rosidl_generator_py generated targets from our dependencies
foreach(_pkg_name ${rosidl_generate_interfaces_DEPENDENCY_PACKAGE_NAMES})
  target_link_libraries(${_target_name_lib} PRIVATE ${${_pkg_name}_TARGETS${rosidl_generator_py_suffix}})
endforeach()

set_target_properties(${_target_name_lib} PROPERTIES COMPILE_OPTIONS "${_extension_compile_flags}")

if(NOT rosidl_generate_interfaces_SKIP_INSTALL)
  install(TARGETS ${_target_name_lib}
    EXPORT export_${_target_name_lib}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)

  # Export this target so downstream interface packages can depend on it
  rosidl_export_typesupport_targets("${rosidl_generator_py_suffix}" "${_target_name_lib}")
  ament_export_targets(export_${_target_name_lib})
endif()

if(BUILD_TESTING AND rosidl_generate_interfaces_ADD_LINTER_TESTS)
  if(
    NOT _generated_py_files STREQUAL "" OR
    NOT _generated_extension_files STREQUAL "" OR
    NOT _generated_c_files STREQUAL ""
  )
    find_package(ament_cmake_cppcheck REQUIRED)
    ament_cppcheck(
      TESTNAME "cppcheck_rosidl_generated_py"
      "${_output_path}")

    find_package(ament_cmake_cpplint REQUIRED)
    get_filename_component(_cpplint_root "${_output_path}" DIRECTORY)
    ament_cpplint(
      TESTNAME "cpplint_rosidl_generated_py"
      # the generated code might contain functions with more lines
      FILTERS "-readability/fn_size"
      # the generated code might contain longer lines for templated types
      MAX_LINE_LENGTH 999
      ROOT "${_cpplint_root}"
      "${_output_path}")

    find_package(ament_cmake_flake8 REQUIRED)
    ament_flake8(
      TESTNAME "flake8_rosidl_generated_py"
      # the generated code might contain longer lines for templated types
      MAX_LINE_LENGTH 999
      "${_output_path}")

    find_package(ament_cmake_pep257 REQUIRED)
    ament_pep257(
      TESTNAME "pep257_rosidl_generated_py"
      "${_output_path}")

    find_package(ament_cmake_uncrustify REQUIRED)
    ament_uncrustify(
      TESTNAME "uncrustify_rosidl_generated_py"
      # the generated code might contain longer lines for templated types
      # a value of zero tells uncrustify to ignore line length
      MAX_LINE_LENGTH 0
      "${_output_path}")
  endif()
endif()
