cmake_minimum_required(VERSION 3.14)

project(rosidl_generator_py)

find_package(ament_cmake REQUIRED)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rmw)

ament_index_register_resource("rosidl_generator_packages")

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  find_package(rmw REQUIRED)
  find_package(rosidl_cmake REQUIRED)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(test_interface_files REQUIRED)

  include(cmake/register_py.cmake)
  include(cmake/rosidl_generator_py_get_typesupports.cmake)

  # Trick ament_execute_extensions into finding this extension in the source space
  set(rosidl_generator_py_DIR "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

  # Need to call extras before get_typesupports, to register the extension
  rosidl_generator_py_extras(
    "${CMAKE_CURRENT_SOURCE_DIR}/bin/rosidl_generator_py"
    "${CMAKE_CURRENT_SOURCE_DIR}/rosidl_generator_py/__init__.py;${CMAKE_CURRENT_SOURCE_DIR}/rosidl_generator_py/generate_py_impl.py"
    "${CMAKE_CURRENT_SOURCE_DIR}/resource"
  )

  # If no message files were generated, skip the test.
  rosidl_generator_py_get_typesupports(_typesupport_impls)
  if(_typesupport_impls STREQUAL "")
    message(WARNING "No valid typesupport found for Python generator, skipping tests.")
  else()
    # Use a custom suffix for the target name here to detect regressions of this:
    #   https://github.com/ros2/rosidl_python/issues/11
    # The alternative style (just using ${PROJECT_NAME}) is used by most interface packages.
    rosidl_generate_interfaces(${PROJECT_NAME}_custom
      ${test_interface_files_MSG_FILES}
      # Cases not covered by test_interface_files
      msg/BuiltinTypeSequencesIdl.idl
      msg/StringArrays.msg
      msg/Property.msg
      ADD_LINTER_TESTS
      SKIP_INSTALL
    )

    set(_append_library_dirs "")
    if(WIN32)
      set(_append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>")
    endif()

    set(pythonpath "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py;${CMAKE_CURRENT_SOURCE_DIR}")
    if(NOT WIN32)
      string(REPLACE ";" ":" pythonpath "${pythonpath}")
    endif()
    ament_add_pytest_test(test_interfaces_py "test/test_interfaces.py"
      APPEND_ENV "PYTHONPATH=${pythonpath}"
      APPEND_LIBRARY_DIRS "${_append_library_dirs}"
      WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py"
    )

    ament_add_pytest_test(test_cli_extension test/test_cli_extension.py)

    ament_add_pytest_test(test_property_py test/test_property.py
      APPEND_ENV "PYTHONPATH=${pythonpath}"
      APPEND_LIBRARY_DIRS "${_append_library_dirs}"
      WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py"
    )
  endif()
endif()

ament_package(
  CONFIG_EXTRAS "cmake/rosidl_generator_py_get_typesupports.cmake"
    "cmake/register_py.cmake"
    "rosidl_generator_py-extras.cmake.in"
)

install(
  PROGRAMS bin/rosidl_generator_py
  DESTINATION lib/rosidl_generator_py
)
install(
  DIRECTORY cmake resource
  DESTINATION share/${PROJECT_NAME}
)
