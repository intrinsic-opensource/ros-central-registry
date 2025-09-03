cmake_minimum_required(VERSION 3.5)
project(rmw_test_fixture LANGUAGES NONE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros_core REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} INTERFACE
  rmw::rmw
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(${PROJECT_NAME})
ament_export_dependencies(rmw)

ament_package()
