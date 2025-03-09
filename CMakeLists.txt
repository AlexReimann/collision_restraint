cmake_minimum_required(VERSION 3.8)
project(collision_restraint)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(collision_restraint
  src/collision_restraint.cpp
  src/footprint.cpp
  src/model.cpp
  src/node.cpp
  src/polar_point.cpp
)
add_library(collision_restraint::collision_restraint ALIAS collision_restraint)
target_compile_features(collision_restraint PUBLIC c_std_99 cxx_std_20)
target_include_directories(collision_restraint PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(collision_restraint PUBLIC Eigen3)
ament_target_dependencies(collision_restraint
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(collision_restraint PRIVATE "COLLISION_RESTRAINT_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS collision_restraint
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_cppcheck REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  ament_cppcheck()
  ament_cpplint(
    FILTERS "+build,-build/include_order,+readability,+runtime,+whitespace,-whitespace/braces,-whitespace/indent,-legal/copyright"
  )
  ament_lint_cmake()
  ament_xmllint()

endif()

ament_export_include_directories(
  "include/${PROJECT_NAME}"
)
ament_export_libraries(
  collision_restraint
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_package()
