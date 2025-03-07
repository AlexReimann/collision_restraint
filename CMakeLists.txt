cmake_minimum_required(VERSION 3.8)
project(collision_restraint)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

find_package(Eigen3 REQUIRED)

add_library(collision_restraint src/collision_restraint.cpp)
add_library(collision_restraint::collision_restraint ALIAS collision_restraint)
target_compile_features(collision_restraint PUBLIC c_std_99 cxx_std_20)
target_include_directories(collision_restraint PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(collision_restraint PUBLIC Eigen3)

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
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
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
