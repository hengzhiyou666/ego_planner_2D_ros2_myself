# Copyright 2026 hengzhiyou
# SPDX-License-Identifier: GPL-3.0-only

cmake_minimum_required(VERSION 3.21)
project(dog_ego_planner VERSION 1.0.0 LANGUAGES CXX)

option(DOG_EGO_PLANNER_WARNINGS_AS_ERRORS "Treat first-party compiler warnings as errors" OFF)
option(DOG_EGO_PLANNER_ENABLE_LINT "Enable first-party ament lint tests" ON)
set(DOG_EGO_PLANNER_VENDOR_SYSROOT_PREFIX "" CACHE STRING
  "Relocate stale absolute /usr paths exported below this vendor sysroot prefix"
)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Some vendor ROS installations export native host libraries through a stale
# build-time sysroot (for example /sysroot/usr/lib/libpython.so).  Keep the
# workaround opt-in so normal desktop and cross-compilation builds are not
# changed.  It only rewrites usage requirements on imported dependency targets.
function(relocate_vendor_sysroot_usage_requirements vendor_prefix)
  if("${vendor_prefix}" STREQUAL "")
    return()
  endif()

  if(NOT IS_ABSOLUTE "${vendor_prefix}" OR "${vendor_prefix}" STREQUAL "/")
    message(FATAL_ERROR
      "DOG_EGO_PLANNER_VENDOR_SYSROOT_PREFIX must be an absolute path other than /"
    )
  endif()

  get_property(imported_targets DIRECTORY PROPERTY IMPORTED_TARGETS)
  set(relocated_property_count 0)
  foreach(imported_target IN LISTS imported_targets)
    foreach(target_property INTERFACE_LINK_LIBRARIES INTERFACE_INCLUDE_DIRECTORIES)
      get_target_property(original_value "${imported_target}" "${target_property}")
      if(NOT "${original_value}" STREQUAL "" AND NOT "${original_value}" MATCHES "-NOTFOUND$")
        string(REPLACE "${vendor_prefix}/usr/" "/usr/" relocated_value "${original_value}")
        if(NOT "${relocated_value}" STREQUAL "${original_value}")
          set_property(TARGET "${imported_target}" PROPERTY "${target_property}" "${relocated_value}")
          math(EXPR relocated_property_count "${relocated_property_count} + 1")
        endif()
      endif()
    endforeach()
  endforeach()

  message(STATUS
    "Relocated ${relocated_property_count} imported usage requirements from ${vendor_prefix}/usr"
  )
endfunction()

relocate_vendor_sysroot_usage_requirements("${DOG_EGO_PLANNER_VENDOR_SYSROOT_PREFIX}")

function(enable_project_warnings target)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
    if(DOG_EGO_PLANNER_WARNINGS_AS_ERRORS)
      target_compile_options(${target} PRIVATE -Werror)
    endif()
  elseif(MSVC)
    target_compile_options(${target} PRIVATE /W4)
    if(DOG_EGO_PLANNER_WARNINGS_AS_ERRORS)
      target_compile_options(${target} PRIVATE /WX)
    endif()
  endif()
endfunction()

add_library(dog_ego_grid_map STATIC
  src/mapping/grid_map.cpp
)
target_include_directories(dog_ego_grid_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)
target_link_libraries(dog_ego_grid_map PUBLIC Eigen3::Eigen)
enable_project_warnings(dog_ego_grid_map)

add_library(dog_ego_path_searching STATIC
  src/planning/a_star.cpp
)
target_include_directories(dog_ego_path_searching PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)
target_link_libraries(dog_ego_path_searching PUBLIC dog_ego_grid_map Eigen3::Eigen)
enable_project_warnings(dog_ego_path_searching)

add_library(dog_ego_bspline STATIC
  third_party/ego_planner/src/uniform_bspline.cpp
  third_party/ego_planner/src/bspline_optimizer.cpp
  third_party/ego_planner/src/gradient_descent_optimizer.cpp
)
target_include_directories(dog_ego_bspline SYSTEM PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/ego_planner/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/lbfgs_lite/include>
)
target_link_libraries(dog_ego_bspline PUBLIC
  dog_ego_path_searching
  dog_ego_grid_map
  Eigen3::Eigen
)

add_library(dog_ego_core STATIC
  src/planning/planner_interface_dog.cpp
)
target_include_directories(dog_ego_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)
target_link_libraries(dog_ego_core PUBLIC
  dog_ego_bspline
  dog_ego_path_searching
  dog_ego_grid_map
  Eigen3::Eigen
)
enable_project_warnings(dog_ego_core)

add_library(dog_ego_path_processing STATIC
  src/path_processing/path_utils.cpp
)
target_include_directories(dog_ego_path_processing PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)
target_link_libraries(dog_ego_path_processing PUBLIC Eigen3::Eigen)
enable_project_warnings(dog_ego_path_processing)

add_library(dog_ego_point_cloud STATIC
  src/node/point_cloud_processor.cpp
)
target_include_directories(dog_ego_point_cloud PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)
target_link_libraries(dog_ego_point_cloud Eigen3::Eigen)
ament_target_dependencies(dog_ego_point_cloud geometry_msgs sensor_msgs)
enable_project_warnings(dog_ego_point_cloud)

add_library(dog_ego_node STATIC
  src/node/dog_planner_node.cpp
)
target_include_directories(dog_ego_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)
target_link_libraries(dog_ego_node
  dog_ego_core
  dog_ego_path_processing
  dog_ego_point_cloud
  Eigen3::Eigen
)
ament_target_dependencies(dog_ego_node
  builtin_interfaces
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)
enable_project_warnings(dog_ego_node)

add_executable(dog_planner_node src/node/main.cpp)
target_include_directories(dog_planner_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(dog_planner_node dog_ego_node)
ament_target_dependencies(dog_planner_node rclcpp)
enable_project_warnings(dog_planner_node)

install(TARGETS dog_planner_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config rviz LICENSES
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(FILES README.md CHANGELOG.rst CONTRIBUTING.md LICENSE
  DESTINATION share/${PROJECT_NAME}
)
install(FILES docs/DEPLOYMENT_DOG3.md
  DESTINATION share/${PROJECT_NAME}/docs
)
install(PROGRAMS scripts/test_isolated.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  if(DOG_EGO_PLANNER_ENABLE_LINT)
    find_package(ament_cmake_cppcheck REQUIRED)
    find_package(ament_cmake_cpplint REQUIRED)
    find_package(ament_cmake_flake8 REQUIRED)
    find_package(ament_cmake_lint_cmake REQUIRED)
    find_package(ament_cmake_pep257 REQUIRED)
    find_package(ament_cmake_uncrustify REQUIRED)
    find_package(ament_cmake_xmllint REQUIRED)

    set(first_party_source_files
      src/mapping/grid_map.cpp
      src/mapping/grid_map.hpp
      src/node/dog_planner_node.cpp
      src/node/dog_planner_node.hpp
      src/node/main.cpp
      src/node/point_cloud_processor.cpp
      src/node/point_cloud_processor.hpp
      src/path_processing/path_utils.cpp
      src/path_processing/path_utils.hpp
      src/planning/a_star.cpp
      src/planning/a_star.hpp
      src/planning/planning_state.hpp
      src/planning/planner_interface_dog.cpp
      src/planning/planner_interface_dog.hpp
    )
    set(first_party_test_files
      test/test_a_star.cpp
      test/test_bspline.cpp
      test/test_grid_map.cpp
      test/test_path_utils.cpp
      test/test_point_cloud_processor.cpp
      test/test_node_parameters.cpp
    )
    set(first_party_cxx_files ${first_party_source_files} ${first_party_test_files})
    ament_cppcheck(${first_party_source_files})
    ament_cpplint(
      ${first_party_cxx_files}
      FILTERS -build/include_order
      MAX_LINE_LENGTH 120
    )
    ament_uncrustify(${first_party_cxx_files} MAX_LINE_LENGTH 120)
    ament_flake8(
      launch
      test/test_launch.py
      test/test_node_integration.py
    )
    ament_pep257(launch test/test_launch.py test/test_node_integration.py)
    ament_lint_cmake(CMakeLists.txt)
    ament_xmllint(package.xml)
  endif()

  ament_add_gtest(test_grid_map test/test_grid_map.cpp)
  if(TARGET test_grid_map)
    target_link_libraries(test_grid_map dog_ego_grid_map)
    enable_project_warnings(test_grid_map)
  endif()

  ament_add_gtest(test_a_star test/test_a_star.cpp)
  if(TARGET test_a_star)
    target_link_libraries(test_a_star dog_ego_path_searching dog_ego_grid_map)
    enable_project_warnings(test_a_star)
  endif()

  ament_add_gtest(test_path_utils test/test_path_utils.cpp)
  if(TARGET test_path_utils)
    target_link_libraries(test_path_utils dog_ego_path_processing)
    enable_project_warnings(test_path_utils)
  endif()

  ament_add_gtest(test_point_cloud_processor test/test_point_cloud_processor.cpp)
  if(TARGET test_point_cloud_processor)
    target_link_libraries(test_point_cloud_processor dog_ego_point_cloud)
    enable_project_warnings(test_point_cloud_processor)
  endif()

  ament_add_gtest(test_node_parameters test/test_node_parameters.cpp)
  if(TARGET test_node_parameters)
    target_link_libraries(test_node_parameters dog_ego_node)
    ament_target_dependencies(test_node_parameters rclcpp)
    enable_project_warnings(test_node_parameters)
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_bspline.cpp")
    ament_add_gtest(test_bspline test/test_bspline.cpp)
    if(TARGET test_bspline)
      target_link_libraries(test_bspline dog_ego_core dog_ego_bspline)
      enable_project_warnings(test_bspline)
    endif()
  endif()

  add_launch_test(test/test_launch.py TIMEOUT 30)
  add_launch_test(test/test_node_integration.py TIMEOUT 30)

  set(isolated_test_environment
    "ROS_DOMAIN_ID=232"
    "ROS_LOCALHOST_ONLY=1"
    "RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
    "ROS2CLI_NO_DAEMON=1"
  )
  set_tests_properties(
    test_grid_map
    test_a_star
    test_path_utils
    test_point_cloud_processor
    test_node_parameters
    test_bspline
    test_test_launch.py
    test_test_node_integration.py
    PROPERTIES ENVIRONMENT "${isolated_test_environment}"
  )
endif()

ament_package()
