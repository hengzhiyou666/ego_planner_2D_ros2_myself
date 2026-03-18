cmake_minimum_required(VERSION 3.22)
project(dog_ego_planner)

# Generate compile_commands.json for C++ language servers (clangd/IntelliSense).
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_compile_options(-O3 -Wall -Wextra -Wpedantic)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)

# Vendorized (copied) demo planner sources: no dependency on src_demo/.
set(VENDOR_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/vendor")

if(NOT EXISTS "${VENDOR_ROOT}/bspline_opt/src/uniform_bspline.cpp")
  message(FATAL_ERROR "Vendor planner sources not found at: ${VENDOR_ROOT}. Please ensure vendor/ is present in src_new/dog_ego_planner.")
endif()

add_library(vendor_local_map STATIC
  vendor/GridMap2D/src/grid_map.cpp
)
target_include_directories(vendor_local_map PUBLIC
  "${VENDOR_ROOT}/GridMap2D/include"
  "${VENDOR_ROOT}/GridMap2D/include/plan_env"
)

add_library(vendor_path_searching STATIC
  vendor/path_searching/src/dyn_a_star.cpp
)
target_include_directories(vendor_path_searching PUBLIC
  "${VENDOR_ROOT}/path_searching/include"
  "${VENDOR_ROOT}/GridMap2D/include"
  "${VENDOR_ROOT}/GridMap2D/include/plan_env"
)
target_link_libraries(vendor_path_searching PUBLIC vendor_local_map)

add_library(vendor_bspline_opt STATIC
  vendor/bspline_opt/src/uniform_bspline.cpp
  vendor/bspline_opt/src/bspline_optimizer.cpp
  vendor/bspline_opt/src/gradient_descent_optimizer.cpp
)
target_include_directories(vendor_bspline_opt PUBLIC
  "${VENDOR_ROOT}/bspline_opt/include"
  "${VENDOR_ROOT}/path_searching/include"
  "${VENDOR_ROOT}/GridMap2D/include"
  "${VENDOR_ROOT}/GridMap2D/include/plan_env"
)
target_link_libraries(vendor_bspline_opt PUBLIC vendor_path_searching vendor_local_map)

include_directories("${VENDOR_ROOT}/plan_manage/include")
include_directories("${VENDOR_ROOT}/plan_manage/include/plan_manage")

add_library(dog_ego_core STATIC
  src/planner_interface_dog.cpp
)
target_link_libraries(dog_ego_core
  vendor_bspline_opt
  vendor_path_searching
  vendor_local_map
)

add_executable(dog_planner_node
  src/dog_planner_node.cpp
)
ament_target_dependencies(dog_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
)

target_link_libraries(dog_planner_node
  dog_ego_core
  vendor_bspline_opt
  vendor_path_searching
  vendor_local_map
)

install(TARGETS
  dog_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install RViz config with preserved path (avoid installing duplicate launch file).
install(DIRECTORY
  planner/plan_manage/rviz
  DESTINATION share/${PROJECT_NAME}/planner/plan_manage
)

ament_package()

