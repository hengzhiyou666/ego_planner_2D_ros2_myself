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

# Third-party source libraries are organized under lib/ directory.
set(LIB_BSPLINE_OPT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/lib/lib_bspline_opt")
set(LIB_PATH_SEARCHING_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/lib/lib_path_searching")
set(LIB_GRID_MAP2D_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/lib/lib_grid_map2d")
set(LIB_PLAN_MANAGE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/lib/lib_plan_manage")

if(NOT EXISTS "${LIB_BSPLINE_OPT_ROOT}/src/uniform_bspline.cpp")
  message(FATAL_ERROR "Planner sources not found under lib/ directory. Please ensure lib/lib_bspline_opt exists.")
endif()

add_library(lib_local_map STATIC
  lib/lib_grid_map2d/src/grid_map.cpp
)
target_include_directories(lib_local_map PUBLIC
  "${LIB_GRID_MAP2D_ROOT}/include"
  "${LIB_GRID_MAP2D_ROOT}/include/plan_env"
)

add_library(lib_path_searching STATIC
  lib/lib_path_searching/src/dyn_a_star.cpp
)
target_include_directories(lib_path_searching PUBLIC
  "${LIB_PATH_SEARCHING_ROOT}/include"
  "${LIB_GRID_MAP2D_ROOT}/include"
  "${LIB_GRID_MAP2D_ROOT}/include/plan_env"
)
target_link_libraries(lib_path_searching PUBLIC lib_local_map)

add_library(lib_bspline_opt STATIC
  lib/lib_bspline_opt/src/uniform_bspline.cpp
  lib/lib_bspline_opt/src/bspline_optimizer.cpp
  lib/lib_bspline_opt/src/gradient_descent_optimizer.cpp
)
target_include_directories(lib_bspline_opt PUBLIC
  "${LIB_BSPLINE_OPT_ROOT}/include"
  "${LIB_PATH_SEARCHING_ROOT}/include"
  "${LIB_GRID_MAP2D_ROOT}/include"
  "${LIB_GRID_MAP2D_ROOT}/include/plan_env"
)
target_link_libraries(lib_bspline_opt PUBLIC lib_path_searching lib_local_map)

include_directories("${LIB_PLAN_MANAGE_ROOT}/include")
include_directories("${LIB_PLAN_MANAGE_ROOT}/include/plan_manage")

add_library(dog_ego_core STATIC
  application/planner_interface_dog.cpp
)
target_link_libraries(dog_ego_core
  lib_bspline_opt
  lib_path_searching
  lib_local_map
)

add_executable(dog_planner_node
  application/dog_planner_node.cpp
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
  lib_bspline_opt
  lib_path_searching
  lib_local_map
)

install(TARGETS
  dog_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# 可选：随包安装 RViz 配置，便于本机手动 `rviz2 -d .../dog_debug.rviz`（launch 不启动 rviz）
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

