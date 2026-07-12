cmake_minimum_required(VERSION 3.8)
project(scan_traversability_filter)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(scan_traversability_filter_node src/scan_traversability_filter_node.cpp)
target_include_directories(scan_traversability_filter_node PRIVATE include)
ament_target_dependencies(scan_traversability_filter_node
  builtin_interfaces
  geometry_msgs
  rclcpp
  sensor_msgs
  grid_map_core
  grid_map_msgs
  grid_map_ros
  tf2
  tf2_ros
)

install(TARGETS
  scan_traversability_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_height_filter test/test_height_filter.cpp)
  target_include_directories(test_height_filter PRIVATE include)
endif()

ament_export_include_directories(include)

ament_package()
