cmake_minimum_required(VERSION 3.8)
project(grid_map_terrain_analysis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

add_executable(grid_map_terrain_node src/grid_map_terrain_node.cpp)
target_compile_features(grid_map_terrain_node PUBLIC cxx_std_17)
ament_target_dependencies(
  grid_map_terrain_node
  rclcpp
  sensor_msgs
  nav_msgs
  std_msgs
  grid_map_core
  grid_map_ros
  grid_map_msgs
  pcl_conversions
  pcl_ros
)

install(TARGETS
  grid_map_terrain_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
