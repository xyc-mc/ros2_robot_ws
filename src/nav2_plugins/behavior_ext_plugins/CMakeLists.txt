cmake_minimum_required(VERSION 3.8)
project(behavior_ext_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

set(library_name ${PROJECT_NAME}_lib)

add_library(${library_name} SHARED
  src/back_up_twz_free_action.cpp
)

target_compile_definitions(${library_name}
  PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS"
)

ament_target_dependencies(${library_name}
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  visualization_msgs
  nav2_util
  nav2_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  nav2_costmap_2d
  nav2_core
  nav2_behaviors
  pluginlib
)

pluginlib_export_plugin_description_file(nav2_core behavior_ext_plugins.xml)

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES behavior_ext_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${library_name})

ament_package()