. install/setup.bash

ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"

ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/ubuntu/ros2_robot_ws/src/ros2_slam/cartographer_wp/carto3d_map/map.pbstream', include_unfinished_submaps: false}"

ros2 run cartographer_ros cartographer_pbstream_to_ros_map   -map_filestem=/home/ubuntu/ros2_robot_ws/src/ros2_slam/cartographer_wp/carto3d_map/map   -pbstream_filename=/home/ubuntu/ros2_robot_ws/src/ros2_slam/cartographer_wp/carto3d_map/map.pbstream -resolution=0.05