import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription



def generate_launch_description():
    

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'),'/launch','/gazebo.launch.py']
        ),
        launch_arguments=[('world',LaunchConfiguration('world')),('verbose','true'),('use_sim_time',LaunchConfiguration('use_sim_time'))]
    )


    return LaunchDescription([
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('driver'),'/launch','/gazebo.launch.py']
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('fastlio2_wp'),'/launch','/mapping.launch.py']
            ),
        )
    ])