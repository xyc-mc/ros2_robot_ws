from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import FindExecutable, Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart, OnProcessExit
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    this_package = get_package_share_directory('cartographer_wp')

    cartographer_config_dir = os.path.join(this_package, 'config')

    cartographer_config_basename = '2d_mapping.lua'

    rviz_config_path = os.path.join(this_package, 'rviz', 'slam.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='false',
            description='Enable use_sime_time to true'
        ),

        # DeclareLaunchArgument(
        #     name='slam_params_file', 
        #     default_value=default_params_file_path,
        #     description='Slam params'
        # ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value='false',
            description='Run rviz'
        ),

        # Cartographer node for mapping
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', cartographer_config_basename
            ],
            remappings=[
                ('scan', '/scan'),
                ('imu', '/imu')
            ]
        ),
        
        # Occupancy grid node for visualization
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'resolution': 0.05}
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration("use_sim_time")}]
        )
    ])