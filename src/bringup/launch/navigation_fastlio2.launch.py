from launch import LaunchDescription
from launch_ros.actions import Node
# 封装终端指令相关类--------------
# from launch.actions import ExecuteProcess
# from launch.substitutions import FindExecutable
# 参数声明与获取-----------------
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
# 文件包含相关-------------------
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import AnyLaunchDescriptionSource
# 分组相关----------------------
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
# 事件相关----------------------
# from launch.event_handlers import OnProcessStart, OnProcessExit
# from launch.actions import ExecuteProcess, RegisterEventHandler,LogInfo
# 获取功能包下share目录路径-------
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    navigation_fastlio2_pkg = get_package_share_directory("navigation_fastlio2")
    fast_lio_localization_pkg = get_package_share_directory("fast_lio_localization")
    terrain_analysis_pkg = get_package_share_directory('terrain_analysis')
    terrain_analysis_ext_pkg = get_package_share_directory('terrain_analysis_ext')
    
    return LaunchDescription([
        
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [get_package_share_directory('driver'),'/launch','/gazebo.launch.py']
        #     ),
        # ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["--frame-id", "body", "--child-frame-id", "sensor"]
        ),

        Node(
            package='lio_interface',
            executable='lio_interface'
        ),

        IncludeLaunchDescription(
            launch_description_source=AnyLaunchDescriptionSource(
                launch_file_path=os.path.join(terrain_analysis_pkg,"launch","terrain_analysis.launch")
            ),
        ),

        IncludeLaunchDescription(
            launch_description_source=AnyLaunchDescriptionSource(
                launch_file_path=os.path.join(terrain_analysis_ext_pkg,"launch","terrain_analysis_ext.launch")
            ),
        ),
        
        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                launch_file_path=os.path.join(navigation_fastlio2_pkg,"launch","navigation2.launch.py")
            ),
        ),

        IncludeLaunchDescription(
            launch_description_source=PythonLaunchDescriptionSource(
                launch_file_path=os.path.join(fast_lio_localization_pkg,"launch","localization.launch.py")
            ),
            launch_arguments={
                'pcd_map_topic': '/cloud_pcd',
                'use_sim_time': 'true',
                'config_path': '/home/neepu/ros2_robot_ws/src/bringup/config',
                'map': '/home/neepu/ros2_robot_ws/src/maps_manage/map.pcd'
            }.items(),
        ),

        

        # Node(
        #     package='destination_navigator',
        #     executable='destination_navigator'
        # ),

        # Node(
        #     package='semantic_place_manager',
        #     executable='semantic_place_manager'
        # )




    ])
