from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import FindExecutable, Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import PushRosNamespace
# from launch.actions import GroupAction
from launch.event_handlers import OnProcessStart, OnProcessExit
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue
import os

def generate_launch_description():
    urdf_package_path = get_package_share_directory('description')

    default_urdf_path = os.path.join(urdf_package_path, 'xacro', 'robot.urdf.xacro')
    default_world_path = os.path.join(urdf_package_path, 'world', 'room.world')

    declare_urdf_path = DeclareLaunchArgument(
        name='urdf',default_value=str(default_urdf_path),description='加载的urdf文件路径'
    )
    declare_world_path = DeclareLaunchArgument(
        name='world',default_value=str(default_world_path),description='加载的地图文件路径'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value='true', description='使用仿真时间'
    )
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('urdf')]), value_type=str)



    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':robot_description, 'use_sim_time':LaunchConfiguration('use_sim_time')}]
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('gazebo_ros'),'/launch','/gazebo.launch.py']
        ),
        launch_arguments=[('world',LaunchConfiguration('world')),('verbose','true'),('use_sim_time',LaunchConfiguration('use_sim_time'))]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic','/robot_description','-entity','robot','-x','0.0','-y','0.0','-z','0.2',],
        parameters=[{'use_sim_time':LaunchConfiguration('use_sim_time')}]
    )

    load_joint_controller = ExecuteProcess(
        cmd='ros2 control load_controller joint_state_broadcaster --set-state active'.split(' '),
        output='screen'
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd='ros2 control load_controller diff_drive_controller --set-state active'.split(' '),
        output='screen'
    )

    return LaunchDescription([
        declare_urdf_path,
        declare_world_path,
        declare_use_sim_time,
        robot_state_publisher,
        gazebo_launch,
        spawn_entity,
        load_joint_controller,
        load_diff_drive_controller,
        
    ])