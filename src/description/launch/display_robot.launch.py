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
    default_rviz_path = os.path.join(urdf_package_path, 'rviz', 'display_robot.rviz')

    declare_urdf_path = DeclareLaunchArgument(
        name='urdf',default_value=str(default_urdf_path),description='加载的urdf文件路径'
    )
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('urdf')]), value_type=str)



    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':robot_description}]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', default_rviz_path]
    )

    return LaunchDescription([
        declare_urdf_path,
        robot_state_publisher,
        joint_state_publisher,
        rviz
        
    ])