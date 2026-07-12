import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory("description")
    gazebo_share = get_package_share_directory("gazebo_ros")

    default_urdf = os.path.join(description_share, "xacro", "robot.urdf.xacro")
    default_world = os.path.join(description_share, "world", "multi_floor_ramp.world")

    urdf = LaunchConfiguration("urdf")
    world = LaunchConfiguration("world")
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            urdf,
            " enable_lidar:=",
            LaunchConfiguration("enable_lidar"),
            " lidar_downsample:=",
            LaunchConfiguration("lidar_downsample"),
            " lidar_update_rate:=",
            LaunchConfiguration("lidar_update_rate"),
            " lidar_visualize:=",
            LaunchConfiguration("lidar_visualize"),
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_share, "launch", "gazebo.launch.py")),
        launch_arguments={
            "world": world,
            "gui": LaunchConfiguration("gui"),
            "verbose": LaunchConfiguration("verbose"),
            "pause": LaunchConfiguration("paused"),
        }.items(),
    )

    spawn_a1 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_a1",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-entity", "a1",
            "-x", LaunchConfiguration("spawn_x"),
            "-y", LaunchConfiguration("spawn_y"),
            "-z", LaunchConfiguration("spawn_z"),
            "-Y", LaunchConfiguration("spawn_yaw"),
        ],
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="a1_controller_spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "a1_effort_controller",
            "--inactive",
            "--controller-manager-timeout", "120",
        ],
    )

    locomotion_controller = Node(
        package="a1_controller",
        executable="a1_controller_node",
        name="a1_controller",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "auto_start": LaunchConfiguration("auto_start"),
            "locomotion_mode": LaunchConfiguration("locomotion_mode"),
            "stamped_cmd_vel_topic": "/cmd_vel_stamped",
            "cmd_vel_topic": "/cmd_vel",
        }],
    )

    controller_activator = ExecuteProcess(
        cmd=[
            FindExecutable(name="ros2"),
            "control", "switch_controllers",
            "--activate", "joint_state_broadcaster", "a1_effort_controller",
            "--strict",
        ],
        name="a1_controller_activator",
        output="screen",
    )

    unpause_gazebo = ExecuteProcess(
        cmd=[
            FindExecutable(name="ros2"),
            "service", "call", "/unpause_physics", "std_srvs/srv/Empty", "{}",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("paused")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("urdf", default_value=default_urdf),
        DeclareLaunchArgument("world", default_value=default_world),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("verbose", default_value="false"),
        DeclareLaunchArgument(
            "enable_lidar",
            default_value="true",
            description="Enable the simulated MID360.",
        ),
        DeclareLaunchArgument(
            "lidar_downsample",
            default_value="5",
            description="MID360 ray stride; 5 publishes 8000 points per scan.",
        ),
        DeclareLaunchArgument(
            "lidar_update_rate",
            default_value="10",
            description="MID360 scan rate in simulation seconds.",
        ),
        DeclareLaunchArgument(
            "lidar_visualize",
            default_value="false",
            description="Render Gazebo ray visualization (expensive).",
        ),
        DeclareLaunchArgument(
            "model_database_uri",
            default_value="",
            description="Gazebo model database URI; empty disables the online database.",
        ),
        DeclareLaunchArgument("paused", default_value="true"),
        DeclareLaunchArgument("spawn_x", default_value="-5.5"),
        DeclareLaunchArgument("spawn_y", default_value="-3.5"),
        DeclareLaunchArgument("spawn_z", default_value="0.09"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        DeclareLaunchArgument("auto_start", default_value="true"),
        DeclareLaunchArgument("locomotion_mode", default_value="rl"),
        SetEnvironmentVariable(
            "GAZEBO_MODEL_DATABASE_URI", LaunchConfiguration("model_database_uri")
        ),
        robot_state_publisher,
        gazebo,
        spawn_a1,
        RegisterEventHandler(
            OnProcessExit(target_action=spawn_a1, on_exit=[controller_spawner])
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=controller_spawner,
                on_exit=[locomotion_controller],
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=locomotion_controller,
                on_start=[TimerAction(period=1.0, actions=[controller_activator])],
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=controller_activator,
                on_start=[TimerAction(period=0.5, actions=[unpause_gazebo])],
            )
        ),
    ])
