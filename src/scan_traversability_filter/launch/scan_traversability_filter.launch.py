# Copyright 2026 neepu
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('enable_height_filter', default_value='true'),
        DeclareLaunchArgument('height_reference_frame', default_value='body'),
        DeclareLaunchArgument('max_height_range', default_value='0.3'),
        DeclareLaunchArgument('ramped_height_range_a', default_value='0.5'),
        DeclareLaunchArgument('ramped_height_range_b', default_value='0.8'),
        DeclareLaunchArgument('ramped_height_range_c', default_value='0.35'),
        DeclareLaunchArgument('tf_timeout_sec', default_value='0.05'),
        Node(
            package='scan_traversability_filter',
            executable='scan_traversability_filter_node',
            name='scan_traversability_filter',
            output='screen',
            parameters=[{
                'scan_topic': '/registered_scan',
                'grid_map_topic': '/elevation_mapping_node/elevation_map_raw',
                'output_topic': '/terrain_map',
                'traversability_layer': 'traversability',
                'output_frame_id': 'map',
                'unknown_as_obstacle': False,
                'min_risk_to_publish': 0.0,
                'clamp_traversability': True,
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
                'enable_height_filter': ParameterValue(
                    LaunchConfiguration('enable_height_filter'), value_type=bool),
                'height_reference_frame': LaunchConfiguration(
                    'height_reference_frame'),
                'max_height_range': ParameterValue(
                    LaunchConfiguration('max_height_range'), value_type=float),
                'ramped_height_range_a': ParameterValue(
                    LaunchConfiguration('ramped_height_range_a'), value_type=float),
                'ramped_height_range_b': ParameterValue(
                    LaunchConfiguration('ramped_height_range_b'), value_type=float),
                'ramped_height_range_c': ParameterValue(
                    LaunchConfiguration('ramped_height_range_c'), value_type=float),
                'tf_timeout_sec': ParameterValue(
                    LaunchConfiguration('tf_timeout_sec'), value_type=float),
            }],
        )
    ])
