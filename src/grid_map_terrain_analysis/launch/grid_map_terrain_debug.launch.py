# Copyright 2026 neepu
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('grid_map_terrain_analysis')
    params_file = os.path.join(pkg_dir, 'config', 'terrain_params.yaml')
    rviz_file = os.path.join(pkg_dir, 'rviz', 'terrain_debug.rviz')

    return LaunchDescription([
        Node(
            package='grid_map_terrain_analysis',
            executable='grid_map_terrain_node',
            name='grid_map_terrain_node',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='terrain_debug_rviz',
            output='screen',
            arguments=['-d', rviz_file],
        ),
    ])
