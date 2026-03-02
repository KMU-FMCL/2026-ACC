#!/usr/bin/env python3
# Copyright 2026 FMCL (Future Mobility Control Lab), Kookmin University
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
"""
Launch file for RANSAC Object Localizer.

This launch file starts the ransac_localizer_node with configurable parameters.
The node localizes objects detected by YOLO using depth information and RANSAC.

Usage:
  ros2 launch qcar2_ransac ransac_localizer_launch.py
  ros2 launch qcar2_ransac ransac_localizer_launch.py use_sim_time:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('qcar2_ransac')

    # Paths
    default_params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the parameter file'
    )

    # RANSAC Localizer Node
    ransac_localizer_node = Node(
        package='qcar2_ransac',
        executable='ransac_localizer_node',
        name='ransac_localizer',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # Remap topics if needed
            # ('/yolo_detections', '/yolo_detections'),
            # ('/camera/depth_image', '/camera/depth_image'),
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        ransac_localizer_node,
    ])
