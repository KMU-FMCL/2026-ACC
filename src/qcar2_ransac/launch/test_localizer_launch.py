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
Launch file for Test Localizer (Python).

Usage:
  ros2 launch qcar2_ransac test_localizer_launch.py
  ros2 launch qcar2_ransac test_localizer_launch.py use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Test Localizer Node
    test_localizer_node = Node(
        package='qcar2_ransac',
        executable='test_localizer.py',
        name='test_localizer',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),

            # RGB camera intrinsics
            'rgb_fx': 455.2,
            'rgb_fy': 459.43,
            'rgb_cx': 308.53,
            'rgb_cy': 213.56,

            # Depth camera intrinsics
            'depth_fx': 385.6,
            'depth_fy': 385.6,
            'depth_cx': 321.9,
            'depth_cy': 237.3,

            # Depth processing
            'min_depth': 0.2,
            'max_depth': 5.0,
            'depth_scale': 0.001,

            # ROI
            'bbox_scale': 0.4,

            # Tracking (enhanced noise robustness)
            'tracking_distance': 1.5,
            'min_measurements': 15,
            'std_threshold': 0.5,
            'outlier_threshold': 1.5,
            'duplicate_distance': 0.5,
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        test_localizer_node,
    ])
