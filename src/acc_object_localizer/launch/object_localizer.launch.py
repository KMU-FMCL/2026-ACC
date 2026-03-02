#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('acc_object_localizer')

    # Path to the parameter file
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    # Object Localizer Node
    object_localizer_node = Node(
        package='acc_object_localizer',
        executable='object_localizer_node',
        name='object_localizer',
        output='screen',
        parameters=[params_file],
        emulate_tty=True,
    )

    return LaunchDescription([
        object_localizer_node,
    ])
