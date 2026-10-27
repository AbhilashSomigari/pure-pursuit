"""Launch the pure pursuit node with parameters from config/params.yaml."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('pure_pursuit_pkg'),
        'config', 'params.yaml')
    return LaunchDescription([
        Node(
            package='pure_pursuit_pkg',
            executable='pure_pursuit_node',
            name='pure_pursuit_node',
            output='screen',
            parameters=[params],
        ),
    ])
