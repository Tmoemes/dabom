#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Point to the YAML config file
    config_file_path = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'config',
        'rplidar_a2m8.yaml'
    )

    return LaunchDescription([

        # Define the RPLIDAR node and load parameters from YAML
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[config_file_path],  # Use the YAML file
            output='screen'),
    ])
