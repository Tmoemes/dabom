import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='inverse_kinematics',
            executable='inverse_kinematics_node.py',
            output='screen',
        )
    ])
