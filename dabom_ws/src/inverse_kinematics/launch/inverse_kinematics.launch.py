import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Set Domain ID and Namespace
    os.environ['ROS_DOMAIN_ID'] = '42'

    return LaunchDescription([
        Node(
            package='inverse_kinematics',
            executable='inverse_kinematics_node.py',
            namespace='robot_namespace',  # Customize the namespace here
            output='screen',
        )
    ])
