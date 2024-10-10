from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='unique_joint_state_publisher',
            executable='unique_joint_state_publisher_node.py',
            name='unique_joint_state_publisher_node',
            output='screen',
            emulate_tty=True,
        )
    ])
