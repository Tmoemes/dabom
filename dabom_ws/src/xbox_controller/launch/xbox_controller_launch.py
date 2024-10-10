from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='xbox_controller',
            executable='xbox_controller_node.py',
            name='xbox_controller_node',
            output='screen',
            emulate_tty=True,
        )
    ])
