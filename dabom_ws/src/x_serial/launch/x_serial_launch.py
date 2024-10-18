from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='x_serial',
            executable='x_serial_node',
            name='x_serial_node',
            output='screen'
        )
    ])
