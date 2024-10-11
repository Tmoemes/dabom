from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='odom',
            executable='odom_node.py',
            name='mecanum_odometry_node',
            output='screen',
            parameters=[],
            emulate_tty=True  # Helps in proper logging display
        )
    ])
