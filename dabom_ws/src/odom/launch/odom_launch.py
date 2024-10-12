from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='odom',
            executable='odom_node',
            name='odom_node',
            output='screen',
            parameters=[{
                'odom_topic': '/odom',
                'arduino_vel_topic': '/arduino_vel',
                'wheel_radius': 0.04,
                'lx': 0.3,
                'ly': 0.3,
            }]
        )
    ])
