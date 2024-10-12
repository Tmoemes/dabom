from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='inv_kin',
            executable='inv_kin_node',
            name='inv_kin_node',
            output='screen',
            parameters=[{
                'lx': 0.3,
                'ly': 0.3,
                'wheel_radius': 0.04,
                'cmd_vel_topic': '/cmd_vel',
                'motor_vel_topic': '/motor_vel',
                'frame_id': 'base_link'
            }]
        )
    ])
