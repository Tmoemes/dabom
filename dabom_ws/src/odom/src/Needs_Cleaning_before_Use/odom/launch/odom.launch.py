from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='odom',
            executable='odom_node',
            name='odom_node',
            output='screen',
            parameters=[]
        ),
    ])
