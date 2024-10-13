from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('odom'),
        'config',
        'odom.yaml'
    )

    return LaunchDescription([
        Node(
            package='odom',
            executable='odom_node',
            name='odom_node',
            output='screen',
            parameters=[config]
        )
    ])
