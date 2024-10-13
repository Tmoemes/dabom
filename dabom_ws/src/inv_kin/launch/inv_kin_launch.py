from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('inv_kin'),
        'config',
        'inv_kin.yaml'
    )

    return LaunchDescription([
        Node(
            package='inv_kin',
            executable='inv_kin_node',
            name='inv_kin_node',
            output='screen',
            parameters=[config]
        )
    ])
