from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('dabomb_description'),
        'urdf',
        'dabomb.xacro')

    return LaunchDescription([
        # Launch robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', urdf_file])}]
        ),
        
        #Launch joint_state_publisher
        Node(
           package='joint_state_publisher',
           executable='joint_state_publisher',
           name='joint_state_publisher',
           output='screen'
        )
    ])
