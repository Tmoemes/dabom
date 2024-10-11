# dabom_bringup/computer_bringup.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Path to the RViz configuration file
    rviz_config_file = LaunchConfiguration('rviz_config', default='dabomb_description/config/display.rviz')

    # Xbox controller node
    xbox_controller_node = Node(
        package='xbox_controller',
        executable='xbox_controller_node',
        name='xbox_controller',
        output='screen'
    )

    # RViz node with configuration file
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        # Xbox controller
        xbox_controller_node,
        # RViz with custom config
        rviz_node
    ])
