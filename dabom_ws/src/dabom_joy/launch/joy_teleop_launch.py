from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    joy_params = os.path.join(get_package_share_directory('dabom_joy'), 'config', 'joystick.yaml')

    game_controller_node = Node(
        package='joy',
        executable='game_controller_node',  # Other option is joy_node
        name='game_controller_node',  # This would also need to change to joy_node
        parameters=[joy_params]
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_params],
        remappings=[('/cmd_vel_joy','/cmd_vel')]
    )

    return LaunchDescription([
        game_controller_node,
        teleop_node
    ])
