# dabom_bringup/rpi_bringup_yaml.launch.py

import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Define the path to the YAML configuration file
    config_file_path = os.path.join(
        os.getenv('HOME'), 'Documents', 'dabom', 'dabom_ws', 'src', 'dabom_bringup', 'config', 'launch_config.yaml'
    )

    # Load the YAML configuration file
    with open(config_file_path, 'r') as file:
        config = yaml.safe_load(file)

    # Nodes to launch based on the configuration
    inverse_kinematics_node = Node(
        package='inverse_kinematics',
        executable='inverse_kinematics_node',
        name='inverse_kinematics',
        output='screen',
        condition=IfCondition(str(config.get('launch_inverse_kinematics', 'true')).lower() == 'true')
    )

    odom_node = Node(
        package='odom',
        executable='odom_node',
        name='odom',
        output='screen',
        condition=IfCondition(str(config.get('launch_odom', 'true')).lower() == 'true')
    )

    serial_comm_node = Node(
        package='serial_comm',
        executable='serial_talker',
        name='serial_comm',
        output='screen',
        condition=IfCondition(str(config.get('launch_serial_comm', 'true')).lower() == 'true')
    )

    sllidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar',
        output='screen',
        condition=IfCondition(str(config.get('launch_sllidar', 'true')).lower() == 'true')
    )

    joint_state_publisher_node = Node(
        package='unique_joint_state_publisher',
        executable='unique_joint_state_publisher_node',
        name='joint_state_publisher',
        output='screen',
        condition=IfCondition(str(config.get('launch_joint_state_publisher', 'true')).lower() == 'true')
    )

    robot_state_publisher_node = Node(
        package='dabomb_description',
        executable='robot_state.launch.py',
        name='robot_state_publisher',
        output='screen',
        condition=IfCondition(str(config.get('launch_robot_state_publisher', 'true')).lower() == 'true')
    )

    # Return the launch description with all the nodes
    return LaunchDescription([
        inverse_kinematics_node,
        odom_node,
        serial_comm_node,
        sllidar_node,
        joint_state_publisher_node,
        robot_state_publisher_node
    ])
