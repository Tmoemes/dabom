from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare arguments
    robot_id_arg = DeclareLaunchArgument('robot_id', default_value='dabom')
    domain_id_arg = DeclareLaunchArgument('domain_id', default_value='0')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='False')

    # Launch configurations
    robot_id = LaunchConfiguration('robot_id')
    domain_id = LaunchConfiguration('domain_id')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Include robot state launch
    robot_state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('dabomb_description'), '/launch/robot_state.launch.py']),
        launch_arguments={
            'robot_id': robot_id,
            'domain_id': domain_id,
            'use_sim_time': use_sim_time
        }.items()
    )

    # Optional: Add any headless-specific nodes or configurations here

    return LaunchDescription([
        robot_id_arg,
        domain_id_arg,
        use_sim_time_arg,
        robot_state_launch
    ])
