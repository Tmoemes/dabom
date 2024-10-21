#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package directories
    dabom_bringup_dir = get_package_share_directory('dabom_bringup')
    rplidar_ros_dir = get_package_share_directory('rplidar_ros')
    serial_comm_dir = get_package_share_directory('serial_comm')
    x_serial_dir = get_package_share_directory('x_serial')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    inv_kin_dir = get_package_share_directory('inv_kin')
    odom_dir = get_package_share_directory('odom')
    dabomb_description_dir = get_package_share_directory('dabomb_description')

    # Declare launch arguments
    use_serial_talker = LaunchConfiguration('use_serial_talker')
    use_x_serial = LaunchConfiguration('use_x_serial')
    use_inv_kin = LaunchConfiguration('use_inv_kin')
    use_odom = LaunchConfiguration('use_odom')
    use_rplidar = LaunchConfiguration('use_rplidar')
    use_slam_pi = LaunchConfiguration('use_slam_pi')
    use_nav2_pi = LaunchConfiguration('use_nav2_pi')
    use_robot_state = LaunchConfiguration('use_robot_state')

    declare_use_serial_talker = DeclareLaunchArgument(
        'use_serial_talker', default_value='true',
        description='Enable Serial Talker')
    declare_use_x_serial = DeclareLaunchArgument(
        'use_x_serial', default_value='false',
        description='Enable X Serial')
    declare_use_inv_kin = DeclareLaunchArgument(
        'use_inv_kin', default_value='true',
        description='Enable Inverse Kinematics Node')
    declare_use_odom = DeclareLaunchArgument(
        'use_odom', default_value='true',
        description='Enable Odometry Node')
    declare_use_rplidar = DeclareLaunchArgument(
        'use_rplidar', default_value='true',
        description='Enable RPLIDAR')
    declare_use_slam_pi = DeclareLaunchArgument(
        'use_slam_pi', default_value='true',
        description='Enable SLAM on Pi')
    declare_use_nav2_pi = DeclareLaunchArgument(
        'use_nav2_pi', default_value='false',
        description='Enable Nav2 on Pi')
    declare_use_robot_state = DeclareLaunchArgument(
        'use_robot_state', default_value='true',
        description='Enable Robot State Publisher')

    # Paths to config files
    common_params_path = os.path.join(dabom_bringup_dir, 'config', 'common_params.yaml')
    slam_params_path = os.path.join(dabom_bringup_dir, 'config', 'mapper_params_online_async.yaml')
    nav2_params_path = os.path.join(dabom_bringup_dir, 'config', 'nav2_params.yaml')
    robot_state_launch = os.path.join(dabomb_description_dir, 'launch', 'robot_state_launch.py')

    # Create the launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_serial_talker)
    ld.add_action(declare_use_x_serial)
    ld.add_action(declare_use_inv_kin)
    ld.add_action(declare_use_odom)
    ld.add_action(declare_use_rplidar)
    ld.add_action(declare_use_slam_pi)
    ld.add_action(declare_use_nav2_pi)
    ld.add_action(declare_use_robot_state)

    # Function to conditionally launch nodes
    def launch_nodes(context, *args, **kwargs):
        launches = []

        if context.launch_configurations['use_inv_kin'] == 'true':
            launches.append(LogInfo(msg="Launching inv_kin..."))
            launches.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(inv_kin_dir, 'launch', 'inv_kin_launch.py')),
                launch_arguments={'common_params_file': common_params_path}.items()
            ))

        if context.launch_configurations['use_odom'] == 'true':
            launches.append(LogInfo(msg="Launching odom..."))
            launches.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(odom_dir, 'launch', 'odom_launch.py')),
                launch_arguments={'common_params_file': common_params_path}.items()
            ))

        if context.launch_configurations['use_serial_talker'] == 'true':
            launches.append(LogInfo(msg="Launching serial talker..."))
            launches.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(serial_comm_dir, 'launch', 'serial_talker_launch.py'))
            ))

        if context.launch_configurations['use_x_serial'] == 'true':
            launches.append(LogInfo(msg="Launching x_serial..."))
            launches.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(x_serial_dir, 'launch', 'x_serial_launch.py'))
            ))

        if context.launch_configurations['use_rplidar'] == 'true':
            launches.append(LogInfo(msg="Launching RPLIDAR..."))
            launches.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(rplidar_ros_dir, 'launch', 'rplidar_a2m8_launch.py'))
            ))

        if context.launch_configurations['use_slam_pi'] == 'true':
            launches.append(LogInfo(msg="Launching SLAM Toolbox..."))
            launches.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')),
                launch_arguments={
                    'slam_params_file': slam_params_path,
                    'use_sim_time': 'false'
                }.items()
            ))

        if context.launch_configurations['use_nav2_pi'] == 'true':
            launches.append(LogInfo(msg="Launching Nav2..."))
            launches.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')),
                launch_arguments={
                    'params_file': nav2_params_path,
                    'use_sim_time': 'false'
                }.items()
            ))

        if context.launch_configurations['use_robot_state'] == 'true':
            launches.append(LogInfo(msg="Launching robot state publisher..."))
            launches.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot_state_launch)
            ))

        return launches

    # Add OpaqueFunction to launch nodes
    ld.add_action(OpaqueFunction(function=launch_nodes))

    return ld
