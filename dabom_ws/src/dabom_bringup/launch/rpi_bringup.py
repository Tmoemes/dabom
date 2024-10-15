from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    # Paths to common parameters and launch config files
    config_dir = get_package_share_directory('dabom_bringup')
    common_params_path = os.path.join(config_dir, 'config', 'common_params.yaml')
    launch_config_path = os.path.join(config_dir, 'config', 'launch_config.yaml')

    # Load launch configuration
    with open(launch_config_path, 'r') as f:
        config = yaml.safe_load(f)['launch_config']
        use_serial_talker = config.get('use_serial_talker', True)
        use_x_serial = config.get('use_x_serial', True)
        use_inv_kin = config.get('use_inv_kin', True)
        use_odom = config.get('use_odom', True)
        use_rplidar = config.get('use_rplidar', True)
        use_slam = config.get('use_slam', True)

    # Paths to individual launch files
    launch_files = {
        'rplidar': os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a2m8_launch.py'),
        'serial_talker': os.path.join(get_package_share_directory('serial_comm'), 'launch', 'serial_talker_launch.py'),
        'x_serial': os.path.join(get_package_share_directory('x_serial'), 'launch', 'x_serial_launch.py')
    }
    
    slam_params_path = os.path.join(config_dir, 'config', 'mapper_params_online_async.yaml')

    # Create the launch description
    ld = LaunchDescription()

    # Conditionally add inv_kin and odom nodes with common parameters
    if use_inv_kin:
        ld.add_action(LogInfo(msg="Launching inv_kin_node..."))
        ld.add_action(Node(
            package='inv_kin',
            executable='inv_kin_node',
            name='inv_kin_node',
            parameters=[common_params_path],
            output='screen'
        ))

    if use_odom:
        ld.add_action(LogInfo(msg="Launching odom_node..."))
        ld.add_action(Node(
            package='odom',
            executable='odom_node',
            name='odom_node',
            parameters=[common_params_path],
            output='screen'
        ))

    # Conditionally include the serial talker and x_serial based on launch_config.yaml
    if use_serial_talker:
        ld.add_action(LogInfo(msg="Launching serial talker..."))
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_files['serial_talker'])
        ))

    if use_x_serial:
        ld.add_action(LogInfo(msg="Launching x_serial..."))
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_files['x_serial'])
        ))

    # Conditionally include RPLIDAR
    if use_rplidar:
        ld.add_action(LogInfo(msg="Launching RPLIDAR..."))
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_files['rplidar'])
        ))

    # Conditionally launch SLAM Toolbox with custom parameters
    if use_slam:
        ld.add_action(LogInfo(msg="Launching SLAM Toolbox..."))
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
            launch_arguments={
                'slam_param_file': slam_params_path,
                'use_sim_time': 'false'
            }.items()
        ))

    return ld
