from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    # Path to the launch configuration file
    launch_config_path = os.path.join(
        get_package_share_directory('dabom_bringup'),
        'config',
        'launch_config.yaml'
    )

    # Load the launch config YAML file
    with open(launch_config_path, 'r') as f:
        config = yaml.safe_load(f)
        use_joy = config['launch_config'].get('use_joy', True)
        use_rviz = config['launch_config'].get('use_rviz', True)
        use_slam = config['launch_config'].get('use_slam', True)
        use_nav2 = config['launch_config'].get('use_nav2', True)
        use_robot_state = config['launch_config'].get('use_robot_state', True)

    # Paths to individual launch files
    joy_teleop_launch = os.path.join(get_package_share_directory('dabom_joy'), 'launch', 'joy_teleop_launch.py')
    slam_params_path = os.path.join(get_package_share_directory('dabom_bringup'), 'config', 'mapper_params_online_async.yaml')
    robot_state_launch = os.path.join(get_package_share_directory('dabomb_description'), 'launch', 'robot_state_launch.py')
    rviz_config_path = os.path.join(get_package_share_directory('dabom_bringup'), 'config', 'dabom.rviz')

    # Create the launch description
    ld = LaunchDescription()

    # Conditionally launch the robot state publisher
    if use_robot_state:
        ld.add_action(LogInfo(msg="Launching robot state publisher..."))
        ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(robot_state_launch)))

    # Conditionally launch the joystick control
    if use_joy:
        ld.add_action(LogInfo(msg="Launching joystick control..."))
        ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(joy_teleop_launch)))

    # Conditionally launch RViz
    if use_rviz:
        ld.add_action(LogInfo(msg="Launching RViz..."))
        ld.add_action(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],  # Load the custom RViz config
            output='screen',
        ))

    # Conditionally launch SLAM Toolbox with custom parameters
    if use_slam:
        ld.add_action(LogInfo(msg="Launching SLAM Toolbox..."))
        ld.add_action(Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params_path]  # Load SLAM parameters
        ))

    # Conditionally launch Nav2 (Navigation stack)
    if use_nav2:
        ld.add_action(LogInfo(msg="Launching Nav2..."))
        ld.add_action(Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            name='nav2',
            output='screen',
        ))

    return ld