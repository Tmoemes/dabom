from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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
        use_joy = config['launch_config'].get('use_joy', False)
        use_rviz = config['launch_config'].get('use_rviz', True)
        use_slam = config['launch_config'].get('use_slam', False)
        use_nav2 = config['launch_config'].get('use_nav2', True)
        use_robot_state = config['launch_config'].get('use_robot_state', True)
        use_xbox = config['launch_config'].get('use_xbox', True)

    # Paths to individual launch files
    joy_teleop_launch = os.path.join(
        get_package_share_directory('dabom_joy'), 'launch', 'joy_teleop_launch.py'
    )
    slam_params_path = os.path.join(
        get_package_share_directory('dabom_bringup'), 'config', 'mapper_params_online_async.yaml'
    )
    robot_state_launch = os.path.join(
        get_package_share_directory('dabomb_description'), 'launch', 'robot_state_launch.py'
    )
    rviz_config_path = os.path.join(
        get_package_share_directory('dabom_bringup'), 'config', 'dabom.rviz'
    )

    # Path to the custom Nav2 parameters
    nav2_params_path = os.path.join(
        get_package_share_directory('dabom_bringup'),
        'config',
        'nav2_params.yaml'  # Ensure this points to your customized parameters file
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Create the launch description
    ld = LaunchDescription()

    # Declare simulation time argument
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (for playback)'
    ))

    # Conditionally launch the robot state publisher
    if use_robot_state:
        ld.add_action(LogInfo(msg="Launching robot state publisher..."))
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_state_launch)
        ))

    # Conditionally launch the joystick control
    if use_joy:
        ld.add_action(LogInfo(msg="Launching joystick control..."))
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(joy_teleop_launch)
        ))

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
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
            launch_arguments={
                'slam_param_file': slam_params_path,
                'use_sim_time': 'false'
            }.items()
        ))

    # Conditionally launch Nav2 (Navigation stack)
    if use_nav2:
        ld.add_action(LogInfo(msg="Launching Nav2..."))
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
            launch_arguments={
                'params_file': nav2_params_path,  # Use the custom nav2_params.yaml
                'use_sim_time': 'false'
                #'map': '/dev/null'  # Override the map argument with an empty value
        }.items()
    ))

    # Conditionally launch Xbox Controller
    if use_xbox:
        ld.add_action(LogInfo(msg="Launching Xbox Controller..."))
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('xbox_controller'), 'launch', 'xbox_controller_launch.py')),
        ))

    return ld
