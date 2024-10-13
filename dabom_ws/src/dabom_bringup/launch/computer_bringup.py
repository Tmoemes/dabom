from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the launch configuration file
    launch_config_path = os.path.join(
        get_package_share_directory('dabom_bringup'),
        'config',
        'launch_config.yaml'
    )

    # Declare Launch Arguments
    use_joy = LaunchConfiguration('use_joy')
    use_rviz = LaunchConfiguration('use_rviz')
    use_slam = LaunchConfiguration('use_slam')
    use_nav2 = LaunchConfiguration('use_nav2')

    # Declare Launch Configuration Arguments
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_joy',
            default_value='true',
            description='Launch joystick control'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz'
        ),
        DeclareLaunchArgument(
            'use_slam',
            default_value='true',
            description='Launch SLAM Toolbox'
        ),
        DeclareLaunchArgument(
            'use_nav2',
            default_value='true',
            description='Launch Nav2'
        ),

        # Conditionally launch the joystick control
        LogInfo(condition=LaunchConfiguration('use_joy'), msg="Launching joystick control..."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('dabom_joy'), 'launch', 'joy_teleop_launch.py')),
            condition=LaunchConfiguration('use_joy')
        ),

        # Conditionally launch RViz
        LogInfo(condition=LaunchConfiguration('use_rviz'), msg="Launching RViz..."),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=LaunchConfiguration('use_rviz')
        ),

        # Conditionally launch SLAM Toolbox
        LogInfo(condition=LaunchConfiguration('use_slam'), msg="Launching SLAM Toolbox..."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
            condition=LaunchConfiguration('use_slam')
        ),

        # Conditionally launch Nav2 (Navigation stack)
        LogInfo(condition=LaunchConfiguration('use_nav2'), msg="Launching Nav2..."),
        Node(
            package='nav2_bringup',
            executable='bringup_launch.py',
            name='nav2',
            output='screen',
            condition=LaunchConfiguration('use_nav2')
        )
    ])
