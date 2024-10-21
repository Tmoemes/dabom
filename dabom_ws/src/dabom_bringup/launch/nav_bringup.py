#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')
    map_yaml_file = LaunchConfiguration('map')
    use_slam = LaunchConfiguration('use_slam')
    autostart = LaunchConfiguration('autostart', default='true')
    namespace = LaunchConfiguration('namespace', default='')
    use_namespace = LaunchConfiguration('use_namespace', default='false')

    # Determine if a map is provided
    map_provided = PythonExpression(["'", LaunchConfiguration('map'), "' != ''"])

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('dabom_bringup'),
            'config',
            'nav2_params.yaml'
        ),
        description='Full path to the ROS 2 parameters file to use'
    )

    declare_slam_params_file = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(
            get_package_share_directory('dabom_bringup'),
            'config',
            'mapper_params_online_async.yaml'
        ),
        description='Full path to the SLAM Toolbox parameters file'
    )

    declare_map_yaml_file = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map file to load (leave empty to use SLAM)'
    )

    declare_use_slam = DeclareLaunchArgument(
        'use_slam',
        default_value=PythonExpression(['not ', map_provided]),
        description='Whether to run SLAM (true if no map is provided)'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the Navigation2 stack'
    )

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )

    declare_use_namespace = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the Navigation2 stack'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the declared launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_params_file)
    ld.add_action(declare_slam_params_file)
    ld.add_action(declare_map_yaml_file)
    ld.add_action(declare_use_slam)
    ld.add_action(declare_autostart)
    ld.add_action(declare_namespace)
    ld.add_action(declare_use_namespace)

    # Get the package directories
    dabom_bringup_dir = get_package_share_directory('dabom_bringup')

    # Include rpi_bringup.py
    rpi_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(dabom_bringup_dir, 'launch', 'rpi_bringup.py')
        )
    )
    ld.add_action(LogInfo(msg="Launching Dabom Bringup..."))
    ld.add_action(rpi_bringup)

    # Include bringup_launch.py from nav2_bringup
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'slam_params_file': slam_params_file,
            'map': map_yaml_file,
            'use_slam': use_slam,
            'autostart': autostart,
            'namespace': namespace,
            'use_namespace': use_namespace
        }.items()
    )
    ld.add_action(LogInfo(msg="Launching Navigation2 Bringup..."))
    ld.add_action(nav2_bringup_launch)

    return ld
