from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # Declare arguments
    robot_id_arg = DeclareLaunchArgument('robot_id', default_value='dabom')
    domain_id_arg = DeclareLaunchArgument('domain_id', default_value='0')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='False')
    launch_rviz_arg = DeclareLaunchArgument('launch_rviz', default_value='True')
    launch_gazebo_arg = DeclareLaunchArgument('launch_gazebo', default_value='False')  # New argument to conditionally launch Gazebo

    # Launch configurations
    robot_id = LaunchConfiguration('robot_id')
    domain_id = LaunchConfiguration('domain_id')
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_gazebo = LaunchConfiguration('launch_gazebo')  # Gazebo condition

    # Include robot state launch (specifically for RViz without Gazebo)
    robot_state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('dabomb_description'), '/launch/robot_state.launch.py']),
        launch_arguments={
            'robot_id': robot_id,
            'domain_id': domain_id,
            'use_sim_time': use_sim_time
        }.items()
    )

    # RViz node with IfCondition
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('dabomb_description'), 'config', 'display.rviz')],
        output='screen',
        namespace=robot_id,
        condition=IfCondition(launch_rviz)  # Conditional on 'launch_rviz' argument being True
    )

    # Conditionally launch Gazebo based on 'launch_gazebo' argument
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py']),
        condition=IfCondition(launch_gazebo)  # Launch Gazebo if 'launch_gazebo' is True
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py']),
        condition=IfCondition(launch_gazebo)  # Launch Gazebo GUI if 'launch_gazebo' is True
    )

    return LaunchDescription([
        robot_id_arg,
        domain_id_arg,
        use_sim_time_arg,
        launch_rviz_arg,
        launch_gazebo_arg,  # Add the Gazebo argument
        robot_state_launch,
        rviz_node,
        gazebo_server,  # Conditionally launch Gazebo server
        gazebo_client   # Conditionally launch Gazebo client
    ])
