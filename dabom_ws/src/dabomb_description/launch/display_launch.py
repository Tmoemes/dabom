import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true'
    )
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='True',
        description='Flag to launch RViz'
    )
    # Removed Gazebo-related arguments

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_rviz = LaunchConfiguration('launch_rviz')
    # Removed launch_gazebo configuration

    # Include robot_state.launch.py
    #robot_state_launch = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
    #        os.path.join(
    #            get_package_share_directory('dabomb_description'),
    #            'launch',
    #            'robot_state.launch.py'
    #        )
    #    ),
    #    launch_arguments={
    #        'use_sim_time': use_sim_time
            # 'gui' can be passed here if needed, currently using default
    #    }.items()
    #)

    # RViz node with IfCondition
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            os.path.join(
                get_package_share_directory('dabomb_description'),
                'config',
                'display.rviz'
            )
        ],
        output='screen',
        condition=IfCondition(launch_rviz)
    )

    return LaunchDescription([
        use_sim_time_arg,
        launch_rviz_arg,
        #robot_state_launch,
        rviz_node,
        # Removed Gazebo nodes
    ])

