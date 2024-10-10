import os
import xacro
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true'
    )
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='True',
        description='Flag to enable joint_state_publisher GUI'
    )
    # Removed launch_rviz_arg as RViz is not launched here
    # Removed Gazebo-related arguments

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    # Removed launch_rviz configuration

    # Process xacro file
    share_dir = get_package_share_directory('dabomb_description')
    xacro_file = os.path.join(share_dir, 'urdf', 'dabomb.xacro')

    # Generate the URDF from xacro
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf},
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    #joint_state_publisher_node = Node(
    #    package='joint_state_publisher',
    #    executable='joint_state_publisher',
    #    name='joint_state_publisher',
    #    parameters=[{'use_gui': gui}],
    #    output='screen'
    #)

    return LaunchDescription([
        use_sim_time_arg,
        gui_arg,
        robot_state_publisher_node,
        #joint_state_publisher_node,
        # Removed RViz node
        # Removed Gazebo nodes
    ])

