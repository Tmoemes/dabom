import os
import xacro
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition  # Correct import
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments
    # robot_id_arg = DeclareLaunchArgument('robot_id', default_value='dabom')  # Commented out robot_id
    # domain_id_arg = DeclareLaunchArgument('domain_id', default_value='0')  # Commented out domain_id
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='True')
    gui_arg = DeclareLaunchArgument('gui', default_value='True')
    launch_rviz_arg = DeclareLaunchArgument('launch_rviz', default_value='False')
    # launch_gazebo_arg = DeclareLaunchArgument('launch_gazebo', default_value='False')  # Commented out Gazebo argument

    # Launch configurations
    # robot_id = LaunchConfiguration('robot_id')  # Commented out robot_id
    # domain_id = LaunchConfiguration('domain_id')  # Commented out domain_id
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    launch_rviz = LaunchConfiguration('launch_rviz')
    # launch_gazebo = LaunchConfiguration('launch_gazebo')  # Commented out Gazebo configuration

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
        # namespace=robot_id,  # Commented out namespace
        parameters=[
            {'robot_description': robot_urdf},
            {'use_sim_time': use_sim_time}
        ],
        # additional_env={'ROS_DOMAIN_ID': domain_id}  # Commented out ROS domain ID
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        # namespace=robot_id,  # Commented out namespace
        # additional_env={'ROS_DOMAIN_ID': domain_id}  # Commented out ROS domain ID
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('dabomb_description'), 'config', 'display.rviz')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_rviz'))
    )

    # Gazebo-related nodes (commented out)
    # # Gazebo server using system-wide executable, only launched if launch_gazebo is true
    # gazebo_server = Node(
    #     package='gazebo_ros',
    #     executable='/usr/bin/gzserver',  # System-installed gzserver
    #     arguments=['-s', 'libgazebo_ros_factory.so', '--verbose'],
    #     output='screen',
    #     condition=IfCondition(launch_gazebo)  # Conditional launch
    # )

    # # Gazebo client using system-wide executable, conditionally launched based on 'gui' and 'launch_gazebo'
    # gazebo_client = Node(
    #     package='gazebo_ros',
    #     executable='/usr/bin/gzclient',  # System-installed gzclient
    #     arguments=['--verbose'],
    #     output='screen',
    #     condition=IfCondition(gui)  # Conditional on 'gui' argument being True
    # )

    # # Spawn entity in Gazebo, conditionally launched based on 'launch_gazebo'
    # spawn_entity = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=['-entity', 'dabomb', '-topic', 'robot_description'],
    #     output='screen',
    #     condition=IfCondition(launch_gazebo)  # Conditional launch
    # )

    return LaunchDescription([
        # robot_id_arg,  # Commented out robot_id argument
        # domain_id_arg,  # Commented out domain_id argument
        use_sim_time_arg,
        gui_arg,
        launch_rviz_arg,
        # launch_gazebo_arg,  # Commented out Gazebo argument
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        # gazebo_server,  # Commented out Gazebo server node
        # gazebo_client,  # Commented out Gazebo client node
        # spawn_entity  # Commented out Gazebo spawn entity node
    ])
