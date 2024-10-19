from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare arguments
    robot_id_arg = DeclareLaunchArgument('robot_id', default_value='dabom')
    domain_id_arg = DeclareLaunchArgument('domain_id', default_value='0')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    # Launch configurations
    robot_id = LaunchConfiguration('robot_id')
    domain_id = LaunchConfiguration('domain_id')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Nodes
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_id,
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        additional_env={'ROS_DOMAIN_ID': domain_id}
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=robot_id,
        additional_env={'ROS_DOMAIN_ID': domain_id}
    )

    return LaunchDescription([
        robot_id_arg,
        domain_id_arg,
        use_sim_time_arg,
        robot_state_publisher_node,
        joint_state_publisher_node
    ])
