from launch import LaunchDescription
from launch_ros.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare('sllidar_ros2'), '/src/sllidar_ros2/launch/sllidar_a2m8_launch.py']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare('sub_package_2'), '/launch/sub_package_2_launch_file.launch.py']),
        ),
    ])
