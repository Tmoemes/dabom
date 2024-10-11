from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from typing import Optional

def generate_launch_file_path(package_name: str, launch_file_name: Optional[str] = None) -> str:
    package_dir = get_package_share_directory(package_name)
    if launch_file_name is None:
        launch_file_path =  os.path.join(
                    package_dir,
                    'launch',
                    f'{package_name}_launch.py')
    else:
        launch_file_path =  os.path.join(
                    package_dir,
                    'launch',
                    launch_file_name)

    if os.path.isfile(launch_file_path):
        return launch_file_path
    else:
        print(f"[ERROR] Launch file path does not exist: \n\t{launch_file_path}")




def generate_launch_description():
    joint_state_launch = generate_launch_file_path("unique_joint_state_publisher")
    odom_launch = generate_launch_file_path("odom")
    kinematics_launch = generate_launch_file_path("inverse_kinematics")


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(joint_state_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(odom_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(kinematics_launch)
        ),
    ])