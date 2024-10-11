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
        print("QUITTING")
        quit()





def generate_launch_description():
    packages_launch_files = [
        ["unique_joint_state_publisher"],
        ["odom"],
        ["inverse_kinematics"],
        ["sllidar_ros2", "sllidar_a2m8_launch.py"],
        ["unique_joint_state_publisher"],
        ["serial_comm", "serial_talker_launch.py"],
        ["dabomb_description", "display_launch.py"],
        ["dabomb_description", "robot_state_launch.py"]
        ]
    
    launch_descriptions = []

    for package in packages_launch_files:
        launch_description = None
        if len(package) == 2:
            launch_description = generate_launch_file_path(package[0], package[1])
        elif len(package) == 1:
            launch_description = generate_launch_file_path(package[0])
        else:
            print("[ERROR] check package_launch_files list in master launch element containts to many elements should be 1 or 2")
        launch_descriptions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_description)))
        


    return LaunchDescription(launch_descriptions)