from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
from typing import Optional

# Helper function to generate launch file paths dynamically
def generate_launch_file_path(package_name: str, launch_file_name: Optional[str] = None) -> str:
    package_dir = get_package_share_directory(package_name)
    if launch_file_name is None:
        launch_file_path = os.path.join(
            package_dir,
            'launch',
            f'{package_name}_launch.py'
        )
    else:
        launch_file_path = os.path.join(
            package_dir,
            'launch',
            launch_file_name
        )

    if os.path.isfile(launch_file_path):
        return launch_file_path
    else:
        print(f"[ERROR] Launch file path does not exist: \n\t{launch_file_path}")
        print("QUITTING")
        quit()

def generate_launch_description():
    # Path to the RViz configuration file
    rviz_config_file = LaunchConfiguration('rviz_config', default='dabomb_description/config/display.rviz')

    # List of package names and their optional launch files
    packages_launch_files = [
        ["xbox_controller"],
    ]

    # Store launch descriptions
    launch_descriptions = []

    # Dynamically include launch files for packages
    for package in packages_launch_files:
        launch_description = None
        if len(package) == 2:
            launch_description = generate_launch_file_path(package[0], package[1])
        elif len(package) == 1:
            launch_description = generate_launch_file_path(package[0])
        else:
            print("[ERROR] Check the packages_launch_files list; each element should contain 1 or 2 elements")
            continue  # Skip invalid entries

        launch_descriptions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_description)
            )
        )

    # RViz node with configuration file (manually added as it's not from a launch file)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Append RViz node to the list of launch descriptions
    launch_descriptions.append(rviz_node)

    # Return the complete launch description
    return LaunchDescription(launch_descriptions)

