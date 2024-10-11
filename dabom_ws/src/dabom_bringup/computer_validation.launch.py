# dabom_bringup/computer_validation.launch.py

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the computer_bringup.launch.py file
    bringup_launch_path = os.path.join(
        get_package_share_directory('dabom_bringup'),
        'computer_bringup.launch.py'
    )

    # Include the computer_bringup.launch.py file (Xbox controller + RViz)
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch_path)
    )

    # Launch rqt_graph for node visualization
    rqt_graph_process = ExecuteProcess(
        cmd=['rqt_graph'],
        output='screen'
    )

    # Generate the TF tree using view_frames
    view_frames_process = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_tools', 'view_frames'],
        output='screen'
    )

    return LaunchDescription([
        # Include the computer_bringup.launch.py launch file
        bringup_launch,

        # Launch rqt_graph
        rqt_graph_process,

        # Generate TF tree
        view_frames_process
    ])
