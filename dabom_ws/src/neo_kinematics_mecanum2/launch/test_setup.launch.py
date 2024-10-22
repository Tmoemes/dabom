import launch
import launch.actions
import launch.substitutions
import os
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions

def generate_launch_description():
    config = os.path.join(get_package_share_directory('neo_kinematics_mecanum'),'launch','test_setup.yaml')
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='neo_kinematics_mecanum', executable='neo_mecanum_node', output='screen',
            name='neo_mecanum_node', parameters = [config])
    ])