from launch import LaunchDescription
#from launch.substitution import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('ovcrobot_control')
    config_file = os.path.join(package_dir, 'config', 'pid_parameters.yaml')
    return LaunchDescription([
        Node(
            package='ovcrobot_control',
            executable='ovcrobot_pid',
            name='ovcrobot_PID',
            output='screen',
            parameters=[config_file]
        ),
        Node(
            package='ovcrobot_control',
            executable='ovcrobot_targets',
            name='action_client_bridge',
            output = 'screen'
        )
    ])