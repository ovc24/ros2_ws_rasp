from launch import LaunchDescription
#from launch.substitution import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('ovcrasp')
    #config_file = os.path.join(package_dir, 'config', 'pid_parameters.yaml')
    return LaunchDescription([
        Node(
            package='ovcrasp',
            executable='encoders_publisher',
            name='encoders_publisher',
            #output='screen',
        ),
        Node(
            package='ovcrasp',
            executable='encoders_joint_state_pub',
            name='encoders_joint_state_pub',
            #output = 'screen'
        ),
        Node(
            package='ovcrasp',
            executable='motores_subscriber',
            name='motores_subscriber',
            output = 'screen'
        ),
        Node(
            package='ovcrasp',
            executable='servo_subscriber',
            name='servo_subscriber',
            #output = 'screen'
        )
    ])