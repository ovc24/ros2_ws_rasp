from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_desc = get_package_share_directory('ovcrobot_description')
    pkg_moveit = get_package_share_directory('ovcrobot_moveit_config')

    urdf_file = os.path.join(pkg_desc, 'urdf', 'ovcrobotv3_copp.urdf')
    srdf_file = os.path.join(pkg_moveit, 'config', 'ovcrobot.srdf')
    ompl_yaml = os.path.join(pkg_moveit, 'config', 'ompl_planning.yaml')
    kinematics_file = os.path.join(pkg_moveit, "config", "kinematics.yaml")

    robot_description = ParameterValue(Command(["xacro ", urdf_file]), value_type=str)
    robot_description_semantic = ParameterValue(open(srdf_file).read(), value_type=str)
    planning_scene_monitor_parameters = {
                        "publish_planning_scene": True,
                        "publish_geometry_updates": True,
                        "publish_state_updates": True,
                        "publish_transforms_updates": True,
                        "publish_robot_description": True,
                        "publish_robot_description_semantic": True,
                        }
    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{'robot_description': open("/home/ovcpc/ros2_ws_rasp/src/ovcrobot_description/urdf/ovcrobotv3_copp.urdf").read()}],
        ),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[{"robot_description_semantic": open("/home/ovcpc/ros2_ws_rasp/src/ovcrobot_moveit_config/config/ovcrobot.srdf").read(), 
                        "robot_description": open("/home/ovcpc/ros2_ws_rasp/src/ovcrobot_description/urdf/ovcrobotv3_copp.urdf").read(),                        
                        "planning_pipeline": "ompl_interface/OMPLPlanner",
                        "request_adapters": [
                            "default_planner_request_adapters/AddTimeOptimalParameterization",
                            "default_planner_request_adapters/FixWorkspaceBounds",
                            "default_planner_request_adapters/FixStartStateBounds",
                            "default_planner_request_adapters/FixStartStateCollision",
                            "default_planner_request_adapters/FixStartStatePathConstraints"
                        ],
                        "start_state_max_bounds_error": 0.1,
                        "planning_scene_monitor_parameters": planning_scene_monitor_parameters
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            #arguments=['-d', open("/home/ovcpc/ros2_ws_rasp/src/ovcrobot_moveit_config/config/launch.rviz").read()]
        )
    ])
