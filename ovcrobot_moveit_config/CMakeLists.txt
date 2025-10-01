cmake_minimum_required(VERSION 3.22)
project(ovcrobot_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_kinematics REQUIRED)
find_package(moveit_planners_ompl REQUIRED)
find_package(moveit_ros_visualization REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_visual_tools REQUIRED)
ament_package()

# if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
#   install(
#     DIRECTORY launch
#     DESTINATION share/${PROJECT_NAME}
#     PATTERN "setup_assistant.launch" EXCLUDE)
# endif()

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
