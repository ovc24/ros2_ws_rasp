cmake_minimum_required(VERSION 3.8)
project(ovcrobot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ovcrobot_interfaces REQUIRED)
find_package(moveit_msgs REQUIRED)

add_executable(ovcrobot_pid src/ovcrobot_PID_Ctrl.cpp)
add_executable(ovcrobot_targets src/ovcrobot_targets.cpp)

ament_target_dependencies(ovcrobot_pid ovcrobot_interfaces rclcpp std_msgs rclcpp_action)
ament_target_dependencies(ovcrobot_targets ovcrobot_interfaces rclcpp std_msgs geometry_msgs rclcpp_action moveit_msgs)

install(TARGETS
  ovcrobot_pid
  ovcrobot_targets
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
