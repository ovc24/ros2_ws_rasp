cmake_minimum_required(VERSION 3.8)
project(ovcrasp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial_driver REQUIRED)
#find_package(serial_port REQUIRED)

add_executable(encoders_publisher src/encoders_publisher.cpp)
add_executable(encoders_joint_state_pub src/encoders_joint_state_pub.cpp)
add_executable(servo_subscriber src/servo_ena_subscriber.cpp)
add_executable(motores_subscriber src/motores_subscriber.cpp)
ament_target_dependencies(encoders_publisher rclcpp std_msgs serial_driver)
ament_target_dependencies(encoders_joint_state_pub rclcpp std_msgs sensor_msgs)
ament_target_dependencies(servo_subscriber rclcpp std_msgs serial_driver)
ament_target_dependencies(motores_subscriber rclcpp std_msgs serial_driver)
 
install(TARGETS
  encoders_publisher
  encoders_joint_state_pub
  servo_subscriber
  motores_subscriber
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
