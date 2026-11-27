cmake_minimum_required(VERSION 3.16)
project(servo_state_probe)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)

add_executable(servo_state_probe src/servo_state_probe.cpp)
target_link_libraries(servo_state_probe
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  moveit_core::moveit_robot_model
  moveit_core::moveit_robot_state
  moveit_ros_planning::moveit_robot_model_loader
  # CurrentStateMonitor moved into moveit_ros_planning in MoveIt 2.5.x;
  # verified with `nm -DC libmoveit_planning_scene_monitor.so | grep CurrentStateMonitor`.
  moveit_ros_planning::moveit_planning_scene_monitor
)

install(TARGETS servo_state_probe
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
