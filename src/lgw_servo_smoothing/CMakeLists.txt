cmake_minimum_required(VERSION 3.10)
project(lgw_servo_smoothing)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)

# Shared library so pluginlib can dlopen it by class name.
add_library(lgw_servo_smoothing SHARED src/no_op_smoothing.cpp)
target_link_libraries(lgw_servo_smoothing
  moveit_core::moveit_smoothing_base
)
# ament_target_dependencies (not just target_link_libraries) is what puts the
# pluginlib/rclcpp include dirs on the compile line here.
ament_target_dependencies(lgw_servo_smoothing rclcpp pluginlib)

# Register this package as a provider of moveit_core plugins. The first
# argument is the package that OWNS the base class (moveit_core), which is the
# key ament uses (share/ament_index/resource_index/moveit_core__pluginlib__plugin);
# the XML itself lives in this package.
pluginlib_export_plugin_description_file(
  moveit_core no_op_smoothing_plugin.xml)

install(
  TARGETS lgw_servo_smoothing
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  FILES no_op_smoothing_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(pluginlib rclcpp moveit_core)
ament_package()
