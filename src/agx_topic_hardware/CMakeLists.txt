cmake_minimum_required(VERSION 3.16)
project(agx_topic_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(agx_topic_hardware SHARED
  src/agx_topic_system.cpp
)
target_include_directories(agx_topic_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(agx_topic_hardware
  hardware_interface
  pluginlib
  rclcpp
  sensor_msgs
)
pluginlib_export_plugin_description_file(
  hardware_interface agx_topic_hardware_plugin.xml)

install(
  TARGETS agx_topic_hardware
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(agx_topic_hardware)
ament_package()
