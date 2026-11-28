cmake_minimum_required(VERSION 3.16)
project(go2_front_camera_driver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_CXX_STANDARD_REQUIRED)
  # (no-op; just ensures standard is required)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)

add_executable(go2_front_camera_driver_node src/go2_front_camera_driver_node.cpp)
ament_target_dependencies(go2_front_camera_driver_node
  rclcpp
  sensor_msgs
  cv_bridge
  camera_info_manager
  image_transport
)
target_link_libraries(go2_front_camera_driver_node ${OpenCV_LIBRARIES})

install(TARGETS
  go2_front_camera_driver_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
