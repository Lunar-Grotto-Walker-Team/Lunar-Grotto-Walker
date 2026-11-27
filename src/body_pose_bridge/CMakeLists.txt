cmake_minimum_required(VERSION 3.16)
project(body_pose_bridge VERSION 0.1.0 LANGUAGES C CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_go REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ---- SetBodyPose service interface ----
set(SRV_FILES
    "srv/SetBodyPose.srv"
  )

rosidl_generate_interfaces(${PROJECT_NAME}
  ${SRV_FILES}
  DEPENDENCIES builtin_interfaces
)

# ---- Bridge node executable ----
add_executable(body_pose_bridge_node
  src/body_pose_bridge_node.cpp
)
target_include_directories(body_pose_bridge_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(body_pose_bridge_node
  rclcpp
  unitree_api
  unitree_go
)
# Self-link the generated C++ typesupport target so the executable can
# resolve the SetBodyPose service type support at link time. Pattern
# lifted from lgw_bt_planner/CMakeLists.txt:284-288.
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(body_pose_bridge_node
  "${cpp_typesupport_target}")

install(TARGETS body_pose_bridge_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_export_dependencies(rclcpp unitree_api unitree_go)

ament_package()