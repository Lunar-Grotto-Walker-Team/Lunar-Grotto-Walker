cmake_minimum_required(VERSION 3.8)
project(go2_transit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# DEPENDENCIES 不能漏：goal 里用了 geometry_msgs/PoseStamped，
# 少了它生成的头文件找不到该类型。
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/TransitToPose.action"
  "action/RotateToYaw.action"
  DEPENDENCIES geometry_msgs
)

# ---- 纯逻辑库（不依赖 ROS，供 gtest 与节点链接）----
# 用 STATIC：纯逻辑没有对外 ABI 需求，静态链接省掉 install 后的 .so 查找问题
# （否则 transit_node 找不到 libgo2_transit_core.so）。
add_library(${PROJECT_NAME}_core STATIC
  src/transit_core.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)

# ---- 节点可执行 ----
add_executable(transit_node
  src/transit_main.cpp
  src/transit_node.cpp
)
target_include_directories(transit_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_features(transit_node PRIVATE cxx_std_17)
target_link_libraries(transit_node
  ${PROJECT_NAME}_core
)
ament_target_dependencies(transit_node
  rclcpp
  rclcpp_action
  action_msgs
  builtin_interfaces
  geometry_msgs
  nav2_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
# 本包自己生成的动作头：ament_target_dependencies(<target> ${PROJECT_NAME})
# 在同一包内会报 "package name was not found before"（构建期还没 install）。
# 用 rosidl_get_typesupport_target + target_link_libraries 把类型支持挂上去
# （比已废弃的 rosidl_target_interfaces 干净，不产生 deprecation warning）。
rosidl_get_typesupport_target(transit_typesupport ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(transit_node ${transit_typesupport})

install(TARGETS transit_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

# ---- BehaviorTree.CPP 插件 ----
# BT.ROS2 从 share/<pkg>/bt_plugins/ 里 dlopen，目录名必须和 launch 里
# plugins: ['go2_transit/bt_plugins'] 的第二段完全一致；目录不存在会抛异常。
add_library(go2_transit_bt SHARED
  src/transit_bt_nodes.cpp
)
target_compile_definitions(go2_transit_bt PRIVATE BT_PLUGIN_EXPORT)
target_include_directories(go2_transit_bt PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_features(go2_transit_bt PRIVATE cxx_std_17)
target_link_libraries(go2_transit_bt
  ${PROJECT_NAME}_core
  ${transit_typesupport}
)
ament_target_dependencies(go2_transit_bt
  rclcpp
  rclcpp_action
  action_msgs
  builtin_interfaces
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  behaviortree_cpp
  behaviortree_ros2
)
install(TARGETS go2_transit_bt
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins)

# 真机联调用的临时客户端：按"相对当前位置"发一次摆渡指令（将来由行为树替代）
install(PROGRAMS scripts/transit_relative.py
  DESTINATION lib/${PROJECT_NAME})

# 逐段前进探索（局部代价地图避障 + 目标识别即停）
install(PROGRAMS scripts/forward_explore.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_angle_math
    test/test_angle_math.cpp
  )
  target_include_directories(test_angle_math PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_angle_math
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_path_builder
    test/test_path_builder.cpp
  )
  target_include_directories(test_path_builder PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_path_builder
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_transit_state_machine
    test/test_transit_state_machine.cpp
  )
  target_include_directories(test_transit_state_machine PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_transit_state_machine
    ${PROJECT_NAME}_core
  )

  ament_add_gtest(test_local_frame
    test/test_local_frame.cpp
  )
  target_include_directories(test_local_frame PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_local_frame
    ${PROJECT_NAME}_core
  )

  # BT 节点级测试：直接构造节点并手动 tick，用假 transit 动作服务器注入故障。
  ament_add_gtest(test_bt_transit_node
    test/test_bt_transit_node.cpp
    src/transit_bt_nodes.cpp
  )
  target_include_directories(test_bt_transit_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_bt_transit_node
    ${PROJECT_NAME}_core
    ${transit_typesupport}
  )
  ament_target_dependencies(test_bt_transit_node
    rclcpp
    rclcpp_action
    action_msgs
    builtin_interfaces
    geometry_msgs
    nav_msgs
    std_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    behaviortree_cpp
    behaviortree_ros2
  )

  # 节点层回归测试：进程内假 nav2（FollowPath/Spin 服务器 + TF 广播）。
  ament_add_gtest(test_transit_node_integration
    test/test_transit_node_integration.cpp
    src/transit_node.cpp
  )
  target_include_directories(test_transit_node_integration PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_transit_node_integration
    ${PROJECT_NAME}_core
    ${transit_typesupport}
  )
  ament_target_dependencies(test_transit_node_integration
    rclcpp
    rclcpp_action
    action_msgs
    builtin_interfaces
    geometry_msgs
    nav2_msgs
    nav_msgs
    std_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
  )
endif()

ament_package()
