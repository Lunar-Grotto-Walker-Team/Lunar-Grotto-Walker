cmake_minimum_required(VERSION 3.5)
project(simple_loop_closure)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(unitree_go REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)

include_directories(
  include
  thirdparty/nanoflann/include
  ${GTSAM_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(loop_noise src/loop_noise.cpp)
target_include_directories(loop_noise PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}_node src/simple_loop_closure_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node
  "pcl_conversions"
  "rcl_interfaces"
  "rclcpp"
  "rcutils"
  "sensor_msgs"
  "nav_msgs"
  "std_msgs"
  "geometry_msgs"
  "pcl_ros"
  "visualization_msgs"
  "tf2"
  "tf2_eigen"
  "unitree_go"
)
target_link_libraries(${PROJECT_NAME}_node
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  gtsam
  loop_noise
)
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Offline port check for the leg-z integrator (no ROS, no PCL/GTSAM).
add_executable(leg_z_offline tools/leg_z_offline.cpp)
target_include_directories(leg_z_offline PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)
install(TARGETS leg_z_offline DESTINATION lib/${PROJECT_NAME})

# Offline prior-map asset builder (PCL IO only, no ROS/GTSAM).
add_executable(make_prior_map tools/make_prior_map.cpp)
target_include_directories(make_prior_map PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(make_prior_map ${PCL_LIBRARIES})
install(TARGETS make_prior_map DESTINATION lib/${PROJECT_NAME})

# add_executable(test_lc_node src/test.cpp)
# ament_target_dependencies(test_lc_node
#   "pcl_conversions"
#   "rcl_interfaces"
#   "rclcpp"
#   "rcutils"
#   "sensor_msgs"
#   "nav_msgs"
#   "std_msgs"
#   "geometry_msgs"
#   "pcl_ros"
#   "visualization_msgs"
#   "tf2"
#   "tf2_eigen"
# )
# target_link_libraries(test_lc_node
#   ${Boost_LIBRARIES}
#   ${PCL_LIBRARIES}
#   gtsam
# )
# install(TARGETS
#   test_lc_node
#   DESTINATION lib/${PROJECT_NAME}
# )

install(DIRECTORY config launch rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_loop_noise test/test_loop_noise.cpp)
  target_link_libraries(test_loop_noise loop_noise)

  ament_add_gtest(test_loop_graph test/test_loop_graph.cpp)
  target_link_libraries(test_loop_graph loop_noise gtsam)

  ament_add_gtest(test_factors test/test_factors.cpp)
  target_link_libraries(test_factors gtsam)

  ament_add_gtest(test_go2_buffer test/test_go2_buffer.cpp)

  ament_add_gtest(test_loop_quality test/test_loop_quality.cpp)

  ament_add_gtest(test_ground_diagnostics test/test_ground_diagnostics.cpp)
  target_link_libraries(test_ground_diagnostics gtsam)

  ament_add_gtest(test_gravity_math test/test_gravity_math.cpp)
  target_link_libraries(test_gravity_math gtsam)

  ament_add_gtest(test_gravity_buffer test/test_gravity_buffer.cpp)

  ament_add_gtest(test_leg_kinematics test/test_leg_kinematics.cpp)

  ament_add_gtest(test_leg_z_integrator test/test_leg_z_integrator.cpp)

  ament_add_gtest(test_prior_map test/test_prior_map.cpp)
endif()

ament_package()