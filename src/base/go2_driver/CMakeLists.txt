cmake_minimum_required(VERSION 3.8)
project(go2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(unitree_go REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)




add_executable(driver src/driver.cpp)
add_executable(footprint_to_link src/footprint_to_link.cpp)
add_executable(lowstate_to_imu src/lowstate_to_imu.cpp)
# 2026-10-23: 动态 map -> odom 修正节点（让 nav2 全局位姿与 FAST-LIO 同源）
add_executable(map_odom_correction src/map_odom_correction.cpp)

target_include_directories(driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  driver
  "rclcpp"
  "unitree_go"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  "geometry_msgs"
  "sensor_msgs"
)

ament_target_dependencies(
  footprint_to_link
  "rclcpp"
  "unitree_go"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  "geometry_msgs"
  "sensor_msgs"
)

ament_target_dependencies(
  lowstate_to_imu
  "rclcpp"
  "unitree_go"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  "geometry_msgs"
  "sensor_msgs"
)

# 只用到 tf2 / tf2_ros / geometry_msgs / nav_msgs / rclcpp / std_srvs + Eigen，
# 无需 unitree_go
ament_target_dependencies(
  map_odom_correction
  "rclcpp"
  "tf2"
  "tf2_ros"
  "geometry_msgs"
  "nav_msgs"
  "std_srvs"
)
target_include_directories(map_odom_correction PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(map_odom_correction Eigen3::Eigen)

install(TARGETS
  driver
  footprint_to_link
  lowstate_to_imu
  map_odom_correction
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch params rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
