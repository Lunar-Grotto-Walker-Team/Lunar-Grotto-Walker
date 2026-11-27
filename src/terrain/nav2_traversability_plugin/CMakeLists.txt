cmake_minimum_required(VERSION 3.8)
project(nav2_traversability_plugin)

set(lib_name ${PROJECT_NAME}_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

set(dep_pkgs
    rclcpp
    nav2_costmap_2d
    pluginlib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(${lib_name} SHARED src/traversability_layer.cpp)

include_directories(include)

install(TARGETS ${lib_name}
  DESTINATION lib)

if(BUILD_TESTING)
  ament_add_gtest(test_terrain_cost test/test_terrain_cost.cpp)
  target_include_directories(test_terrain_cost PRIVATE include)
  ament_target_dependencies(test_terrain_cost nav2_costmap_2d)

  ament_add_gtest(test_terrain_grid_alignment test/test_terrain_grid_alignment.cpp)
  target_include_directories(test_terrain_grid_alignment PRIVATE include)
  ament_target_dependencies(test_terrain_grid_alignment nav2_costmap_2d)
endif()

pluginlib_export_plugin_description_file(nav2_costmap_2d traversability_layer.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs}) 

ament_package()
