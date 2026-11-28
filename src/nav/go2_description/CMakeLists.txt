cmake_minimum_required(VERSION 3.8)
project(go2_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# dae/ 与 meshes/ 共约 50 MB（Go2 官方模型），不入库，用
# tools/fetch_go2_meshes.sh 拉取。缺目录时仍然能构建，只是 RViz / robot_state_publisher
# 加载不出模型；拉取后重新 build 即可。
set(_assets launch urdf)
foreach(_d dae meshes)
  if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${_d})
    list(APPEND _assets ${_d})
  else()
    message(WARNING "go2_description: 缺少 ${_d}/，模型不会安装。先运行 tools/fetch_go2_meshes.sh 再 colcon build")
  endif()
endforeach()

install(
  DIRECTORY ${_assets}
  DESTINATION share/${PROJECT_NAME}
)

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
