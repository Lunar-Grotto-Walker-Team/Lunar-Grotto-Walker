cmake_minimum_required(VERSION 3.16)
project(lgw_vui_bridge VERSION 0.1.0 LANGUAGES C CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ---- SetVuiBrightness service interface ----
set(SRV_FILES
    "srv/SetVuiBrightness.srv"
  )

rosidl_generate_interfaces(${PROJECT_NAME}
  ${SRV_FILES}
  DEPENDENCIES builtin_interfaces
)

# === unitree_sdk2 ===
# The SDK is linked ONLY into the helper subprocess (go2_vui_set_brightness_helper).
# vui_bridge_node has no SDK link — it popen's the helper for every service call.
# This is the architectural fix for the in-process double-participant SIGSEGV
# that originally bit SetLedBrightnessNode: rclcpp's rmw_cyclonedds_cpp (which
# statically carries its own libddscxx 0.10.5) and the SDK's vendored libddscxx
# 0.10.2 can NEVER share an address space here, because they live in separate
# OS processes. See srv/SetVuiBrightness.srv header for the full rationale.
# unitree_sdk2 不随本仓库/vendor 分发：构建前需要宿主机 foxy 工作区里编译好的源码树
# （产出 lib/<arch>/libunitree_sdk2.a 与 thirdparty/lib/<arch>/libddsc*.so）。
# 位置优先取环境变量 UNITREE_SDK2_DIR，否则回退到本机约定路径。
# 详见 docs/dependencies.md §3.3。
if(DEFINED ENV{UNITREE_SDK2_DIR})
  set(_unitree_sdk2_default "$ENV{UNITREE_SDK2_DIR}")
else()
  set(_unitree_sdk2_default "/my_ws/foxy/unitree_sdk2")
endif()
set(UNITREE_SDK2_SOURCE_DIR
  "${_unitree_sdk2_default}"
  CACHE PATH
    "Path to a built unitree_sdk2 source tree. Override with -DUNITREE_SDK2_SOURCE_DIR=<path> or env UNITREE_SDK2_DIR."
)
if(NOT EXISTS "${UNITREE_SDK2_SOURCE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "unitree_sdk2 source not found at ${UNITREE_SDK2_SOURCE_DIR}. "
    "Build upstream unitree_sdk2 first, then set UNITREE_SDK2_DIR=<path> "
    "or pass -DUNITREE_SDK2_SOURCE_DIR=<path>. See docs/dependencies.md §3.3.")
endif()
find_library(UNITREE_SDK2_LIB
  NAMES unitree_sdk2
  PATHS "${UNITREE_SDK2_SOURCE_DIR}/lib/${CMAKE_SYSTEM_PROCESSOR}"
  NO_DEFAULT_PATH
  REQUIRED)
find_library(UNITREE_SDK2_DDSC_LIB
  NAMES ddsc
  PATHS "${UNITREE_SDK2_SOURCE_DIR}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}"
  NO_DEFAULT_PATH
  REQUIRED)
find_library(UNITREE_SDK2_DDSCXX_LIB
  NAMES ddscxx
  PATHS "${UNITREE_SDK2_SOURCE_DIR}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}"
  NO_DEFAULT_PATH
  REQUIRED)
find_package(Threads REQUIRED)

# ---- One-shot SDK helper subprocess ----
# CLI: go2_vui_set_brightness_helper <network_interface> <level>
#      exit 0 = firmware acked, 1 = SDK error, 2 = CLI usage.
# This binary owns the entire SDK lifecycle: ChannelFactory::Init, VuiClient,
# the SDK's vendored libddscxx.so.0, the DomainParticipant on eth10. It runs
# in its own short-lived OS process, so nothing it does can leak into
# vui_bridge_node's rclcpp state.
add_executable(go2_vui_set_brightness_helper
  src/go2_vui_set_brightness_helper.cpp
)
target_include_directories(go2_vui_set_brightness_helper PRIVATE
  ${UNITREE_SDK2_SOURCE_DIR}/include
  ${UNITREE_SDK2_SOURCE_DIR}/thirdparty/include
  ${UNITREE_SDK2_SOURCE_DIR}/thirdparty/include/ddscxx
)
target_link_libraries(go2_vui_set_brightness_helper
  ${UNITREE_SDK2_LIB}
  ${UNITREE_SDK2_DDSC_LIB}
  ${UNITREE_SDK2_DDSCXX_LIB}
  Threads::Threads
)
# Force DT_RPATH (not DT_RUNPATH) on the helper so its SDK vendored
# cyclonedds wins over the parent vui_bridge_node's LD_LIBRARY_PATH,
# which (after l4t_setup.sh) contains /opt/ros/humble/lib/aarch64-linux-gnu
# pointing at libddsc.so.0.10.5. With RUNPATH (cmake's default on
# modern toolchains), LD_LIBRARY_PATH outranks it and the helper ends
# up loading 0.10.5 libddsc + 0.10.2 libddscxx → ABI mismatch inside
# the helper too. DT_RPATH reverses that priority. See
# ld.so(8) for the ordering rules.
target_link_options(go2_vui_set_brightness_helper PRIVATE
  -Wl,--disable-new-dtags
)
# Helper loads the SDK vendored cyclonedds exclusively — its own RPATH
# keeps the loader from accidentally resolving to /opt/ros/humble's
# libddsc.so.0.10.5 (which would mismatch the SDK static archive's 0.10.2
# link assumptions and cause helper-side crashes too).
set_target_properties(go2_vui_set_brightness_helper PROPERTIES
  BUILD_RPATH "${UNITREE_SDK2_SOURCE_DIR}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}"
  INSTALL_RPATH "${UNITREE_SDK2_SOURCE_DIR}/thirdparty/lib/${CMAKE_SYSTEM_PROCESSOR}"
)

# ---- Bridge node executable (rclcpp only, NO SDK link) ----
# This is the rclcpp service front-end. It popen's the helper for every
# SetVuiBrightness request. Its loader path is the humble default — it
# never sees libddscxx.so.0 (rmw_cyclonedds_cpp's ddscxx is statically
# linked inside that .so), so there is nothing for the SDK vendored
# libddscxx 0.10.2 to collide with.
add_executable(vui_bridge_node
  src/vui_bridge_node.cpp
)
ament_target_dependencies(vui_bridge_node
  rclcpp
  ament_index_cpp
)
target_link_libraries(vui_bridge_node
  # Self-link generated C++ typesupport (vui_bridge_node uses rclcpp service)
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)
# No BUILD_RPATH / INSTALL_RPATH override: vui_bridge_node uses ros humble's
# loader path verbatim, which is exactly what rclcpp was linked against.

install(TARGETS vui_bridge_node go2_vui_set_brightness_helper
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
# Keep headers + launch on disk for downstream packages & ros2 launch.
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_export_dependencies(rclcpp ament_index_cpp)

ament_package()
