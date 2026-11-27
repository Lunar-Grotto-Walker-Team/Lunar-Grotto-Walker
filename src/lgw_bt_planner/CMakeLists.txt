cmake_minimum_required(VERSION 3.16)
project(lgw_bt_planner VERSION 0.1.0 LANGUAGES C CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)  # servo_node: Servo's output topic
find_package(builtin_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(agx_arm_msgs REQUIRED)

find_package(ament_index_cpp REQUIRED)
find_package(yoloe_interfaces REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_go REQUIRED)
find_package(body_pose_bridge REQUIRED)
find_package(lgw_vui_bridge REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake_gtest REQUIRED)  # for the gtest target below
# Pinned-posture lookat family (plan 2026-10-31): KDL chain + FK.
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
# Servo node parameter schema (Task 4 of the 2026-11-07 node-split plan).
find_package(generate_parameter_library REQUIRED)

# === unitree_sdk2 (REMOVED 2026-10-15) ===
# SetLedBrightnessNode used to call libunitree_sdk2.a's
# VuiClient::SetBrightness directly, which required linking the SDK
# static archive + its INTERFACE deps libddsc.so / libddscxx.so into
# the SHARED plugin lib. That in-process linkage triggered a cyclone
# DDS double-participant SIGSEGV at first plugin instantiation
# (rmw_cyclonedds_cpp's ddscxx vs libunitree_sdk2.a's ddscxx, both
# writing into libddsc.so.0 in the same address space — see
# feedback_unitree_sdk_cyclonedds_process_isolation.md).
#
# v3 (2026-10-15): SetLedBrightnessNode became a
# BT::RosServiceNode<lgw_vui_bridge::srv::SetVuiBrightness>. The SDK
# lifecycle lives in a one-shot helper subprocess
# (go2_vui_set_brightness_helper) spawned by vui_bridge_node, which
# is the only place that links the SDK now. lgw_bt_planner no longer
# needs the SDK plumbing, so the entire UNITREE_SDK2_SOURCE_DIR /
# find_library(UNITREE_SDK2_*) / RPATH block above is removed.
# If a future BT node wants to call SDK again, prefer the
# RosServiceNode + helper subprocess pattern over re-adding this.

set(NODE_SOURCES
  # Per-tick control pipeline (Task 6): refusals, feed-forward memory and the
  # control law, with no rclcpp / TF / clock in sight.
  src/servo/tracking_loop.cpp
  # HOLD-recovery decision (Task 7): the worker thread stays in the node.
  src/servo/recovery_orchestrator.cpp
  # /lgw/servo/state message content (Task 8, step 4b).
  src/servo/servo_state_reporter.cpp
  # release / pause / resume sequencing (Task 8, step 3).
  src/servo/servo_session.cpp
  # ServoNode's own implementation: construction, parameters, calibration and
  # the subscription callbacks (2026-11-12: moved out of the header).
  src/servo/servo_node.cpp
  # The 100 Hz control tick and the Servo start path.
  src/servo/servo_tick.cpp
  # HOLD entry/exit, recovery glue and the recovery worker thread.
  src/servo/servo_recovery.cpp
  # Takeover sequencing, MoveGroup pre-position glue and the service handlers.
  src/servo/servo_takeover.cpp
  src/evaluate_detection.cpp
  src/named_target_loader.cpp
  # xacro <xacro:if>/<xacro:unless> branch filter for the SRDF named-target
  # table (2026-11-17): keeps per-arm-type states from clobbering each other.
  src/xacro_condition_filter.cpp
  src/pose_string_parser.cpp
  src/set_pose_stamped_node.cpp
  src/move_arm_to_pose_action_node.cpp
  src/move_arm_to_named_action_node.cpp
  # ControlGripper BT leaf (2026-11-16): 直接发 JointTrajectory 给
  # /gripper_controller, 支持开合量 + 用时(速度) + 力判据。
  src/control_gripper_node.cpp
  # Track-collect validation leaves (2026-11-17): UnifiedTrackArray consumers
  # and the tracker/servo service shells.
  src/target_track_nodes.cpp
  src/track_service_nodes.cpp
  # 基地圆心 (2026-11-18): RecordBasePose / IsWithinBaseRadius, 纯 TF+黑板。
  src/base_pose_nodes.cpp
  # Grasp/pre-grasp tcp poses from the selected track (arm base frame,
  # look-at orientation, 100 mm optical-axis standoff).
  src/compute_grasp_poses_from_track.cpp
  # Sampled variant (2026-11-20): pitch grid x yaw perturbations, first
  # candidate whose two poses pass /compute_ik wins. v2 tree only.
  src/compute_grasp_poses_sampled.cpp
  # IsHead/ArmDetectionConfirmed BT leaf nodes (added 2026-09-24) —
  # RosTopicSubNode<ObservationResult> subclasses; logic delegates to
  # evaluate_detection(). Not yet registered with BT.ROS2 (Task 3).
  src/is_detection_confirmed_node.cpp
  # WaitForUserInput BT leaf node — snapshot-style RosTopicSubNode<Bool>;
  # plugin-registered in bt_plugin_registration.cpp. Used by
  # bt_trees/user_gated_auto_pick_and_place.xml (added 2026-10-01).
  src/wait_for_user_input_bt_node.cpp
  # BT.ROS2 plugin entry point: BT_REGISTER_ROS_NODES(...) block. Compiled
  # into the SHARED library so the registration symbol BT_RegisterRosNodeFromPlugin
  # is dlopen-able by TreeExecutionServer's RegisterPlugins walker (bt_utils.cpp)
  # when loading <pkg_share>/bt_plugins/*.so.
  src/bt_plugin_registration.cpp
  # ComputeLookatPose BT node (added 2026-10-01) — pure math + JSON
  # parser lives here. The actual ComputeLookatPoseNode class is added
  # in Task 6; Task 2 ships the parse_calibration_json pure function
  # used by the node constructor's load_calibration_() helper.
  src/compute_lookat_pose_node.cpp
  # Servo takeover (2026-10-26) — pure 100Hz P control law: tcp_link pose
  # error -> clamped/scaled twist -> first-order low-pass, with a strict
  # 2deg/1cm deadband that preserves the filter state. Consumed by the servo
  # node (Task 7); no rclcpp in the module itself.
  src/servo_control_law.cpp
  # Servo takeover (2026-10-26) — pure §7.1 state machine (takeover /
  # unpause / hold / release / fault) + tracker stream watchdog on
  # UnifiedTrackArray arrival time. Consumed by the servo node (Task 7);
  # no rclcpp in the module itself.
  src/servo_state_machine.cpp
  # VerifyHeadArmConsistency + ComputeGraspPose BT nodes (added
  # 2026-10-07) — companion to ComputeLookatPose for
  # bt_trees/head_to_arm_verify_grasp.xml. Both reuse
  # parse_calibration_json + build_lookat_orientation from
  # compute_lookat_pose_node.hpp. Each loads both calibration JSONs at
  # construction time (same idempotent default-fallback pattern).
  src/verify_head_arm_consistency.cpp
  src/compute_grasp_pose.cpp
  # ComputeHeadTargetWorld / ComputeArmTargetWorld BT nodes (added
  # 2026-10-08) — pure transform helpers used by
  # bt_trees/head_arm_independent_lookat.xml. Each loads one calibration
  # JSON (head or arm) and emits a world-frame Point to the blackboard.
  src/compute_head_target_world.cpp
  src/compute_arm_target_world.cpp
  # MakePoseFromPointAndReference BT node (added 2026-10-08) — trivial
  # pose composition: position=point, orientation=reference_pose.orientation.
  # Used in head_arm_independent_lookat.xml Phase B/C to preserve Phase A's
  # lookat orientation while swapping in head/arm target positions.
  src/make_pose_from_point_and_reference.cpp
  # PoseSelector pure (no-rclcpp, no-BT) geom + scoring module
  # (added 2026-10-09 per
  # docs/superpowers/specs/2026-10-09-select-arm-observation-pose-spec.md
  # §4.3). Linked into the SHARED plugin library so both BT thin shell
  # and the standalone service_node can use it. See tests/
  # test_select_arm_observation_pose_geom.cpp for unit coverage.
  src/select_arm_observation_pose_geom.cpp
  # SelectArmObservationPoseBTNode — BT thin shell (added 2026-10-09 per
  # spec §4.8). Plugin-registered in bt_plugin_registration.cpp.
  src/select_arm_observation_pose_bt.cpp
  # Go2 sport API BT nodes (added 2026-10-06) — 3 independent
  # StatefulActionNode subclasses that publish /api/sport/request and
  # subscribe lf/sportmodestate. See
  # docs/superpowers/specs/2026-10-06-go2-posture-nodes-design.md §3.
  src/stand_up_node.cpp
  src/balance_stand_node.cpp
  src/stand_down_node.cpp
  # Go2 BodyHeight BT node (added 2026-10-06) — publishes api_id=1013
  # with parameter {"data": offset}. Companion to the 3 posture nodes.
  src/set_body_height_node.cpp
  # Go2 BodyPose BT node (added 2026-10-10, simplified 2026-10-13 v7)
  # — bundles api_id=1007 (Euler rpy) + 1002 (BalanceStand trigger)
  # into one node, because the Go2 state machine latches rpy at
  # BalanceStand time (Euler after BalanceStand is ignored — see
  # go2_sport_client.cpp:63-67 "controlled by dBodyHeight + rpy";
  # the SDK comment mentions body_h but Go2 firmware does not honor
  # it, hence v7 removal of BodyHeight from the 3-call). Replaces
  # the earlier SetEulerPoseNode that exposed the same bug at the
  # BT-XML layer.
  src/set_body_pose_node.cpp
  # Go2 SwitchGait BT node (added 2026-10-13) — publishes Go2 gait
  # switch commands to /api/sport/request. Supports 4 trigger gaits
  # (1061 StaticWalk / 1062 TrotRun / 1063 EconomicGait / 2045
  # FreeWalk) + 7 bool flag gaits (2046 FreeBound / 2047 FreeJump /
  # 2048 FreeAvoid / 2049 ClassicWalk / 2050 WalkUpright / 2051
  # CrossStep / 2054 AutoRecoverSet). Replaces the deprecated
  # SwitchGait(1011) API (deleted in SDK v0.2.0 2025-07-29). XML
  # author must invoke BalanceStand first.
  src/switch_gait_node.cpp
  # Go2 EDU VUI SetBrightness BT node (added 2026-10-13) — publishes
  # api_id=1005 to rt/api/vui/request (NOT /api/sport/request — VUI
  # is a separate service; topic 命名跟 C++ unitree_sdk2
  # channel_namer.hpp ROBOT_SDK_CHANNEL_PREFIX="rt/api/" 一致,
  # sport 走 "/api/sport/request" 是 unitree_ros2 官方 wrapper 自定
  # 命名,VUI 没官方 wrapper 必须跟 SDK 同名)。Naming caveat: this
  # is 屏幕亮度 not RGB LED; Go2 SDK 高层 no LED API. Level 0-10
  # per go2_vui_client.cpp:32.
  src/set_led_brightness_node.cpp
  # point_lio /start_mapping trigger (2026-11-20).  Wraps the std_srvs/Empty
  # service that point_lio's wait_for_start gate listens on: the tree raises
  # it after the operator has placed the dog at the base origin, so the map
  # origin is that pose instead of wherever warm-up happened.
  src/start_mapping_node.cpp
)

# Light static library: pure logic with no rclcpp. Kept OUT of the node
# sources so unit tests link a ~200 KB library instead of the 5 MB BT plugin
# library — that turns the edit-test loop from ~30 s into a few seconds.
# STATIC (not SHARED) because lgw_bt_planner_nodes is dlopen-ed by BT.ROS2:
# one fewer .so to resolve at runtime.
# --- library split (2026-11-12) -------------------------------------------
# NODE_SOURCES used to be one 37-file SHARED library that 17 tests all linked,
# so any change relinked all of them (measured: 21 links ~27 s per build).
# Two subsystems move into their own STATIC libraries below; STATIC on purpose,
# because lgw_bt_planner_nodes is dlopen-ed by BT.ROS2 and a STATIC dependency
# is compiled into it -- no extra .so for the loader to resolve at runtime.
#
# Dependency direction (verified by an include scan, no cycles):
#   lgw_bt_planner_nodes -> lgw_geometry
#   lgw_servo            -> lgw_geometry, lgw_posture, lgw_preposition
#   lgw_geometry         -> (only ROS/message libs)
# No BT source includes a servo header and no servo source includes a BT one,
# which is what makes the two halves separable.
set(GEOMETRY_SOURCES
  src/calibration_json.cpp
  src/evaluate_detection.cpp
  src/pose_string_parser.cpp
  src/compute_lookat_pose_node.cpp
  src/verify_head_arm_consistency.cpp
  src/compute_grasp_pose.cpp
  src/compute_head_target_world.cpp
  src/compute_arm_target_world.cpp
  src/make_pose_from_point_and_reference.cpp
  src/select_arm_observation_pose_geom.cpp
)
set(SERVO_SOURCES
  src/servo/servo_node.cpp
  src/servo/servo_tick.cpp
  src/servo/servo_recovery.cpp
  src/servo/servo_takeover.cpp
  src/servo/servo_session.cpp
  src/servo/servo_state_reporter.cpp
  src/servo/tracking_loop.cpp
  src/servo/recovery_orchestrator.cpp
  src/servo_control_law.cpp
  src/servo_state_machine.cpp
  src/servo/scan_trajectory.cpp
  src/servo/scan_roll_lock.cpp
  src/servo/servo_scan_worker.cpp
)
list(REMOVE_ITEM NODE_SOURCES ${GEOMETRY_SOURCES} ${SERVO_SOURCES})

add_library(lgw_geometry STATIC ${GEOMETRY_SOURCES})
set_target_properties(lgw_geometry PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(lgw_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/lgw_bt_planner>
)
ament_target_dependencies(lgw_geometry
  rclcpp geometry_msgs sensor_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs
  yoloe_interfaces behaviortree_cpp
)

# Servo subsystem. Links the look-at solvers (lgw_posture) and the MoveGroup
# client (lgw_preposition), and nothing BT-related -- so a servo edit relinks
# the servo tests and the servo_node executable, but never the BT plugin.
add_library(lgw_servo STATIC ${SERVO_SOURCES})
set_target_properties(lgw_servo PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(lgw_servo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/lgw_bt_planner>
)
ament_target_dependencies(lgw_servo
  rclcpp rclcpp_action geometry_msgs sensor_msgs std_msgs std_srvs action_msgs
  trajectory_msgs builtin_interfaces tf2 tf2_ros yoloe_interfaces
  ament_index_cpp moveit_msgs
)
# Plain signature: ament_target_dependencies above already used it, and CMake
# refuses to mix the two on one target. Plain is equivalent to PUBLIC here,
# which is what a static library's consumers need.
target_link_libraries(lgw_servo
  ${PROJECT_NAME}__rosidl_generator_cpp
  lgw_posture lgw_preposition lgw_geometry servo_node_params
)

add_library(lgw_posture STATIC
  src/lookat_solver.cpp
  src/lookat_posture_family.cpp
  src/lookat_qp_solver.cpp
  # calibration_json.cpp moved to lgw_geometry (2026-11-12): parsing the
  # hand-eye JSON is geometry, and compute_lookat_pose_node needs it there.
  src/preposition_decision.cpp
  # HOLD recovery criteria (spec 2026-11-03 §4). Pure: reason whitelist,
  # budgets, geometry age, follower_lag never moving.
  src/hold_recovery_decision.cpp
  # Grasp / pre-grasp SolveTask implementations (2026-11-19): the pose task
  # fed to the generic solve_task() engine. Pure math (FK via a callback).
  src/grasp_tasks.cpp
)
set_target_properties(lgw_posture PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(lgw_posture PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/lgw_bt_planner>
)
# qpmad is used only inside lookat_qp_solver.cpp, so it stays PRIVATE (an
# exported PUBLIC path would have to be installed as well).
target_include_directories(lgw_posture PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/qpmad/include
)
ament_target_dependencies(lgw_posture tf2 kdl_parser orocos_kdl)

# MoveGroup client used by the takeover pre-position. Separate from lgw_posture
# because it pulls rclcpp/rclcpp_action; keeping it out of the pure library
# keeps the pure tests' compile time down.
add_library(lgw_preposition STATIC src/move_group_prepositioner.cpp)
set_target_properties(lgw_preposition PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(lgw_preposition PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/lgw_bt_planner>
)
ament_target_dependencies(lgw_preposition rclcpp rclcpp_action moveit_msgs)

# The servo node's parameter schema. The generated header lands in
# <build>/include/<pkg>/servo_node_params.hpp and owns every name, type and
# default; declare_parameter calls in the node are gone.
generate_parameter_library(servo_node_params
  config/servo_node_parameters.yaml
)
# The header-only target has to join the same export set as the library that
# links it, or `install(EXPORT ...)` refuses to resolve the dependency.
install(TARGETS servo_node_params EXPORT lgw_bt_planner_nodes)

add_library(lgw_bt_planner_nodes SHARED ${NODE_SOURCES})
# BT_PLUGIN_EXPORT expands BTCPP_EXPORT (BT.CPP's visibility decorator) so
# the BT_RegisterRosNodeFromPlugin symbol is exposed for BT.ROS2's
# BT::SharedLibrary dlopen (see plugins.hpp:75 RegisterRosNode).
target_compile_definitions(lgw_bt_planner_nodes PRIVATE BT_PLUGIN_EXPORT)
target_include_directories(lgw_bt_planner_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/lgw_bt_planner>
)
ament_target_dependencies(lgw_bt_planner_nodes
  rclcpp
  rclcpp_action
  behaviortree_cpp
  behaviortree_ros2
  btcpp_ros2_interfaces
  moveit_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  action_msgs
  builtin_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  agx_arm_msgs
  yoloe_interfaces
  unitree_api
  unitree_go
  body_pose_bridge
  lgw_vui_bridge
  # Pinned-posture lookat family: parses the launched robot_description into
  # a KDL chain and runs FK on it.
  kdl_parser
  orocos_kdl

)
# Self-link rosidl-generated C++ headers for lgw_bt_planner (added
# 2026-10-09). rosidl_generate_interfaces (called later, near the
# bottom) creates the INTERFACE target ${PROJECT_NAME}__rosidl_generator_cpp
# which carries the include path to build/<pkg>/rosidl_generator_cpp/.
# The lgw_bt_planner_nodes SHARED lib needs it because
# select_arm_observation_pose_bt.cpp includes
# "lgw_bt_planner/srv/select_arm_observation_pose.hpp".
# Must use plain signature to match ament_target_dependencies' call style.
target_link_libraries(lgw_bt_planner_nodes
  ${PROJECT_NAME}__rosidl_generator_cpp
  lgw_geometry
  lgw_posture
  lgw_preposition
  servo_node_params
)
# Precompiled header: TRIED 2026-11-12 and reverted. It worked (the compiler
# got `-include .../cmake_pch.hxx`, and a 349 MB .gch was produced) but a
# header-triggered rebuild was 146 s against a 150 s baseline -- no gain. A
# PCH that size has to be read by every TU, and on this eMMC that eats the
# parsing time it saves. Re-add only if clean builds (42 TUs from scratch)
# become the common case, where the saving is not offset by incremental I/O.
# No SDK RPATH / link — SetLedBrightnessNode is now a RosServiceNode
# over lgw_vui_bridge::srv::SetVuiBrightness; the SDK lifecycle lives
# in the vui_bridge helper subprocess, not in this SHARED plugin lib.

# bt_runner executable (TreeExecutionServer host). It must NOT link against
# lgw_bt_planner_nodes: that library is a BT.ROS2 plugin dlopen-ed at
# runtime from share/<pkg>/bt_plugins/, which is not on the loader path.
# named_target_loader.cpp is compiled in directly because bt_runner itself
# reads the SRDF table before any tree exists.
add_executable(bt_runner
  src/bt_runner_node.cpp
  src/named_target_loader.cpp
  src/xacro_condition_filter.cpp
)
target_include_directories(bt_runner PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(bt_runner
  rclcpp
  rclcpp_action
  behaviortree_cpp
  behaviortree_ros2
  btcpp_ros2_interfaces
  moveit_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  builtin_interfaces
  tf2_ros
  tf2_geometry_msgs

  ament_index_cpp
)

# BT leaf node plugin shared library — installed where BT.ROS2's
# TreeExecutionServer `plugins` ROS param walker expects to find it
# (share/<pkg>/bt_plugins/). The walker (BT::RegisterPlugins in
# bt_utils.cpp) globs *.so here and dlopens each via BT::SharedLibrary.
install(TARGETS lgw_bt_planner_nodes
  EXPORT lgw_bt_planner_nodes
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
)

# The two static helpers are linked into lgw_bt_planner_nodes. CMake refuses to
# export a target whose static dependencies are not exported too ("requires
# target ... that is not in any export set"), so register them in the same set.
install(TARGETS lgw_geometry lgw_servo
  EXPORT lgw_bt_planner_nodes
  ARCHIVE DESTINATION lib
)
install(TARGETS lgw_posture lgw_preposition
  EXPORT lgw_bt_planner_nodes
  ARCHIVE DESTINATION lib
)

# Plain executables (no plugin discovery needed).
install(TARGETS bt_runner
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Standalone rclcpp::Node — provides /select_arm_observation_pose service
# (added 2026-10-09 per spec §4.4). Links the SHARED plugin library to
# reuse the PoseSelector class; rclcpp::spin on its own. The BT thin shell
# (in the plugin lib) dials this service via BT::RosServiceNode.
add_executable(select_arm_observation_pose_service_node
  src/select_arm_observation_pose_service_node.cpp
)
target_include_directories(select_arm_observation_pose_service_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(select_arm_observation_pose_service_node
  rclcpp
  moveit_msgs
  sensor_msgs
  geometry_msgs
  tf2_geometry_msgs
  ament_index_cpp
)
target_link_libraries(select_arm_observation_pose_service_node
  lgw_bt_planner_nodes
)
# service_node executable links lgw_bt_planner_nodes SHARED lib which is
# intentionally installed to share/<pkg>/bt_plugins/ (BT.ROS2 dlopen path),
# NOT lib/<pkg>/. Without INSTALL_RPATH the runtime can't find the .so
# because it's outside the standard library search path.
# $ORIGIN = directory of the executable at runtime
#          → installed at lib/<pkg>/, .so at share/<pkg>/bt_plugins/
#          → relative path = ../../share/<pkg>/bt_plugins/
# 2026-10-15: SDK thirdparty path dropped — lgw_bt_planner_nodes no
# longer links the SDK (SetLedBrightnessNode is now a RosServiceNode
# over lgw_vui_bridge; SDK lifecycle lives in helper subprocess).
set_target_properties(select_arm_observation_pose_service_node PROPERTIES
  INSTALL_RPATH
    "$ORIGIN/../../share/${PROJECT_NAME}/bt_plugins"
)
install(TARGETS select_arm_observation_pose_service_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Standalone rclcpp::Node — applies head camera collision object to MoveIt
# planning scene at startup (added 2026-10-10). Loads
# calibration/head_calibration.json, builds a BOX 0.15×0.07×0.07 at the
# head camera position in arm base_link, calls /apply_planning_scene
# synchronously, then self-exits. launch wrapper inserts it between
# agx_arm_moveit/demo.launch.py and the BT runner so MoveIt is ready
# before apply. See apply_head_camera_collision_node.cpp file-top docs.
add_executable(apply_head_camera_collision_node
  src/apply_head_camera_collision_node.cpp
)
target_include_directories(apply_head_camera_collision_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(apply_head_camera_collision_node
  rclcpp
  moveit_msgs
  shape_msgs
  geometry_msgs
  ament_index_cpp
)
install(TARGETS apply_head_camera_collision_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# lgw_servo_node — Servo takeover ROS interface layer (spec 2026-10-26 §7,
# plan Task 7). Owns /lgw/servo/{takeover,release,pause,resume},
# /lgw/servo/state and the /servo_node/delta_twist_cmds publisher that feeds
# moveit_servo. It links lgw_bt_planner_nodes for the five pure servo modules
# (geometry_source / lookat_setpoint / servo_control_law /
# servo_state_machine) plus parse_calibration_json.
# NOTE: servo_node.cpp is deliberately NOT in NODE_SOURCES — that list builds
# the BT.ROS2 plugin SHARED library, and a translation unit with main() must
# not be part of a dlopen-able plugin (it would also drag ament_index_cpp /
# std_srvs / action_msgs undefined symbols into every gtest that links it).
add_executable(servo_node
  src/servo_node.cpp
)
target_include_directories(servo_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(servo_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  action_msgs
  trajectory_msgs
  builtin_interfaces
  tf2
  tf2_ros
  yoloe_interfaces
  ament_index_cpp
)
target_link_libraries(servo_node
  lgw_servo
)
# Same reason as select_arm_observation_pose_service_node above: the SHARED
# plugin lib lives in share/<pkg>/bt_plugins/, not lib/<pkg>/.
set_target_properties(servo_node PROPERTIES
  INSTALL_RPATH
    "$ORIGIN/../../share/${PROJECT_NAME}/bt_plugins"
)
install(TARGETS servo_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY bt_trees/ DESTINATION share/${PROJECT_NAME}/bt_trees)

# Servo stack parameters (2026-10-26 servo takeover, plan Task 6):
#   servo_params.yaml — moveit_servo/servo_node_main (dotted `moveit_servo.*`)
#   servo_node.yaml   — lgw_bt_planner/servo_node defaults (Task 7)
# lgw_full.launch.py loads both from share/${PROJECT_NAME}/config.
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

# Calibration JSONs for ComputeLookatPose node — loaded at node
# construction time via ament_index_cpp::get_package_share_directory.
# arm_calibration.json MUST be the _tcp.json variant (T_tcp_link →
# camera_arm_color_optical_frame); see spec §4.1, §9 #4.
install(DIRECTORY calibration/ DESTINATION share/${PROJECT_NAME}/calibration)

ament_export_targets(lgw_bt_planner_nodes HAS_LIBRARY_TARGET)
ament_export_dependencies(behaviortree_cpp behaviortree_ros2 moveit_msgs btcpp_ros2_interfaces unitree_api unitree_go)

# ==== Service message generation ====
# rosidl_generate_interfaces enables building/using the custom service
# message `lgw_bt_planner::srv::SelectArmObservationPose` (declared in
# srv/SelectArmObservationPose.srv). Required by both the standalone
# select_arm_observation_pose_service_node executable and the BT thin
# shell node SelectArmObservationPoseBTNode.
#
# Added 2026-10-27 (servo takeover, plan Task 1):
#   msg/ServoState.msg -> lgw_bt_planner::msg::ServoState
#   srv/Takeover.srv   -> lgw_bt_planner::srv::Takeover
# NOTE: lgw_bt_planner::msg::ServoState is a different type from the C++
# enum lgw_bt_planner::ServoState in servo_types.hpp — see that header.
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SelectArmObservationPose.srv"
  "srv/Takeover.srv"
  "msg/ServoState.msg"
  DEPENDENCIES geometry_msgs builtin_interfaces
)

# After rosidl_generate_interfaces — the C++ type support target now
# exists. Self-link it so the SHARED plugin lib + the standalone
# service_node executable can resolve
# `rosidl_typesupport_cpp::get_service_type_support_handle<S>()` at link
# time. Pattern lifted from livox_ros_driver2/CMakeLists.txt:287.
rosidl_get_typesupport_target(cpp_typesupport_target_nodes
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(lgw_bt_planner_nodes
  "${cpp_typesupport_target_nodes}"
)
# servo_node links lgw_servo (STATIC) instead of the plugin SHARED lib since
# the 2026-11-12 split, so it no longer inherits the typesupport target
# transitively -- the executable publishes msg::ServoState and offers
# srv::Takeover, so it needs the handle itself.
target_link_libraries(servo_node
  "${cpp_typesupport_target_nodes}"
)
# body_pose_bridge's SetBodyPose.srv typesupport is brought in via
# `ament_target_dependencies(... body_pose_bridge)` above — no
# explicit rosidl_get_typesupport_target needed since the dependency
# package already exports the target via its Config.cmake.
rosidl_get_typesupport_target(cpp_typesupport_target_svc
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(select_arm_observation_pose_service_node
  "${cpp_typesupport_target_svc}"
)
# servo_node publishes lgw_bt_planner::msg::ServoState and serves
# lgw_bt_planner::srv::Takeover — same generated typesupport target.
target_link_libraries(servo_node
  "${cpp_typesupport_target_svc}"
)
# Any consumer of lgw_servo needs the generated type support: the node
# publishes msg::ServoState and serves srv::Takeover, so the gtest binaries
# that instantiate ServoNode need it too, not just the executable.
target_link_libraries(lgw_servo
  "${cpp_typesupport_target_svc}"
)

# gtest binary for the pure-function logic in evaluate_detection.cpp.
# Built only when BUILD_TESTING=ON. Per CLAUDE.md / bt-tech-summary §8,
# this container lacks ament_cmake_test so the test isn't auto-run;
# invoke the binary directly from build/lgw_bt_planner/.
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Scenario-level launch tests (2026-11-07 node-split plan, Task 2). They need
  # no hardware: the servo node runs on its own and asserts its own topics.
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/session_launch_test.py
    TIMEOUT 90
  )
  ament_add_gtest(test_evaluate_detection
    tests/test_evaluate_detection.cpp
  )
  target_link_libraries(test_evaluate_detection lgw_geometry)
  ament_target_dependencies(test_evaluate_detection
    rclcpp
    yoloe_interfaces
  )
  # gtest binary for ComputeLookatPose pure-function math (TC1-11).
  # Task 2 ships TC7-10 (parse_calibration_json); Tasks 3-5 add TC1-6
  # and Task 5 adds the integration smoke TC11.
  ament_add_gtest(test_compute_lookat_pose
    tests/test_compute_lookat_pose.cpp
  )
  target_link_libraries(test_compute_lookat_pose lgw_geometry)
  ament_target_dependencies(test_compute_lookat_pose
    rclcpp
    tf2
    tf2_geometry_msgs
    geometry_msgs
    yoloe_interfaces
  )
  # Diagnostic BT-node tick test (Task 9 真机抖动 root cause hunt).
  # Constructs ComputeLookatPoseNode with REAL install calibration
  # JSONs + synthetic ObservationResult, prints computed tcp pose.
  ament_add_gtest(test_compute_lookat_node_tick
    tests/test_compute_lookat_node_tick.cpp
  )
  target_link_libraries(test_compute_lookat_node_tick lgw_geometry)
  ament_target_dependencies(test_compute_lookat_node_tick
    rclcpp
    tf2
    tf2_geometry_msgs
    geometry_msgs
    yoloe_interfaces
    ament_index_cpp
    behaviortree_cpp
  )
  # gtest for the pure UnifiedTrackArray queries used by the BT leaves.
  ament_add_gtest(test_target_track_query
    tests/test_target_track_query.cpp
  )
  target_include_directories(test_target_track_query PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  ament_target_dependencies(test_target_track_query yoloe_interfaces)
  # gtest for the MoveArmToPose position gate (2026-11-17): a FakeSystem
  # MoveGroup result says nothing about whether the real arm arrived, so
  # the node now fails when ||pos_err|| exceeds max_pos_err_m.
  ament_add_gtest(test_pose_error_gate
    tests/test_pose_error_gate.cpp
  )
  # header-only: no library to link, just the package include dir.
  target_include_directories(test_pose_error_gate PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  # gtest for the xacro branch filter (2026-11-17): keeps per-arm-type
  # group_states from clobbering each other in the named-target table.
  ament_add_gtest(test_xacro_condition_filter
    tests/test_xacro_condition_filter.cpp
    src/xacro_condition_filter.cpp
  )
  target_include_directories(test_xacro_condition_filter PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  # gtest for the pure scan waypoint generator (2026-11-13).
  ament_add_gtest(test_scan_trajectory
    tests/test_scan_trajectory.cpp
  )
  target_link_libraries(test_scan_trajectory lgw_servo)
  ament_add_gtest(test_scan_roll_lock
    tests/test_scan_roll_lock.cpp
  )
  target_link_libraries(test_scan_roll_lock lgw_servo)
  ament_add_gtest(test_scan_publish
    tests/test_scan_publish.cpp
  )
  target_link_libraries(test_scan_publish lgw_servo)
  ament_target_dependencies(test_scan_publish rclcpp trajectory_msgs)
  ament_add_gtest(test_servo_scan_gate
    tests/test_servo_scan_gate.cpp
  )
  target_link_libraries(test_servo_scan_gate lgw_servo)
  ament_target_dependencies(test_servo_scan_gate rclcpp)
  # gtest for servo_control_law pure module (servo takeover, plan Task 4):
  # P-law saturation, deadband (state preserved), first-order low-pass,
  # source_scale on the clamped output, angular sign convention.
  # Math-only, no executor / Servo / MoveIt needed.
  ament_add_gtest(test_servo_params
    tests/test_servo_params.cpp
  )
  target_link_libraries(test_servo_params lgw_servo)
  ament_target_dependencies(test_servo_params rclcpp)
  ament_add_gtest(test_servo_control_law
    tests/test_servo_control_law.cpp
  )
  target_link_libraries(test_servo_control_law lgw_servo)
  # Task 6 of the 2026-11-07 node-split plan: the per-tick pipeline is a class
  # with no rclcpp / TF / clock, so its input rules are unit-testable.
  ament_add_gtest(test_tracking_loop
    tests/test_tracking_loop.cpp
  )
  target_link_libraries(test_tracking_loop lgw_servo)
  # Step 4 of Task 6: the pre-refactor tick block, transcribed as the reference
  # implementation, driven over the same trajectory as TrackingLoop.
  ament_add_gtest(test_tracking_loop_replay
    tests/test_tracking_loop_replay.cpp
  )
  target_link_libraries(test_tracking_loop_replay lgw_servo)
  # Task 7 of the 2026-11-07 node-split plan: the recovery decision is pure.
  ament_add_gtest(test_recovery_orchestrator
    tests/test_recovery_orchestrator.cpp
  )
  target_link_libraries(test_recovery_orchestrator lgw_servo)
  # Task 8, step 4b: what /lgw/servo/state is allowed to say.
  ament_add_gtest(test_qp_j5_sweep
    tests/test_qp_j5_sweep.cpp
  )
  target_link_libraries(test_qp_j5_sweep lgw_servo)
  ament_add_gtest(test_servo_state_reporter
    tests/test_servo_state_reporter.cpp
  )
  target_link_libraries(test_servo_state_reporter lgw_servo)
  # Task 8: the lifecycle services are idempotent and never make a blocking
  # Servo call they do not need.
  ament_add_gtest(test_servo_session
    tests/test_servo_session.cpp
  )
  target_link_libraries(test_servo_session lgw_servo)
  # Task 4 preparatory step: the node declares exactly this set of parameter
  # names, so the generate_parameter_library migration cannot silently rename
  # or drop one.
  ament_add_gtest(test_param_contract
    tests/test_param_contract.cpp
  )
  target_link_libraries(test_param_contract lgw_servo)
  # Proves servo_node.hpp carries every include its declarations need: this TU
  # includes nothing from the package but the header.
  ament_add_gtest(test_servo_node_header_includes
    tests/test_servo_node_header_includes.cpp
  )
  target_link_libraries(test_servo_node_header_includes lgw_servo)
  # Task 1 of the 2026-11-07 node-split plan: the node class is linkable and the
  # control tick takes an explicit timestamp, so the whole tick is drivable from
  # a test / the replay harness instead of wall-clock time.
  ament_add_gtest(test_servo_node_tick
    tests/test_servo_node_tick.cpp
  )
  target_include_directories(test_servo_node_tick PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_servo_node_tick lgw_servo)
  ament_target_dependencies(test_servo_node_tick
    rclcpp
    geometry_msgs
    sensor_msgs
    std_msgs
    std_srvs
    action_msgs
    trajectory_msgs
    builtin_interfaces
    tf2
    tf2_ros
    yoloe_interfaces
    ament_index_cpp
  )
  ament_target_dependencies(test_servo_control_law
    tf2
  )
  # gtest for servo_state_machine pure module (servo takeover, plan Task 5):
  # §7.1 transition table (takeover gating on move_group_busy, unpause
  # failure -> FAULT, HOLD keeps control and resumes after TARGET_OK, FAULT
  # needs RESET) + StreamWatchdog arrival-time liveness. No rclcpp / no
  # executor / no MoveIt needed.
  ament_add_gtest(test_servo_state_machine
    tests/test_servo_state_machine.cpp
  )
  target_link_libraries(test_servo_state_machine lgw_servo)
  # gtest for the HOLD recovery criteria (spec 2026-11-03 §4): reason
  # whitelist, budgets, geometry age, follower_lag never moving.
  ament_add_gtest(test_hold_recovery_decision
    tests/test_hold_recovery_decision.cpp
  )
  # Same pattern as test_preposition_decision: the module lives in the small
  # static lgw_posture lib, so the test does not rebuild the 5 MB plugin.
  target_link_libraries(test_hold_recovery_decision lgw_posture)
  # gtest for select_arm_observation_pose_geom pure module — TC-G1..G11.
  # Math-only tests, no executor / no MoveIt needed.
  ament_add_gtest(test_select_arm_observation_pose_geom
    tests/test_select_arm_observation_pose_geom.cpp
  )
  target_link_libraries(test_select_arm_observation_pose_geom lgw_geometry)
  ament_target_dependencies(test_select_arm_observation_pose_geom
    tf2
    tf2_geometry_msgs
    geometry_msgs
  )
  # gtest for the pinned-posture lookat family (plan 2026-10-31). KDL chain +
  # FK, the 3-DOF damped-least-squares aim, the j2 candidate search and its
  # guards. Math-only: no executor / no MoveIt runtime, the chain is parsed
  # from the installed piper URDF.
  ament_add_gtest(test_lookat_posture_family
    tests/test_lookat_posture_family.cpp
  )
  # Solver interface + startup selection (plan 2026-11-05, Task 1).
  ament_add_gtest(test_lookat_solver_select
    tests/test_lookat_solver_select.cpp
  )
  target_link_libraries(test_lookat_solver_select lgw_posture lgw_geometry)
  # 画面不许倒转的硬判据 (2026-11-19)。纯数学, 只依赖 header 里的 inline 函数。
  ament_add_gtest(test_lookat_upright
    tests/test_lookat_upright.cpp
  )
  target_link_libraries(test_lookat_upright lgw_posture)
  # Vendored qpmad smoke test (plan 2026-11-05, Task 2).
  ament_add_gtest(test_qpmad_smoke
    tests/test_qpmad_smoke.cpp
  )
  target_link_libraries(test_qpmad_smoke lgw_posture lgw_geometry)
  target_include_directories(test_qpmad_smoke PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/qpmad/include)
  # QP lookat solver (plan 2026-11-05, Task 3).
  ament_add_gtest(test_lookat_qp_solver
    tests/test_lookat_qp_solver.cpp
  )
  target_link_libraries(test_lookat_qp_solver lgw_posture lgw_geometry)
  # Coverage/continuity A/B (plan 2026-11-05, Task 6).
  ament_add_gtest(test_lookat_solver_ab
    tests/test_lookat_solver_ab.cpp
  )
  target_link_libraries(test_lookat_solver_ab lgw_posture lgw_geometry)
  ament_target_dependencies(test_lookat_solver_ab
    ament_index_cpp
    tf2
  )
  ament_target_dependencies(test_lookat_qp_solver
    ament_index_cpp
    tf2
  )
  # Generic task solve (2026-11-19): pose IK engine shared by the grasp /
  # pre-grasp generation. The look-at path is untouched; these tests only
  # exercise solve_task().
  ament_add_gtest(test_pose_ik
    tests/test_pose_ik.cpp
  )
  target_link_libraries(test_pose_ik lgw_posture lgw_geometry)
  ament_target_dependencies(test_pose_ik
    ament_index_cpp
    tf2
  )
  # Grasp / pre-grasp SolveTask implementations (2026-11-19).
  ament_add_gtest(test_grasp_tasks
    tests/test_grasp_tasks.cpp
  )
  target_link_libraries(test_grasp_tasks lgw_posture lgw_geometry)
  ament_target_dependencies(test_grasp_tasks
    ament_index_cpp
    tf2
  )
  ament_target_dependencies(test_lookat_solver_select
    ament_index_cpp
    tf2
  )
  # Pre-position criteria (plan 2026-10-31, spec §3.8 rev3). Pure math.
  ament_add_gtest(test_preposition_decision
    tests/test_preposition_decision.cpp
  )
  target_link_libraries(test_preposition_decision lgw_posture)
  target_link_libraries(test_lookat_posture_family lgw_posture lgw_geometry)
  ament_target_dependencies(test_lookat_posture_family
    ament_index_cpp
    tf2
  )
  ament_target_dependencies(test_preposition_decision tf2)
  # MoveGroup client: fake /move_action server runs inside the test process.
  ament_add_gtest(test_move_group_prepositioner
    tests/test_move_group_prepositioner.cpp
  )
  target_link_libraries(test_move_group_prepositioner lgw_preposition)
  ament_target_dependencies(test_move_group_prepositioner
    rclcpp rclcpp_action moveit_msgs
  )
endif()

ament_package()