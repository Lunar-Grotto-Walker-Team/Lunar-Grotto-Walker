// bt_plugin_registration.cpp — Register ONLY the 3 BT.ROS2 nodes used by
// pick_and_place_demo.xml: MoveArmToPose, MoveArmToNamed, SetPoseStamped.
// Other nodes (AttachObject, DetachObject, CheckPoseReachable, ControlGripper,
// WaitForUserInput, WaitForStage1/2, ComputePickPose) were removed when
// lgw_bt_planner was trimmed to pick-and-place scope (spec §4.2).
//
// Compiled into lgw_bt_planner_nodes SHARED library with -DBT_PLUGIN_EXPORT
// defined (see CMakeLists.txt). BT.ROS2 dlopens the .so and calls
// BT_RegisterRosNodeFromPlugin via the BT_REGISTER_ROS_NODES macro below.

#include "lgw_bt_planner/control_gripper_node.hpp"
#include "lgw_bt_planner/is_detection_confirmed_node.hpp"
#include "lgw_bt_planner/move_arm_to_named_action_node.hpp"
#include "lgw_bt_planner/move_arm_to_pose_action_node.hpp"
#include "lgw_bt_planner/set_pose_stamped_node.hpp"
#include "lgw_bt_planner/wait_for_user_input_bt_node.hpp"
#include "lgw_bt_planner/compute_lookat_pose_node.hpp"
#include "lgw_bt_planner/verify_head_arm_consistency.hpp"
#include "lgw_bt_planner/compute_grasp_pose.hpp"
#include "lgw_bt_planner/compute_head_target_world.hpp"
#include "lgw_bt_planner/compute_arm_target_world.hpp"
#include "lgw_bt_planner/make_pose_from_point_and_reference.hpp"
#include "lgw_bt_planner/stand_up_node.hpp"
#include "lgw_bt_planner/balance_stand_node.hpp"
#include "lgw_bt_planner/stand_down_node.hpp"
#include "lgw_bt_planner/set_body_height_node.hpp"
#include "lgw_bt_planner/set_body_pose_node.hpp"
#include "lgw_bt_planner/switch_gait_node.hpp"
#include "lgw_bt_planner/set_led_brightness_node.hpp"
#include "lgw_bt_planner/start_mapping_node.hpp"
#include "lgw_bt_planner/select_arm_observation_pose_bt.hpp"
#include "lgw_bt_planner/target_track_nodes.hpp"
#include "lgw_bt_planner/track_service_nodes.hpp"
#include "lgw_bt_planner/compute_grasp_poses_from_track.hpp"
#include "lgw_bt_planner/compute_grasp_poses_sampled.hpp"
#include "lgw_bt_planner/base_pose_nodes.hpp"
#include "lgw_bt_planner/target_track_nodes.hpp"
#include "lgw_bt_planner/track_service_nodes.hpp"
#include "lgw_bt_planner/compute_grasp_poses_from_track.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_ros2/plugins.hpp>

#include <chrono>

BT_REGISTER_ROS_NODES(factory, params)
{
  // params.nh is std::weak_ptr<rclcpp::Node>; RosNodeParams constructors only
  // accept std::shared_ptr, so lock() it once. The node handle is the
  // bt_runner_node itself, which outlives this call.
  auto nh_shared = params.nh.lock();
  if (!nh_shared) {
    throw std::runtime_error(
      "BT_RegisterRosNodeFromPlugin: params.nh weak_ptr expired; "
      "the rclcpp::Node passed to BT::RegisterRosNode must outlive the call");
  }

  factory.registerNodeType<lgw_bt_planner::MoveArmToPoseActionNode>(
    "MoveArmToPose", BT::RosNodeParams(nh_shared, "/move_action"));

  factory.registerNodeType<lgw_bt_planner::MoveArmToNamedActionNode>(
    "MoveArmToNamed",
    BT::RosNodeParams(nh_shared, "/move_action"));

  factory.registerNodeType<lgw_bt_planner::SetPoseStampedNode>(
    "SetPoseStamped");

  // 夹爪开合 / 速度 / 力判据(2026-11-16)。直接向
  // /gripper_controller/joint_trajectory 发两点轨迹, 不经过 MoveIt 规划。
  factory.registerNodeType<lgw_bt_planner::ControlGripperNode>(
    "ControlGripper", BT::RosNodeParams(nh_shared));

  factory.registerNodeType<lgw_bt_planner::IsHeadDetectionConfirmed>(
    "IsHeadDetectionConfirmed", BT::RosNodeParams(nh_shared));
  factory.registerNodeType<lgw_bt_planner::IsArmDetectionConfirmed>(
    "IsArmDetectionConfirmed", BT::RosNodeParams(nh_shared));

  // WaitForUserInput: snapshot-style gate for user_gated_auto_pick_and_place
  // (ReactiveFallback root → WaitForUserInput + Repeat num_cycles="-1").
  // default_port_value "/bt_user_input" so XML can omit topic_name unless
  // overriding per-instance.
  factory.registerNodeType<lgw_bt_planner::WaitForUserInputActionNode>(
    "WaitForUserInput", BT::RosNodeParams(nh_shared, "/bt_user_input"));

  factory.registerNodeType<lgw_bt_planner::ComputeLookatPoseNode>(
    "ComputeLookatPose");

  // VerifyHeadArmConsistency: head/arm ObservationResult cross-check in
  // world frame; writes arm_target_world (Point) + consistency_error_m
  // (double) to blackboard for downstream ComputeGraspPose.
  // Pure SyncActionNode — no ROS subscriptions; reads blackboard ports only.
  factory.registerNodeType<lgw_bt_planner::VerifyHeadArmConsistencyNode>(
    "VerifyHeadArmConsistency");

  // ComputeGraspPose: tcp at arm_target_world with lookat orientation
  // re-anchored to the new target. Pure SyncActionNode.
  factory.registerNodeType<lgw_bt_planner::ComputeGraspPoseNode>(
    "ComputeGraspPose");

  // ComputeHeadTargetWorld / ComputeArmTargetWorld (added 2026-10-08):
  // pure transform helpers for head_arm_independent_lookat.xml — emit
  // world-frame Point so a downstream ComputeGraspPose can drive tcp
  // straight to the head-camera or arm-camera view of the target,
  // without any cross-camera consistency check. SyncActionNode.
  factory.registerNodeType<lgw_bt_planner::ComputeHeadTargetWorldNode>(
    "ComputeHeadTargetWorld");
  factory.registerNodeType<lgw_bt_planner::ComputeArmTargetWorldNode>(
    "ComputeArmTargetWorld");

  // MakePoseFromPointAndReference: trivial pose composition used by
  // head_arm_independent_lookat.xml Phase B/C — reuses a reference pose's
  // orientation while swapping in a new position. Pure SyncActionNode,
  // no transforms.
  factory.registerNodeType<lgw_bt_planner::MakePoseFromPointAndReferenceNode>(
    "MakePoseFromPointAndReference");

  // SelectArmObservationPose: BT thin shell over the standalone
  // /select_arm_observation_pose service (provided by
  // select_arm_observation_pose_service_node). Replaces the hard-coded
  // "0.4;0.0;0.3" ee_position_world anchor in head_arm_independent_lookat
  // .xml step 1 with an adaptive 3-axis selection (fixed yaw along
  // base→target, α tilt ∈ {90°, 60°, 45°} priority-ordered, ratio r ∈
  // {0.7, 0.85, 1.0, 1.2} fallback). Spec:
  // docs/superpowers/specs/2026-10-09-select-arm-observation-pose-spec.md §4.8.
  factory.registerNodeType<lgw_bt_planner::SelectArmObservationPoseBTNode>(
    "SelectArmObservationPose", BT::RosNodeParams(nh_shared));

  factory.registerNodeType<lgw_bt_planner::StandUpNode>(
    "StandUp", BT::RosNodeParams(nh_shared));

  factory.registerNodeType<lgw_bt_planner::BalanceStandNode>(
    "BalanceStand", BT::RosNodeParams(nh_shared));

  factory.registerNodeType<lgw_bt_planner::StandDownNode>(
    "StandDown", BT::RosNodeParams(nh_shared));

  factory.registerNodeType<lgw_bt_planner::SetBodyHeightNode>(
    "SetBodyHeight", BT::RosNodeParams(nh_shared));

  // SetBodyPose: bundles Euler(1007) + BalanceStand(1002) into one
  // node so the BT-tree author cannot mis-order the call sequence.
  // The Go2 state machine latches rpy at BalanceStand time, so
  // Euler after BalanceStand is silently dropped (2026-10-10 lesson).
  // 2026-10-13 v7:BodyHeight(1013)从 3-call 中移除,Go2 固件对
  // api 1013 bh_off 是 dead control(真机 2026-10-10 验证 act
  // body_h 在 bh_off=0 vs -0.18 几乎无差,差 ≤ 2mm)。Replaces the
  // earlier SetEulerPoseNode that exposed the bug at the BT-XML
  // layer.
  //
  // The 2nd RosNodeParams arg is the DEFAULT service name. Without it,
  // bt_service_node.hpp:328 `getInput("service_name", ...)` returns ""
  // and `createClient("")` fails → "no service client was specified
  // neither as default or in the ports" (real-hardware 2026-10-10
  // first-tick crash, see feedback_rosservicenode_providedbasicports.md).
  //
  // 2026-10-10 BCD fix: bump server_timeout from 1 s default to 10 s.
  // The bridge's cv_.wait_until holds for 5 s waiting on tolerance
  // verification; the BT 1 s default timeout fired SERVICE_TIMEOUT
  // ~1 s into HOLDING and aborted the tree before the dog could
  // settle, even when the target rpy was reachable. 10 s leaves a
  // 5 s margin over the bridge verify window for DDS/CPU jitter.
  // wait_for_server_timeout also widened 500 ms → 5 s so launch
  // bringup race (bridge takes longer to advertise than bt_runner
  // to construct clients) doesn't false-fail the first tick.
  BT::RosNodeParams set_body_pose_params(nh_shared, "/set_body_pose");
  set_body_pose_params.server_timeout = std::chrono::milliseconds(10000);
  set_body_pose_params.wait_for_server_timeout = std::chrono::milliseconds(5000);
  factory.registerNodeType<lgw_bt_planner::SetBodyPoseNode>(
    "SetBodyPose", set_body_pose_params);

  // SwitchGait (2026-10-13 added): publishes Go2 gait commands
  // (StaticWalk/TrotRun/EconomicGait/FreeWalk trigger gaits + 7 bool
  // flag gaits) to /api/sport/request. Replaces the deprecated
  // SwitchGait(1011) API with the v0.2.0+ split commands
  // (1061/1062/1063/2045/2046-2051/2054). XML author must invoke
  // BalanceStand first — gait commands are silently dropped
  // otherwise (2026-10-10 lesson).
  factory.registerNodeType<lgw_bt_planner::SwitchGaitNode>(
    "SwitchGait", BT::RosNodeParams(nh_shared));

  // SetLedBrightness (2026-10-15 migrated to RosServiceNode over
  // /set_vui_brightness — was BT::StatefulActionNode calling SDK
  // VuiClient::SetBrightness in-process; that caused bt_runner SIGSEGV
  // at first plugin instantiation due to in-process cyclone DDS
  // double-participant clash between rclcpp's rmw_cyclonedds_cpp
  // DomainParticipant and libunitree_sdk2.a's. SDK lifecycle now
  // lives in a one-shot helper subprocess (go2_vui_set_brightness_helper)
  // spawned by vui_bridge_node — see
  // feedback_unitree_sdk_cyclonedds_process_isolation.md).
  //
  // 2nd RosNodeParams arg is the DEFAULT service_name. Without it,
  // bt_service_node.hpp:328 `getInput("service_name", ...)` returns ""
  // and `createClient("")` fails → "no service client was specified
  // neither as default or in the ports" (real-hardware 2026-10-09
  // first-tick crash pattern that also bit SetBodyPoseNode — see
  // feedback_rosservicenode_providedbasicports.md).
  //
  // 10s server_timeout + 5s wait_for_server_timeout mirror
  // SetBodyPoseNode's settings. The helper popen+SDK Init round-trip
  // on a warm dog is ~50-150ms; the wider window absorbs DDS bringup
  // race (vui_bridge_node takes longer to advertise than bt_runner to
  // construct clients) and out-of-order launch start without false-
  // failing the first tick.
  BT::RosNodeParams set_led_brightness_params(nh_shared, "/set_vui_brightness");
  set_led_brightness_params.server_timeout = std::chrono::milliseconds(10000);
  set_led_brightness_params.wait_for_server_timeout = std::chrono::milliseconds(5000);
  factory.registerNodeType<lgw_bt_planner::SetLedBrightnessNode>(
    "SetLedBrightness", set_led_brightness_params);

  // StartMapping (2026-11-20): triggers point_lio's /start_mapping, which is
  // the other half of the same service that point_lio's `wait_for_start`
  // gate waits on (point_lio drops lidar/IMU until it is called).  The
  // service is one-shot and idempotent, so the node returns SUCCESS on the
  // first accepted call; a repeat call is harmless.
  //
  // 30 s wait_for_server_timeout (vs 5 s for the VUI bridge): the whole
  // point is that this fires minutes into warm-up, and on a cold start the
  // service may not be advertised yet — a 5 s window would false-FAIL the
  // first tick and, worse, teach the tree author to wrap it in
  // ForceSuccess, which would hide a real "LIO is still dropping samples".
  BT::RosNodeParams start_mapping_params(nh_shared, "/start_mapping");
  start_mapping_params.server_timeout = std::chrono::milliseconds(10000);
  start_mapping_params.wait_for_server_timeout = std::chrono::milliseconds(30000);
  factory.registerNodeType<lgw_bt_planner::StartMappingNode>(
    "StartMapping", start_mapping_params);

  // Track-collect validation leaves (2026-11-17).  They consume
  // /perception/tracks/unified and the tracker/servo services; navigation is
  // manual in this tree.  The takeover client gets a generous window because
  // it may pre-position the arm with MoveGroup before Servo unpauses.
  factory.registerNodeType<lgw_bt_planner::IsTargetAvailable>(
    "IsTargetAvailable",
    BT::RosNodeParams(nh_shared, "/perception/tracks/unified"));
  // IsTargetWithin (2026-11-20): 即时条件(永不 RUNNING) —— 新鲜 + XoY 带内。
  // 用作 ReactiveFallback 的左子, 打断巡逻腿(transit_patrol.xml)。
  factory.registerNodeType<lgw_bt_planner::IsTargetWithin>(
    "IsTargetWithin",
    BT::RosNodeParams(nh_shared, "/perception/tracks/unified"));
  factory.registerNodeType<lgw_bt_planner::ComputeGraspPosesFromTrack>(
    "ComputeGraspPosesFromTrack", BT::RosNodeParams(nh_shared));
  // Sampled pose generation (2026-11-20): pitch grid x yaw perturbations,
  // /compute_ik filtered. Used by track_collect_reactive_v2.xml and
  // arm_grasp_test.xml.
  factory.registerNodeType<lgw_bt_planner::ComputeGraspPosesSampled>(
    "ComputeGraspPosesSampled", BT::RosNodeParams(nh_shared));
  factory.registerNodeType<lgw_bt_planner::WaitForTargetSelected>(
    "WaitForTargetSelected", BT::RosNodeParams(nh_shared));
  factory.registerNodeType<lgw_bt_planner::WaitUntilTargetNear>(
    "WaitUntilTargetNear", BT::RosNodeParams(nh_shared));
  factory.registerNodeType<lgw_bt_planner::WaitForArmHits>(
    "WaitForArmHits", BT::RosNodeParams(nh_shared));
  factory.registerNodeType<lgw_bt_planner::IsArmCameraAlive>(
    "IsArmCameraAlive", BT::RosNodeParams(nh_shared));

  BT::RosNodeParams takeover_params(nh_shared, "/lgw/servo/takeover");
  takeover_params.server_timeout = std::chrono::milliseconds(30000);
  takeover_params.wait_for_server_timeout = std::chrono::milliseconds(5000);
  factory.registerNodeType<lgw_bt_planner::ServoTakeoverNode>(
    "ServoTakeover", takeover_params);

  factory.registerNodeType<lgw_bt_planner::ServoReleaseNode>(
    "ServoRelease", BT::RosNodeParams(nh_shared, "/lgw/servo/release"));

  factory.registerNodeType<lgw_bt_planner::MarkTrackNode>(
    "MarkTrack", BT::RosNodeParams(nh_shared, "/lgw/tracker/mark_track"));

  // Base-circle helpers (2026-11-18): RecordBasePose writes the robot's odom
  // pose at mission start into the blackboard; IsWithinBaseRadius compares
  // the live pose against it (XoY).  Pure TF + blackboard, no ROS services.
  factory.registerNodeType<lgw_bt_planner::RecordBasePose>(
    "RecordBasePose", BT::RosNodeParams(nh_shared));
  factory.registerNodeType<lgw_bt_planner::IsWithinBaseRadius>(
    "IsWithinBaseRadius", BT::RosNodeParams(nh_shared));
}