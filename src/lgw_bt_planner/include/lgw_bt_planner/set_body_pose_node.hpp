// set_body_pose_node.hpp
//
// BT::RosServiceNode thin shell over body_pose_bridge::srv::SetBodyPose.
// The actual 50Hz republish, state-callback, and tolerance verification
// live in the standalone `body_pose_bridge_node`. This node just
// packages the BT ports into a service request and unpacks the
// response.
//
// See docs/superpowers/specs/2026-10-10-body-pose-bridge-design.md.
//
// Replaces the previous BT::StatefulActionNode that did the republish
// in-process — that version lost the fight against the Go2 balance
// controller (which pulls the body back to neutral within ~1s of the
// last Euler(1007) publish). See
// docs/superpowers/specs/2026-10-10-body-pose-bridge-design.md §1.1.
//
// Sign conventions (Go2 sport API):
//   - Euler pitch: Go2 firmware `pitch>0` = nose DOWN (verified on
//     real hardware 2026-10-10 — INVERTED from standard right-hand
//     rule). XML authors must write pitch=+0.5236 for "forward-tilt
//     down".
// Out-of-range values are CLAMPED at the bridge (not in this node).
//
// 2026-10-13 v7 简化:Go2 固件对 BodyHeight api(1013)bh_off 不响应
// (真机验证),body_height 字段从 .srv / BT port 全部删除。body 升降
// 需求由 StandUp 的身高档位(api 1004)覆盖。

#pragma once

#include <behaviortree_ros2/bt_service_node.hpp>

#include <body_pose_bridge/srv/set_body_pose.hpp>

#include <string>

namespace lgw_bt_planner
{

class SetBodyPoseNode
  : public BT::RosServiceNode<body_pose_bridge::srv::SetBodyPose>
{
public:
  using Service = body_pose_bridge::srv::SetBodyPose;

  SetBodyPoseNode(const std::string& name, const BT::NodeConfig& config,
                  const BT::RosNodeParams& params);

  SetBodyPoseNode() = delete;
  SetBodyPoseNode(const SetBodyPoseNode&) = delete;
  SetBodyPoseNode& operator=(const SetBodyPoseNode&) = delete;

  static BT::PortsList providedPorts();

protected:
  bool setRequest(typename Service::Request::SharedPtr& req) override;
  BT::NodeStatus onResponseReceived(
    const typename Service::Response::SharedPtr& resp) override;
};

}  // namespace lgw_bt_planner
