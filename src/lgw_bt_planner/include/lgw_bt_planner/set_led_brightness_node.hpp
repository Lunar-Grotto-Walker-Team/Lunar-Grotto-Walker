// set_led_brightness_node.hpp
//
// BT::RosServiceNode thin shell over lgw_vui_bridge::srv::SetVuiBrightness.
// The actual SDK call (VuiClient::SetBrightness, api_id=1005) lives in a
// one-shot subprocess `go2_vui_set_brightness_helper` spawned by
// vui_bridge_node — this is the in-process → subprocess migration that
// avoids the rmw_cyclonedds_cpp vs libunitree_sdk2.a libddscxx ABI
// collision (see feedback_unitree_sdk_cyclonedds_process_isolation.md).
//
// IMPORTANT — naming caveat: "LED" in the node name is shorthand.
// Go2 EDU 头部"灯"实际上是一块屏幕(可调亮度),不是 RGB LED 灯带。
// 本节点调的是这块屏幕的亮度,通过 VUI service 经 lgw_vui_bridge 与
// helper 子进程间接走 SDK `rt/api/vui/request`,与之前 ROS 2 publish
// 路径同一根 topic。
//
// Migration history:
//   v1 (2026-10-13) — BT::StatefulActionNode publishing
//                     unitree_api::msg::Request to rt/api/vui/request.
//                     真机屏幕亮度无变化(firmware no-op)。
//   v2 (2026-10-14) — BT::StatefulActionNode calling SDK VuiClient
//                     directly in-process. 真机可改亮度但 bt_runner
//                     SIGSEGV(exit -11)at first plugin instantiation
//                     due to in-process cyclone DDS double-participant
//                     clash between rclcpp's rmw_cyclonedds_cpp and
//                     libunitree_sdk2.a.
//   v3 (2026-10-15, current) — BT::RosServiceNode over
//                              lgw_vui_bridge::srv::SetVuiBrightness.
//                              SDK lifecycle lives in helper subprocess;
//                              bt_runner owns only rmw DomainParticipant.
//
// Fire-and-forget contract preserved: bridge helper returns success=true
// iff VuiClient::SetBrightness returned 0; this node maps that to BT
// SUCCESS. Helper exit != 0 (link down, firmware rejected, helper
// crashed) → success=false → this node maps to BT FAILURE (a change
// from v2 which was SUCCESS-anyway; rationale: now that we have a
// reliable subprocess signal, the BT should fail loud so the operator
// notices a lost dog mid-tree).

#pragma once

#include <behaviortree_ros2/bt_service_node.hpp>

#include <lgw_vui_bridge/srv/set_vui_brightness.hpp>

#include <string>

namespace lgw_bt_planner
{

class SetLedBrightnessNode
  : public BT::RosServiceNode<lgw_vui_bridge::srv::SetVuiBrightness>
{
public:
  using Service = lgw_vui_bridge::srv::SetVuiBrightness;

  SetLedBrightnessNode(const std::string& name, const BT::NodeConfig& config,
                       const BT::RosNodeParams& params);

  SetLedBrightnessNode() = delete;
  SetLedBrightnessNode(const SetLedBrightnessNode&) = delete;
  SetLedBrightnessNode& operator=(const SetLedBrightnessNode&) = delete;

  static BT::PortsList providedPorts();

protected:
  bool setRequest(typename Service::Request::SharedPtr& req) override;
  BT::NodeStatus onResponseReceived(
    const typename Service::Response::SharedPtr& resp) override;
};

}  // namespace lgw_bt_planner
