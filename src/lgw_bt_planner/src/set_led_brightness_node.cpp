// set_led_brightness_node.cpp
//
// BT thin shell over lgw_vui_bridge::srv::SetVuiBrightness. Pairs with
// vui_bridge_node + go2_vui_set_brightness_helper subprocess. See header
// for the v1→v2→v3 migration history and the in-process SDK
// double-participant SIGSEGV that drove the rewrite.
//
// 2026-10-15 v3: BT::StatefulActionNode (in-process SDK) →
//                BT::RosServiceNode over /set_vui_brightness. SDK
//                lifecycle moved to helper subprocess. bt_plugin_registration
//                registers default service_name=/set_vui_brightness with
//                10s server_timeout / 5s wait_for_server_timeout (mirrors
//                SetBodyPoseNode settings — single-digit-second latency is
//                well under both thresholds; the wider window absorbs
//                DDS bringup race + helper popen + SDK Init on first tick).

#include "lgw_bt_planner/set_led_brightness_node.hpp"

#include "lgw_bt_planner/sport_api_constants.hpp"

#include <rclcpp/logging.hpp>

namespace lgw_bt_planner
{

SetLedBrightnessNode::SetLedBrightnessNode(
  const std::string& name, const BT::NodeConfig& cfg,
  const BT::RosNodeParams& params)
: BT::RosServiceNode<Service>(name, cfg, params)
{
}

BT::PortsList SetLedBrightnessNode::providedPorts()
{
  // providedBasicPorts() exposes `service_name` (default set in
  // bt_plugin_registration.cpp to "/set_vui_brightness"). XML
  // authors can override per-instance via <SetLedBrightness
  // service_name="..."> but the default is correct for the standard
  // posture demo launch.
  return providedBasicPorts({
    BT::InputPort<int>("level", sport_api::kVuiBrightnessDefault,
      std::string("Brightness level 0-10 (0=dimmest, 10=brightest, default=") +
        std::to_string(sport_api::kVuiBrightnessDefault) +
        "). Out-of-range is forwarded to firmware; bridge returns "
        "success=false (VuiClient::SetBrightness ret != 0) and this "
        "node FAILUREs the BT so the operator sees a lost-dog mid-tree "
        "(matches body_pose_bridge contract). "
        "`go2_vui_client.cpp:32` `++level %= 11` firmware range check."),
  });
}

bool SetLedBrightnessNode::setRequest(
  typename Service::Request::SharedPtr& req)
{
  // BT.CPP InputPort default "" trap (see feedback_bt_inputport_default.md):
  // for numeric types (int / double / bool), Optional<T> predicate is
  // truthy iff the port exists in XML — we can rely on `if (p)` here.
  if (auto p = getInput<int>("level"); p) {
    req->level = *p;
  } else {
    // Provided default already in providedPorts()'s port signature,
    // but the empty-XML fallback (no <SetLedBrightness level="…"> in
    // XML) still needs a sensible value.
    req->level = sport_api::kVuiBrightnessDefault;
  }
  return true;
}

BT::NodeStatus SetLedBrightnessNode::onResponseReceived(
  const typename Service::Response::SharedPtr& resp)
{
  if (!resp->success) {
    RCLCPP_ERROR(rclcpp::get_logger("lgw_bt_planner.set_led_brightness"),
      "service /set_vui_brightness success=false, message='%s' "
      "(dog power? eth10 link? EDU screen? out-of-range level? "
      "vui_bridge_node alive in launch?)",
      resp->message.c_str());
    return BT::NodeStatus::FAILURE;
  }
  RCLCPP_INFO(rclcpp::get_logger("lgw_bt_planner.set_led_brightness"),
    "service /set_vui_brightness success=true, message='%s'",
    resp->message.c_str());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace lgw_bt_planner
