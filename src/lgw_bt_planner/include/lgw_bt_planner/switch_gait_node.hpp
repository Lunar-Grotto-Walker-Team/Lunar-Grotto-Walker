// switch_gait_node.hpp
//
// BT::StatefulActionNode that switches Go2 gait via /api/sport/request.
// Supports 4 trigger gaits (StaticWalk / TrotRun / EconomicGait /
// FreeWalk) + 7 bool flag gaits (FreeBound / FreeJump / FreeAvoid /
// ClassicWalk / WalkUpright / CrossStep / AutoRecoverSet) per Go2 SDK
// v0.2.0+ (2025-07-29), see sport_api_constants.hpp for the api_id
// table.
//
// Pattern: Pattern A (fire-and-forget). onStart fires the gait
// command + returns SUCCESS immediately. NO state polling.
//
// 2026-10-13 v9 redesign: was Pattern B (publish + poll gait_type).
// Reason for change: Go2 firmware processes the v0.2.0+ new gait
// commands (2045/2049/1061-1063/2050-2054) and the dog physically
// makes the gait switch, but DOES NOT update lf/sportmodestate.gait_type
// (or mode / foot_speed_body) — this is documented expected firmware
// behavior, see deepwiki citation of unitree_sdk2_python docs:
// "The 'Free' gait family is decoupled from the classical gait state
// machine. No gait transition is announced — `gait_type` typically
// stays at 0 ... expected firmware behavior, not a bug." Same applies
// to 1061/1062/1063 (trigger gaits). The official ros2_sport_client
// itself only publishes, no state verify, so the SDK canonical pattern
// is also fire-and-forget. See feedback memory
// feedback_go2_new_gait_state_dead.
//
// Prerequisites (NOT enforced here, BT author responsibility):
//   - Dog must already be in BalanceStand / default-stand — the gait
//     command is silently dropped otherwise. SetBodyPose with
//     `clear="true"` (which sends Euler(0,0,0) + BalStand pair in the
//     2-call path) is the canonical prerequisite in this project.
//
// Hardware risks (some flag gaits overheat motors — see port docstrings):
//   - WalkUpright, CrossStep: rear-leg motors overheat quickly.
//   - AutoRecoverSet: keep FALSE when carrying payloads (head camera,
//     arm) to avoid payload damage on auto-recover flip.
//   These are operator-responsibility; the BT node does NOT refuse
//   to publish — safety policy lives at the BT-tree authoring layer.

#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <rclcpp/publisher.hpp>
#include <unitree_api/msg/request.hpp>

#include <memory>
#include <string>

namespace lgw_bt_planner
{

class SwitchGaitNode : public BT::StatefulActionNode
{
public:
  SwitchGaitNode(const std::string& name, const BT::NodeConfig& config,
                 const BT::RosNodeParams& params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  // onRunning() is required by BT::StatefulActionNode static_assert
  // (pure virtual in base). In v9 Pattern A onStart returns SUCCESS
  // directly, so BT.CPP never calls onRunning in normal flow — this
  // is a defensive no-op just to satisfy the abstract-class check.
  BT::NodeStatus onRunning() override;
  void onHalted() override;  // defensive no-op (should be unreachable)

private:
  // Internal gait entry — {api_id, is_flag}. is_flag = true means the
  // gait takes a {"data": enable} bool JSON param; is_flag = false
  // means trigger (empty param, enable port ignored).
  struct GaitEntry
  {
    int32_t api_id;
    bool is_flag;
  };

  // gait_name → (api_id, is_flag). nullptr = not found.
  static const GaitEntry * lookup_gait(const std::string& name);

  std::shared_ptr<rclcpp::Publisher<unitree_api::msg::Request>> publisher_;
  // Cached on onStart so onHalted can log which gait was last
  // commanded without re-reading the BT port.
  std::string last_gait_name_;
  bool last_enable_{true};
  bool last_is_flag_{false};
  rclcpp::Logger logger_{rclcpp::get_logger("lgw_bt_planner.switch_gait")};
};

}  // namespace lgw_bt_planner
