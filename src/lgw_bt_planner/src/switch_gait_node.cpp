// switch_gait_node.cpp
//
// Implements SwitchGaitNode: parse gait_name → Go2 sport API id, publish
// one-shot command, return SUCCESS. See hpp for the full design
// rationale + hardware risks. Pattern A (fire-and-forget) per the v9
// redesign — see feedback_go2_new_gait_state_dead.

#include "lgw_bt_planner/switch_gait_node.hpp"

#include "lgw_bt_planner/sport_api_constants.hpp"

#include <rclcpp/qos.hpp>
#include <rclcpp/logging.hpp>

#include <string>
#include <unordered_map>

namespace lgw_bt_planner
{

// ---- Gait mapping table (singleton, lazy-init) ----
// Short names → (api_id, is_flag). `is_flag` true → the command takes
// a `{"data": <bool>}` JSON param (FreeBound/Jump/Avoid, ClassicWalk,
// WalkUpright, CrossStep, AutoRecoverSet); false → trigger command
// with empty param (StaticWalk/TrotRun/EconomicGait/FreeWalk).
//
// Names are case-sensitive (BT XML authoring should pick one of the
// short aliases below). Unknown names → onStart returns FAILURE
// with a log listing the legal names.
const SwitchGaitNode::GaitEntry * SwitchGaitNode::lookup_gait(
  const std::string& name)
{
  // static table — built once on first call, thread-safe under C++11
  // magic statics (std::call_once equivalent).
  static const std::unordered_map<std::string, GaitEntry> kGaitTable = {
    // ---- 4 trigger gaits (no params) ----
    {"static",         {sport_api::kApiIdStaticWalk,     false}},
    {"static_walk",    {sport_api::kApiIdStaticWalk,     false}},
    {"trot",           {sport_api::kApiIdTrotRun,        false}},
    {"trot_run",       {sport_api::kApiIdTrotRun,        false}},
    {"economic",       {sport_api::kApiIdEconomicGait,   false}},
    {"economic_gait",  {sport_api::kApiIdEconomicGait,   false}},
    {"free_walk",      {sport_api::kApiIdFreeWalk,       false}},
    // ---- 7 bool flag gaits ({"data": enable}) ----
    {"free_bound",     {sport_api::kApiIdFreeBound,      true}},
    {"free_jump",      {sport_api::kApiIdFreeJump,       true}},
    {"free_avoid",     {sport_api::kApiIdFreeAvoid,      true}},
    {"classic_walk",   {sport_api::kApiIdClassicWalk,    true}},
    {"walk_upright",   {sport_api::kApiIdWalkUpright,    true}},
    {"cross_step",     {sport_api::kApiIdCrossStep,      true}},
    {"auto_recover",   {sport_api::kApiIdAutoRecoverSet, true}},
  };

  auto it = kGaitTable.find(name);
  if (it == kGaitTable.end()) {
    return nullptr;
  }
  return &it->second;
}

SwitchGaitNode::SwitchGaitNode(
  const std::string& name, const BT::NodeConfig& config,
  const BT::RosNodeParams& params)
  : BT::StatefulActionNode(name, config)
{
  auto node = params.nh.lock();
  if (!node) {
    throw BT::RuntimeError(
      "SwitchGaitNode: RosNodeParams::nh weak_ptr expired");
  }

  logger_ = node->get_logger();

  // RELIABLE publisher on /api/sport/request (mirrors StandUpNode +
  // official SportClient ros2_sport_client.cpp:71-74). No state
  // subscription in v9 — Pattern A trusts the publish.
  publisher_ = node->create_publisher<unitree_api::msg::Request>(
    sport_api::kSportRequestTopic, rclcpp::QoS(1));

  RCLCPP_INFO(logger_,
    "[%s] SwitchGaitNode initialized: pub=/api/sport/request "
    "(Pattern A, no sub, fire-and-forget — see "
    "feedback_go2_new_gait_state_dead)",
    this->name().c_str());
}

BT::PortsList SwitchGaitNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("gait_name", "trot",
      "Target gait. One of: "
      "static|static_walk, trot|trot_run, economic|economic_gait, free_walk, "
      "free_bound, free_jump, free_avoid, classic_walk, walk_upright, "
      "cross_step, auto_recover. "
      "Trigger gaits (first 4) take no params; flag gaits use `enable` port. "
      "NOTE: walk_upright / cross_step overheat rear motors quickly — "
      "operator responsibility to limit duration. auto_recover: keep "
      "FALSE when carrying head camera / arm payload."),
    BT::InputPort<bool>("enable", true,
      "For flag gaits only: true = enter the mode, false = exit (back "
      "to free_walk per Unitree docs). Ignored for trigger gaits."),
  };
}

BT::NodeStatus SwitchGaitNode::onStart()
{
  // Read ports (BT.CPP InputPort default "" trap: getInput<string>()
  // returns Optional<string>{value} when default provided, so `if (p)`
  // works).
  std::string gait_name = "trot";
  if (auto p = getInput<std::string>("gait_name"); p && !p->empty()) {
    gait_name = *p;
  }
  bool enable = true;
  if (auto p = getInput<bool>("enable"); p) {
    enable = *p;
  }

  // Resolve gait_name → (api_id, is_flag).
  const GaitEntry * entry = lookup_gait(gait_name);
  if (entry == nullptr) {
    RCLCPP_ERROR(logger_,
      "[%s] onStart FAILURE: unknown gait_name='%s'. "
      "Legal: static|static_walk, trot|trot_run, economic|economic_gait, "
      "free_walk, free_bound, free_jump, free_avoid, classic_walk, "
      "walk_upright, cross_step, auto_recover",
      this->name().c_str(), gait_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // Build the request — same format as official SportClient
  // (ros2_sport_client.cpp:175-178 FreeWalk / 204-210 ClassicWalk):
  //   - trigger gaits: empty `parameter` (no JSON)
  //   - flag gaits:    `{"data": true|false}` JSON
  unitree_api::msg::Request req;
  req.header.identity.api_id = entry->api_id;
  if (entry->is_flag) {
    req.parameter = std::string("{\"data\": ") +
                    (enable ? "true" : "false") + "}";
  }
  // else: trigger gait, leave req.parameter as default (empty string)

  publisher_->publish(req);

  // Cache for onHalted logging.
  last_gait_name_ = gait_name;
  last_enable_ = enable;
  last_is_flag_ = entry->is_flag;

  RCLCPP_INFO(logger_,
    "[%s] onStart: published gait='%s' (api_id=%d%s%s) → SUCCESS "
    "(Pattern A fire-and-forget, firmware confirmed via deepwiki "
    "unitree_sdk2_python docs that new gait family does not update "
    "gait_type / mode state field)",
    this->name().c_str(), gait_name.c_str(), entry->api_id,
    entry->is_flag ? (enable ? ", enable=true" : ", enable=false") : "",
    entry->is_flag ? "" : " (trigger, no param)");

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SwitchGaitNode::onRunning()
{
  // Defensive no-op. Pattern A returns SUCCESS from onStart directly,
  // so BT.CPP should not call onRunning in normal flow. If somehow
  // reached (e.g. framework quirk), return SUCCESS to keep the node
  // from being a black hole.
  RCLCPP_WARN(logger_,
    "[%s] onRunning called unexpectedly (Pattern A expects this to be "
    "unreachable) — returning SUCCESS",
    this->name().c_str());
  return BT::NodeStatus::SUCCESS;
}

void SwitchGaitNode::onHalted()
{
  // Pattern A: onStart returns SUCCESS immediately, so this method
  // should be unreachable in normal flow. Kept as a no-op defensive
  // guard — BT.CPP can call onHalted if a Sequence sibling fails
  // during the same tick before onStart runs.
  RCLCPP_INFO(logger_,
    "[%s] onHalted: no-op (Pattern A — onStart returns SUCCESS, "
    "this should be unreachable; gait='%s' was published before halt)",
    this->name().c_str(), last_gait_name_.c_str());
}

}  // namespace lgw_bt_planner
