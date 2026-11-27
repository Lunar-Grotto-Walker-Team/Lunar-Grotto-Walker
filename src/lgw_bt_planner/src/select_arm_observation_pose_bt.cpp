// select_arm_observation_pose_bt.cpp — BT thin shell implementation. See
// header for semantics. Pairs with the standalone service node
// `select_arm_observation_pose_service_node`.

#include "lgw_bt_planner/select_arm_observation_pose_bt.hpp"

#include <rclcpp/logging.hpp>

#include <cmath>
#include <string>

namespace lgw_bt_planner
{

SelectArmObservationPoseBTNode::SelectArmObservationPoseBTNode(
  const std::string& name,
  const BT::NodeConfig& cfg,
  const BT::RosNodeParams& params)
: BT::RosServiceNode<
    lgw_bt_planner::srv::SelectArmObservationPose>(name, cfg, params)
{
}

BT::PortsList SelectArmObservationPoseBTNode::providedPorts()
{
  // BT.ROS2 v0.3.0 RosServiceNode 强制要求 subclass 通过
  // providedBasicPorts() 暴露 `service_name` 端口(头文件 line 94):
  //   "Any subclass of RosServiceNode that has ports must implement
  //    a providedPorts method and call providedBasicPorts in it."
  // 漏调 → 构造时和 tick() 都找不到 service_name,基类抛
  // "no service client was specified neither as default or in the ports"
  // (真机 2026-10-09 第一次 send_goal 复现)。
  return providedBasicPorts({
    BT::InputPort<geometry_msgs::msg::Point>(
      "head_target_world",
      "目标在 base_link 系下的 3D 坐标(上游 ComputeHeadTargetWorld 输出)"),
    BT::InputPort<yoloe_interfaces::msg::ObservationResult>(
      "last_head_obs",
      "头相机最后 1 个 ObservationResult,用于算 d_head = ||center_m||"),
    BT::InputPort<std::string>(
      "preferred_ratios_csv",
      "0.7,0.85,1.0,1.2",
      "r 候选 csv,默认 \"0.7,0.85,1.0,1.2\""),
    BT::InputPort<std::string>(
      "preferred_alphas_deg_csv",
      "90,60,45",
      "α 候选 csv(deg),默认 \"90,60,45\""),
    BT::InputPort<std::string>(
      "ik_call_timeout_ms",
      "50",
      "单次 IK 调用超时 ms,默认 50"),
    BT::InputPort<std::string>(
      "arm_base_frame",
      "base_link",
      "机位 yaw 参考系,默认 base_link"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>(
      "tcp_pose_in_world",
      "选中 tcp_pose,header.frame_id='base_link',下游 MoveArmToPose 用"),
    BT::OutputPort<double>(
      "selected_alpha_deg",
      "选中候选的 α (deg),debug"),
    BT::OutputPort<double>(
      "selected_ratio",
      "选中候选的 r,debug"),
    BT::OutputPort<int>(
      "n_reachable",
      "12 候选里 IK 成功个数,debug"),
    BT::OutputPort<int>(
      "n_total_iterated",
      "实际跑的 IK 次数(早退出可 < 12),debug"),
  });
}

bool SelectArmObservationPoseBTNode::setRequest(
  typename Service::Request::SharedPtr& req)
{
  // head_target_world (required)
  if (auto p = getInput<geometry_msgs::msg::Point>("head_target_world");
      p) {
    req->target_world = *p;
  } else {
    return false;
  }

  // last_head_obs (required for d_head)
  auto obs_in = getInput<yoloe_interfaces::msg::ObservationResult>(
    "last_head_obs");
  if (!obs_in || obs_in->detections.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("SelectArmObservationPoseBT"),
      "missing last_head_obs; cannot compute d_head");
    return false;
  }
  const auto& c = obs_in->detections.front().center_m;
  req->d_head = std::sqrt(c.x * c.x + c.y * c.y + c.z * c.z);

  // csv / timeout / frame — defaults from providedPorts() are used when
  // the XML doesn't override; the empty-check skips the explicit
  // fallback (port default already supplies the value).
  if (auto p = getInput<std::string>("preferred_ratios_csv");
      p && !p->empty()) {
    req->preferred_ratios_csv = *p;
  }
  if (auto p = getInput<std::string>("preferred_alphas_deg_csv");
      p && !p->empty()) {
    req->preferred_alphas_deg_csv = *p;
  }
  if (auto p = getInput<std::string>("ik_call_timeout_ms");
      p && !p->empty()) {
    try {
      req->ik_call_timeout_ms = std::stoi(*p);
    } catch (const std::exception& e) {
      RCLCPP_WARN(rclcpp::get_logger("SelectArmObservationPoseBT"),
        "bad ik_call_timeout_ms='%s': %s; using 50", p->c_str(), e.what());
      req->ik_call_timeout_ms = 50;
    }
  }
  if (auto p = getInput<std::string>("arm_base_frame");
      p && !p->empty()) {
    req->arm_base_frame = *p;
  }
  return true;
}

BT::NodeStatus SelectArmObservationPoseBTNode::onResponseReceived(
  const typename Service::Response::SharedPtr& resp)
{
  if (!resp->success) {
    RCLCPP_ERROR(rclcpp::get_logger("SelectArmObservationPoseBT"),
      "service success=false: '%s' (n_reachable=%d n_iter=%d)",
      resp->message.c_str(), resp->n_reachable, resp->n_total_iterated);
    return BT::NodeStatus::FAILURE;
  }

  setOutput("tcp_pose_in_world", resp->selected_tcp_pose_in_world);
  setOutput("selected_alpha_deg", resp->selected_alpha_deg);
  setOutput("selected_ratio",     resp->selected_ratio);
  setOutput("n_reachable",        resp->n_reachable);
  setOutput("n_total_iterated",   resp->n_total_iterated);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace lgw_bt_planner
