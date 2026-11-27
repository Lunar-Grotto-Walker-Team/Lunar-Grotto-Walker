// select_arm_observation_pose_service_node.cpp — 独立 rclcpp::Node,提供
// /select_arm_observation_pose 服务。BT 节点(SelectArmObservationPoseBTNode)
// 调此服务;节点内部循环 12 次 /compute_ik 验证可达候选,挑最优返回。
//
// 启动时序:launch 保证本节点在 agx_arm_moveit/demo.launch.py 之后起,
// 否则 /compute_ik 不可用,handler 返 success=false。
//
// 设计依据:docs/superpowers/specs/2026-10-09-select-arm-observation-pose-spec.md §4

#include "lgw_bt_planner/select_arm_observation_pose_geom.hpp"
#include "lgw_bt_planner/srv/select_arm_observation_pose.hpp"
#include "lgw_bt_planner/compute_lookat_pose_node.hpp"  // parse_calibration_json

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <moveit_msgs/srv/get_position_ik.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace lgw_bt_planner
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr double kJointStateMaxAgeSec = 2.0;  // staleness check
constexpr double kAlpha90_tolerance = 1e-6;  // match enumerate() exactness
constexpr double kRatio07_tolerance = 1e-6;

// Tiny CSV parser: comma-separated doubles, no whitespace tolerance.
bool parse_csv_doubles(const std::string& s, std::vector<double>& out)
{
  out.clear();
  std::stringstream ss(s);
  std::string item;
  while (std::getline(ss, item, ',')) {
    if (item.empty()) continue;
    try {
      size_t pos = 0;
      const double v = std::stod(item, &pos);
      if (pos != item.size()) return false;
      out.push_back(v);
    } catch (const std::exception&) {
      return false;
    }
  }
  return !out.empty();
}
}  // namespace

class SelectArmObservationPoseServiceNode : public rclcpp::Node
{
public:
  explicit SelectArmObservationPoseServiceNode(
    const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
  : rclcpp::Node("select_arm_observation_pose_service", opts)
  {
    using std::placeholders::_1;
    using std::placeholders::_2;

    // Load arm hand-eye calibration (camera→tcp). Required so the
    // geom module can convert the camera-frame candidate pose into a
    // tcp_link pose that /compute_ik can solve and that actually places
    // the arm camera at the designed observation position. Identity
    // handeye would make IK succeed (tcp at the designed camera
    // position) but the real camera would be offset by t_a2t (~8cm
    // away from the target) — see file-top design note.
    try {
      const std::string share_dir =
        ament_index_cpp::get_package_share_directory("lgw_bt_planner");
      const std::string calib_path =
        share_dir + "/calibration/arm_calibration.json";
      std::tie(R_a2t_, t_a2t_) = parse_calibration_json(calib_path);
      RCLCPP_INFO(this->get_logger(),
        "loaded arm handeye: %s  t_a2t=(%.4f,%.4f,%.4f)",
        calib_path.c_str(),
        t_a2t_.x(), t_a2t_.y(), t_a2t_.z());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(),
        "failed to load arm_calibration.json — candidates will be "
        "computed with identity handeye (t_a2t=0); the resulting "
        "tcp_link pose will NOT place the real arm camera at the "
        "designed observation position. err=%s", e.what());
      R_a2t_ = tf2::Matrix3x3();   // identity
      t_a2t_ = tf2::Vector3();     // zero
    }

    // /joint_states — 默认 QoS sensor data,缓存最新 1 帧(含时间戳)
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&SelectArmObservationPoseServiceNode::on_joint_state, this, _1));

    // /compute_ik client — MoveIt 内置服务
    //
    // 2026-10-09 修复:必须挂到独立的 MutuallyExclusive callback group 上。
    // 原代码在 ik_check() 里直接调
    //   rclcpp::spin_until_future_complete(
    //     this->get_node_base_interface(), fut, deadline)
    // 该 static helper 内部会 new 一个 SingleThreadedExecutor 并把整个 node
    // add 进去 —— 但 main() 已经在 rclcpp::spin(node) 把同一 node 交给了另
    // 一个 executor,同一 rclcpp::Node 不能属两个 executor,第二次 add 抛
    // "Node '...select_arm_observation_pose_service' has already been
    // added to an executor." → 第一次 BT tick 进 handler 就 SIGABRT。
    // 正确做法:把 client 挂到独立 group,handler 里用本地
    // SingleThreadedExecutor 只 add_callback_group(ik_group_, node_base_)
    // —— 同一个 node 同一时刻不同 group 可属不同 executor,不冲突。
    ik_cb_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    ik_client_ = this->create_client<moveit_msgs::srv::GetPositionIK>(
      "/compute_ik", rmw_qos_profile_services_default, ik_cb_group_);

    // 自己提供的服务
    service_ = this->create_service<
      lgw_bt_planner::srv::SelectArmObservationPose>(
      "/select_arm_observation_pose",
      std::bind(
        &SelectArmObservationPoseServiceNode::handle_request, this, _1, _2));

    RCLCPP_INFO(this->get_logger(),
      "service up at /select_arm_observation_pose; "
      "/compute_ik %s",
      ik_client_->service_is_ready() ? "READY" : "NOT YET (will retry per request)");
  }

private:
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(js_mutex_);
    latest_joint_state_ = msg;
    latest_joint_state_stamp_ = this->now();
  }

  // Fill q_current with cached joint state. Returns false if cache empty
  // or stale (>2s old); service handler treats false as "skip
  // joint_displacement score".
  bool get_cached_q(std::vector<double>& q_out) const
  {
    std::lock_guard<std::mutex> lk(js_mutex_);
    if (!latest_joint_state_) return false;
    const double age =
      (this->now() - latest_joint_state_stamp_).seconds();
    if (age > kJointStateMaxAgeSec) return false;
    if (latest_joint_state_->position.empty()) return false;
    q_out = latest_joint_state_->position;
    return true;
  }

  // One synchronous /compute_ik call. Returns true on MoveIt SUCCESS and
  // populates q_solution_out with 6-dim joint positions.
  bool ik_check(const geometry_msgs::msg::Pose& pose,
                int timeout_ms,
                std::vector<double>& q_solution_out)
  {
    if (!ik_client_->service_is_ready()) {
      // move_group not up yet — treat all candidates unreachable this round.
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "ik_check: /compute_ik NOT READY");
      return false;
    }
    auto req_ik =
      std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
    req_ik->ik_request.group_name = "arm";
    req_ik->ik_request.ik_link_names = {"tcp_link"};
    req_ik->ik_request.pose_stamped.header.frame_id = "base_link";
    req_ik->ik_request.pose_stamped.pose = pose;
    req_ik->ik_request.timeout =
      rclcpp::Duration(std::chrono::milliseconds(timeout_ms));
    req_ik->ik_request.avoid_collisions = true;

    auto fut = ik_client_->async_send_request(req_ik);
    // 本地 executor 只 add_callback_group(ik_cb_group_),不 add_node;
    // 主线程的 rclcpp::spin(node) 通过 MutuallyExclusive 自动加进去的
    // 默认 group(给 service_)与本 ik_cb_group_ 是两个独立 group,可分属
    // 不同 executor,不会触发 "already added"。
    rclcpp::executors::SingleThreadedExecutor local_exec;
    local_exec.add_callback_group(
      ik_cb_group_, this->get_node_base_interface());
    const auto deadline = std::chrono::milliseconds(timeout_ms + 20);
    auto status = local_exec.spin_until_future_complete(fut, deadline);
    if (status != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "ik_check: spin_until_future_complete returned %d (NOT SUCCESS) "
        "for pos=(%.3f,%.3f,%.3f) timeout=%dms",
        static_cast<int>(status),
        pose.position.x, pose.position.y, pose.position.z, timeout_ms);
      return false;
    }
    auto resp_ik = fut.get();
    if (resp_ik->error_code.val !=
        moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "ik_check: IK FAIL pos=(%.3f,%.3f,%.3f) quat=(%.3f,%.3f,%.3f,%.3f) "
        "timeout=%dms error_code.val=%d",
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w,
        timeout_ms, resp_ik->error_code.val);
      return false;
    }
    q_solution_out = resp_ik->solution.joint_state.position;
    return true;
  }

  // Fill response with a single PoseCandidate (early-exit or final best).
  static void fill_response(
    lgw_bt_planner::srv::SelectArmObservationPose::Response& resp,
    const PoseCandidate& c,
    const char* tag)
  {
    resp.success = true;
    resp.selected_tcp_pose_in_world.header.frame_id = "base_link";
    resp.selected_tcp_pose_in_world.header.stamp = rclcpp::Time(0);
    resp.selected_tcp_pose_in_world.pose = c.tcp_pose;
    resp.selected_alpha_deg = c.alpha_rad * 180.0 / kPi;
    resp.selected_ratio = c.ratio;
    resp.message = tag;
  }

  void handle_request(
    const std::shared_ptr<
      lgw_bt_planner::srv::SelectArmObservationPose::Request> req,
    std::shared_ptr<
      lgw_bt_planner::srv::SelectArmObservationPose::Response> resp)
  {
    // Build config from request, with sane defaults if parsing fails.
    PoseSelector::Config cfg;
    if (!parse_csv_doubles(req->preferred_ratios_csv, cfg.ratios)) {
      cfg.ratios = {0.7, 0.85, 1.0, 1.2};
    }
    if (!parse_csv_doubles(req->preferred_alphas_deg_csv, cfg.alphas_deg)) {
      cfg.alphas_deg = {90.0, 60.0, 45.0};
    }
    cfg.joint_limit_fraction = 0.9;
    // Conservative Piper-H hard limits (radians). Sourced from
    // agx_arm_moveit/config/joint_limits.yaml in production setup; if
    // mismatched, the joint_limit scoring only mildly biases selection.
    cfg.q_max_per_joint = {
      3.141592653589793, 2.617993878, 3.141592653589793,
      1.832595715, 3.141592653589793, 3.141592653589793};

    const int timeout_ms = req->ik_call_timeout_ms > 0
      ? req->ik_call_timeout_ms
      : 50;

    // Read cached joint state (best-effort).
    std::vector<double> q_current;
    if (get_cached_q(q_current)) {
      cfg.q_current = q_current;
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "no recent /joint_states cache; skipping joint_displacement scoring");
    }

    PoseSelector sel(req->target_world, req->d_head, req->arm_base_frame,
                       R_a2t_, t_a2t_);
    auto candidates = sel.enumerate(cfg);
    if (candidates.empty()) {
      RCLCPP_WARN(this->get_logger(),
        "enumerate() returned 0 candidates "
        "(target on base z-axis? target=(%.3f, %.3f, %.3f) d_head=%.3f)",
        req->target_world.x, req->target_world.y, req->target_world.z,
        req->d_head);
      resp->success = false;
      resp->message = "no_candidates_enumerated";
      resp->n_reachable = 0;
      resp->n_total_iterated = 0;
      return;
    }

    int n_iterated = 0;
    int n_reachable = 0;

    for (auto& c : candidates) {
      std::vector<double> q;
      if (ik_check(c.tcp_pose, timeout_ms, q)) {
        c.ik_q_solution = std::move(q);
        ++n_reachable;
        sel.score(c, cfg);  // score now in case of early-exit response
        ++n_iterated;

        // Early-exit: (α=90°, r=0.7) → spec §4.6
        if (std::abs(c.alpha_rad - M_PI / 2.0) < kAlpha90_tolerance &&
            std::abs(c.ratio - 0.7) < kRatio07_tolerance) {
          resp->n_reachable = n_reachable;
          resp->n_total_iterated = n_iterated;
          fill_response(*resp, c, "early_exit_alpha90_r07");
          RCLCPP_INFO(this->get_logger(),
            "early-exit on (α=90°, r=0.7) after %d iters (%d reachable)",
            n_iterated, n_reachable);
          return;
        }
      } else {
        ++n_iterated;
      }
    }

    // Pool ran out — pick best from reachable side-effect
    auto best = sel.select_best(candidates, cfg);
    if (!best) {
      resp->success = false;
      resp->n_reachable = n_reachable;
      resp->n_total_iterated = n_iterated;
      resp->message = "no_reachable_candidate";
      RCLCPP_WARN(this->get_logger(),
        "all %zu candidates unreachable after %d iters (%d reachable)",
        candidates.size(), n_iterated, n_reachable);
      return;
    }

    resp->n_reachable = n_reachable;
    resp->n_total_iterated = n_iterated;
    fill_response(*resp, *best, "ok");
    RCLCPP_INFO(this->get_logger(),
      "selected α=%.1f° r=%.2f (disp=%.3f score=%.3f, %d/%d reachable, %d iters)",
      best->alpha_rad * 180.0 / kPi, best->ratio,
      best->joint_displacement_norm, best->score,
      n_reachable, static_cast<int>(candidates.size()), n_iterated);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
  rclcpp::Service<lgw_bt_planner::srv::SelectArmObservationPose>::SharedPtr service_;
  // 独立 callback group,见构造函数 2026-10-09 注释
  rclcpp::CallbackGroup::SharedPtr ik_cb_group_;

  mutable std::mutex js_mutex_;
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
  rclcpp::Time latest_joint_state_stamp_{0, 0, RCL_ROS_TIME};

  // Arm hand-eye calibration (camera→tcp). Loaded once in ctor from
  // arm_calibration.json so each candidate pose is the tcp_link pose
  // that places the arm camera at the designed observation position.
  tf2::Matrix3x3 R_a2t_;
  tf2::Vector3   t_a2t_;
};

}  // namespace lgw_bt_planner

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lgw_bt_planner::SelectArmObservationPoseServiceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
