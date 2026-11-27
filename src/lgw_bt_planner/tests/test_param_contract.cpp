// tests/test_param_contract.cpp
//
// Task 4 of the 2026-11-07 node-split plan.
//
// Moving 111 parameters from hand-written declare_parameter() calls to a
// generate_parameter_library schema changes where every name and default
// lives. This file is the net under that move: it pins the set of names the
// node declares, so a rename or a dropped parameter fails here instead of
// showing up as a silently defaulted field on the robot.
//
// It also pins the two behaviours the schema adds: an out-of-range value is
// rejected at startup, and the documented sentinels (0 means "unlimited" /
// "disabled") still load.
//
// Names only. Defaults are covered by test_servo_params.cpp (shipped yaml) and
// by the solver tests.

#include <gtest/gtest.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "lgw_bt_planner/servo/servo_node.hpp"

namespace
{

// The full contract, in one list, because the schema declares every name in
// every mode. Before the migration the mode branches declared their own
// subsets, which is exactly the trap this test exists to catch: a params file
// that set lookat_solver without lookat_mode used to be silently ignored.
// (The modes themselves are gone as of 2026-11-10; the lessons are not.)
//
// ROS's own qos_overrides.* and use_sim_time are filtered out: they are not
// this node's contract.
const std::vector<std::string> kExpectedNames = {
  "arm_calibration_path", "arm_source_scale", "auto_resume_on_target",
  "auto_scan",
  "camera_tf_ang_tol_rad", "camera_tf_check_enable", "camera_tf_frame",
  "camera_tf_pos_tol_m",
  "deadband_ang_rad", "deadband_pos_m",
  "ff_deadband_mps", "ff_deadband_radps", "ff_enable", "ff_lowpass_hz",
  "ff_max_mps", "ff_max_radps", "follow_check_enabled",
  "follow_recovery_timeout_ms", "follow_reference_topic",
  "follow_resume_gap_rad", "follow_resume_stable_ms", "follow_timeout_ms",
  "follow_tolerance_rad", "frame_prefix",
  "hold_recovery_enable", "kd_ang", "kd_pos", "kp_ang", "kp_pos",
  "lookat_solver", "lowpass_cutoff_hz",
  "min_observe_distance_m",
  "pose_watch_topic", "posture_aim_tol_rad",
  "posture_cam_floor_distance_far_m", "posture_cam_floor_distance_near_m",
  "posture_cam_frontal_sector_rad", "posture_cam_frontal_taper_rad",
  "posture_cam_max_radius_m", "posture_cam_min_height_m",
  "posture_cam_min_radius_m", "posture_cond_max", "posture_j2_max_rad",
  "posture_j2_min_rad", "posture_j2_nominal_rad", "posture_j2_step_rad",
  "posture_j3_alt_rad", "posture_j3_rad", "posture_j5_alt_rad",
  "posture_j5_rad", "posture_joint_margin", "posture_max_overrun_us",
  "posture_trace", "preposition_accel_scaling", "preposition_branch_delta_rad",
  "preposition_enable", "preposition_joint_delta_rad",
  "preposition_limit_margin_rad", "preposition_tcp_angle_rad",
  "preposition_tcp_distance_m", "preposition_timeout_s",
  "preposition_tolerance_rad", "preposition_velocity_scaling",
  "qp_cond_max_relaxed", "qp_j5_branch_max_rad", "qp_j5_branch_min_rad",
  "qp_joint_margin_rad", "qp_max_linearizations", "qp_prefer_positive_branch",
  "qp_radius_near_m", "qp_radius_side_m", "qp_radius_steep_m",
  "qp_relaxed_min_azimuth_rad", "qp_w_aim", "qp_w_posture", "qp_w_radius",
  "qp_w_smooth",
  "recovery_accel_scaling", "recovery_allow_target_lost",
  "recovery_deadline_ms", "recovery_geometry_max_age_ms",
  "recovery_max_attempts", "recovery_presettle_gap_rad",
  "recovery_presettle_stable_ms", "recovery_presettle_timeout_ms",
  "recovery_retry_cooldown_ms", "recovery_settle_ms", "recovery_timeout_s",
  "recovery_tolerance_rad", "recovery_velocity_scaling", "robot_description",
  "scan_accel_scaling", "scan_dwell_ms", "scan_j1_max_rad",
  "scan_j1_min_rad", "scan_j1_step_rad", "scan_j2_raise_rad",
  "scan_j3_raise_rad", "scan_j5_gap_rad", "scan_j5_max_rad",
  "scan_j5_max_rad_pos", "scan_j5_min_rad", "scan_j5_min_rad_pos",
  "scan_j5_step_rad", "scan_max_attempts", "scan_require_base_static",
  "scan_retry_cooldown_s", "scan_start_delay_ms", "scan_timeout_s",
  "scan_tolerance_rad", "scan_velocity_scaling",
  "tcp_pose_domain", "tracker_stream_timeout_ms", "v_max",
  "v_max_total", "w_max", "w_max_total",
};

std::shared_ptr<lgw_bt_planner::ServoNode> make_node(
  const std::vector<rclcpp::Parameter> & overrides = {})
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  rclcpp::NodeOptions opts;
  if (!overrides.empty()) {
    opts.parameter_overrides(overrides);
  }
  return std::make_shared<lgw_bt_planner::ServoNode>(opts);
}

// ROS's own per-node parameters, not this node's contract.
bool is_infrastructure(const std::string & name)
{
  return name.rfind("qos_overrides.", 0) == 0 || name == "use_sim_time";
}

std::vector<std::string> declared_names(
  const std::shared_ptr<lgw_bt_planner::ServoNode> & node)
{
  std::vector<std::string> names;
  for (const auto & name : node->list_parameters({}, 0).names) {
    if (!is_infrastructure(name)) {
      names.push_back(name);
    }
  }
  std::sort(names.begin(), names.end());
  return names;
}

void expect_exactly(const std::vector<std::string> & names, const char * mode)
{
  std::vector<std::string> missing, added;
  for (const auto & want : kExpectedNames) {
    if (std::find(names.begin(), names.end(), want) == names.end()) {
      missing.push_back(want);
    }
  }
  for (const auto & got : names) {
    if (std::find(kExpectedNames.begin(), kExpectedNames.end(), got) ==
      kExpectedNames.end())
    {
      added.push_back(got);
    }
  }

  EXPECT_TRUE(missing.empty()) << mode << ": parameters removed: " <<
    (missing.empty() ? std::string("-") : missing[0]);
  EXPECT_TRUE(added.empty()) << mode << ": parameters added: " <<
    (added.empty() ? std::string("-") : added[0]);
  EXPECT_EQ(names.size(), kExpectedNames.size()) << mode;
}

}  // namespace

// Bootstrap: run with --gtest_filter='*Dump*' to print the current list.
TEST(ParamContract, DumpDeclaredNames)
{
  const std::vector<std::pair<std::string, std::vector<rclcpp::Parameter>>>
  modes = {
    {"dls", {}},
    {"qp", {rclcpp::Parameter("lookat_solver", "qp")}},
  };
  for (const auto & [label, overrides] : modes) {
    const auto node = make_node(overrides);
    const auto names = declared_names(node);
    std::printf("\n  [%s] %zu parameters:\n", label.c_str(), names.size());
    for (const auto & name : names) {
      std::printf("    \"%s\",\n", name.c_str());
    }
  }
  std::fflush(stdout);
  SUCCEED();
}

// The schema declares everything up front, so the set no longer depends on the
// mode. This is the whole point of the migration: a name can no longer exist
// in one mode's branch and be missing in another.
TEST(ParamContract, DeclaresExactlyTheDocumentedSet)
{
  expect_exactly(declared_names(make_node()), "default");
  // Both solvers read the same schema, so the set must not depend on the
  // choice either.
  expect_exactly(
    declared_names(make_node({rclcpp::Parameter("lookat_solver", "qp")})),
    "lookat_solver=qp");
}

// The schema's job: a value that cannot be right stops the node at startup,
// where an operator sees it, instead of being clamped or used as-is.
TEST(ParamContract, RejectsOutOfRangeKp)
{
  EXPECT_THROW(make_node({rclcpp::Parameter("kp_pos", -1.0)}), std::exception);
  EXPECT_THROW(make_node({rclcpp::Parameter("kp_ang", -0.001)}), std::exception);
}

// 0 is a DOCUMENTED sentinel, not an accident: v_max/w_max 0 = no clamp,
// recovery_max_attempts 0 = no cap, recovery_deadline_ms 0 = no deadline.
// A range check that rejects them would silently change recovery behaviour.
TEST(ParamContract, AcceptsDocumentedSentinels)
{
  EXPECT_NO_THROW(
    make_node({rclcpp::Parameter("v_max", 0.0),
      rclcpp::Parameter("w_max", 0.0),
      rclcpp::Parameter("recovery_max_attempts", 0),
      rclcpp::Parameter("recovery_deadline_ms", 0)}));
}

// The schema must not become the new authority for the ROBOT's tuning: the
// shipped yaml still wins, through the listener. If ParamListener ever stopped
// reading overrides (wrong node name, wrong file), every parameter would fall
// back to its schema default and the arm would run on neutral values -- this
// is the test that would notice.
TEST(ParamContract, ShippedYamlStillWins)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  const std::string yaml =
    ament_index_cpp::get_package_share_directory("lgw_bt_planner") +
    "/config/servo_node.yaml";
  auto node = std::make_shared<lgw_bt_planner::ServoNode>(
    rclcpp::NodeOptions().arguments({"--ros-args", "--params-file", yaml}));

  EXPECT_DOUBLE_EQ(node->get_parameter("kp_pos").as_double(), 3.0);
  EXPECT_DOUBLE_EQ(node->get_parameter("kp_ang").as_double(), 5.0);
  EXPECT_EQ(node->get_parameter("follow_timeout_ms").as_int(), 8000);
  EXPECT_EQ(node->get_parameter("qp_max_linearizations").as_int(), 12);
  // scan_period_s: schema 默认 8.0, 出厂 yaml 是 10.0 —— 用它证明 yaml 仍然
  // 覆盖 schema。原来这条用 posture_cam_min_radius_m(0.350), 但该值
  // 2026-11-16 被临时关成 0.0, 与 schema 默认相同, 失去区分力。
  EXPECT_DOUBLE_EQ(node->get_parameter("scan_period_s").as_double(), 10.0);
}
