// src/compute_lookat_pose_node.cpp
// Pure lookat math + JSON parser (Task 2 complete; Tasks 3-5 fill the rest).

#include "lgw_bt_planner/compute_lookat_pose_node.hpp"

#include <nlohmann/json.hpp>

#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace lgw_bt_planner
{


tf2::Matrix3x3 build_lookat_orientation(
  const tf2::Vector3& direction,
  const tf2::Vector3& world_up)
{
  // Algorithm: spec §3 Step 3 (Gram-Schmidt with world_up fallback).
  //   raw_x = direction × world_up
  //   if ||raw_x|| < epsilon: raw_x = direction × world_x  (fallback)
  //   x_axis = raw_x.normalize()
  //   y_axis = direction × x_axis
  //   R = [x_axis | y_axis | direction]   (columns)
  //
  // Camera optical frame convention (ROS): z = forward, y = down,
  // x = right. Our direction = forward → z_axis = direction.

  const double dir_norm = direction.length();
  if (dir_norm < 1e-6) {
    throw std::runtime_error(
      "build_lookat_orientation: direction vector has zero norm");
  }

  tf2::Vector3 dir_unit = direction * (1.0 / dir_norm);

  tf2::Vector3 raw_x = dir_unit.cross(world_up);
  if (raw_x.length() < 1e-6) {
    // direction ≈ ±world_up: Gram-Schmidt degenerate, fall back to
    // world_x as orthogonalization reference.
    raw_x = dir_unit.cross(tf2::Vector3(1, 0, 0));
  }
  if (raw_x.length() < 1e-6) {
    throw std::runtime_error(
      "build_lookat_orientation: cannot construct orthogonal basis "
      "(direction parallel to both world_up and world_x)");
  }

  tf2::Vector3 x_axis = raw_x.normalized();
  tf2::Vector3 y_axis = dir_unit.cross(x_axis).normalized();

  // Build R with columns [x_axis, y_axis, z_axis=dir_unit].
  tf2::Matrix3x3 R;
  R.setValue(
    x_axis.x(),  y_axis.x(),  dir_unit.x(),
    x_axis.y(),  y_axis.y(),  dir_unit.y(),
    x_axis.z(),  y_axis.z(),  dir_unit.z());
  return R;
}

tf2::Matrix3x3 apply_uprightness_flip(
  const tf2::Matrix3x3& R,
  const tf2::Vector3& world_up)
{
  // Uprightness constraint: cam_y (column 1 of R, the camera optical +y
  // axis, which points "down" in the image) must have a negative
  // component along world_up. If not, the camera would look at the same
  // direction but be rolled 180° ("upside-down"), so rotate 180° about
  // the optical axis: negate cam_x (column 0) and cam_y (column 1),
  // leaving cam_z (column 2, = direction) untouched. det = (-1)(-1)(+1)
  // = +1, so the result is still a proper rotation.
  tf2::Matrix3x3 out = R;
  const tf2::Vector3 cam_y(out[0][1], out[1][1], out[2][1]);
  if (cam_y.dot(world_up) >= 0.0) {
    for (int i = 0; i < 3; ++i) {
      out[i][0] = -out[i][0];
      out[i][1] = -out[i][1];
    }
  }
  return out;
}

geometry_msgs::msg::PoseStamped compute_tcp_pose(
  const tf2::Vector3& P_ee_in_base,
  const tf2::Matrix3x3& R_arm_in_base,
  const tf2::Matrix3x3& R_arm_in_tcp,
  const tf2::Vector3& t_arm_in_tcp)
{
  // spec §3 Step 4-5:
  //   R_tcp_in_base = R_arm_in_base * inverse(R_arm_in_tcp)
  //   P_tcp_in_base = P_ee - R_tcp_in_base * t_arm_in_tcp
  //
  // The "inverse(R_arm_in_tcp)" is R_arm_in_tcp^T since R is a
  // rotation matrix; tf2 provides .inverse() that handles this.
  //
  // CONVENTION NOTE (2026-10-09): the arm_calibration.json data
  // actually represents T_camera -> tcp_link (R_a2t maps camera-frame
  // vectors to tcp-frame vectors; t_a2t is camera origin in tcp
  // frame). This is the inverse of what its _meta field used to
  // claim. The two formulas below happen to give the correct
  // tcp-in-base pose under BOTH conventions:
  //   * R_tcp_in_base = R_arm_in_base * R_arm_in_tcp.inverse():
  //     With T_camera->tcp, R_arm_in_tcp.inverse() rotates tcp->camera,
  //     and R_arm_in_base * (tcp->camera) composes tcp->base
  //     (tcp->camera->base via the arm frame). With T_tcp->camera,
  //     R_arm_in_tcp.inverse() rotates camera->tcp, and the same
  //     composition gives tcp->base (by the relation
  //     R_tcp = R_arm * R_a2t^(-1) ↔ R_arm = R_tcp * R_a2t).
  //   * P_tcp = P_ee - R_tcp * t_arm_in_tcp:
  //     Under T_camera->tcp, t_arm_in_tcp is "camera origin in tcp
  //     frame" (v_cam_in_tcp). P_ee is desired camera origin in base;
  //     tcp_origin_in_base = camera_origin_in_base + R_tcp * v, so
  //     P_tcp = P_ee - R_tcp * v. Under T_tcp->camera, t_arm_in_tcp
  //     is "tcp origin in camera frame" (v_tcp_in_cam) and
  //     v_tcp_in_cam = -R_a2t^(-1) * v_cam_in_tcp, so the same line
  //     reduces to P_tcp = P_ee + R_tcp * v_tcp_in_cam, which is the
  //     standard back-projection formula. The two interpretations
  //     yield the same P_tcp.
  //
  // The convention asymmetry only shows up in compute_arm_target_world
  // and verify_head_arm_consistency, where the same R_a2t / t_a2t
  // are used to project a target in the *opposite* direction
  // (camera -> tcp rather than tcp -> camera). Those call sites
  // therefore MUST apply R_a2t + t_a2t directly (no .inverse(),
  // no (P - t)) to remain consistent with the actual data. See
  // compute_arm_target_world.cpp file-top comment for the bug
  // history.
  tf2::Matrix3x3 R_tcp_in_base = R_arm_in_base * R_arm_in_tcp.inverse();
  tf2::Vector3 P_tcp_in_base =
    P_ee_in_base - R_tcp_in_base * t_arm_in_tcp;

  // Convert R_tcp_in_base to quaternion (x, y, z, w).
  tf2::Quaternion q;
  R_tcp_in_base.getRotation(q);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "";   // caller fills in
  pose.pose.position.x = P_tcp_in_base.x();
  pose.pose.position.y = P_tcp_in_base.y();
  pose.pose.position.z = P_tcp_in_base.z();
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();
  return pose;
}

}  // namespace lgw_bt_planner

// === BT node implementation (Task 6) ===============================

#include "lgw_bt_planner/compute_lookat_pose_node.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/logging.hpp>

#include <stdexcept>

namespace lgw_bt_planner
{

namespace
{
constexpr char kDefaultWorldFrame[] = "world";
constexpr char kDefaultOutputKey[]  = "lookat_pose";
constexpr char kDefaultEePosition[] = "0.5;0.0;0.3";
constexpr char kHeadCalibName[]     = "head_calibration.json";
constexpr char kArmCalibName[]      = "arm_calibration.json";

// Split "x;y;z" string into a Vector3.
tf2::Vector3 parse_semicolon_xyz(const std::string& s)
{
  tf2::Vector3 v;
  size_t p1 = s.find(';');
  size_t p2 = s.find(';', p1 + 1);
  if (p1 == std::string::npos || p2 == std::string::npos) {
    throw std::runtime_error(
      "parse_semicolon_xyz: expected 'x;y;z' format, got '" + s + "'");
  }
  v.setX(std::stod(s.substr(0, p1)));
  v.setY(std::stod(s.substr(p1 + 1, p2 - p1 - 1)));
  v.setZ(std::stod(s.substr(p2 + 1)));
  return v;
}

std::string resolve_default_calib_path(const char* filename)
{
  std::string share = ament_index_cpp::get_package_share_directory(
    "lgw_bt_planner");
  return share + "/calibration/" + filename;
}

// Pure-quaternion rotate-vector: v' = q ⊗ (0, v) ⊗ q*. Avoids
// any tf2::Matrix3x3 dependency in the diagnostic-bypass path. tf2
// doesn't expose a built-in rotate(Vector3) on Quaternion in Humble,
// so we implement the sandwich product manually.
tf2::Vector3 quat_rotate_vector(const tf2::Quaternion& q, const tf2::Vector3& v)
{
  tf2::Quaternion vq(v.x(), v.y(), v.z(), 0.0);
  tf2::Quaternion r = q * vq * q.inverse();
  return tf2::Vector3(r.x(), r.y(), r.z());
}
}  // namespace

ComputeLookatPoseNode::ComputeLookatPoseNode(
  const std::string& name,
  const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
{
  // Pull optional InputPorts for calibration paths. If not set OR empty,
  // fall back to the package share directory. (BT.CPP getInput returns
  // the port's declared default value when the XML omits the port; since
  // we declare default "" in providedPorts, p is always truthy and *p is
  // "" by default — without !empty() check, the fallback never runs and
  // parse_calibration_json("") throws "cannot open ".)
  std::string head_path, arm_path;
  if (auto p = getInput<std::string>("head_calibration_path"); p && !p->empty()) {
    head_path = *p;
  } else {
    head_path = resolve_default_calib_path(kHeadCalibName);
  }
  if (auto p = getInput<std::string>("arm_calibration_path"); p && !p->empty()) {
    arm_path = *p;
  } else {
    arm_path = resolve_default_calib_path(kArmCalibName);
  }

  try {
    std::tie(R_h2b_, t_h2b_) = parse_calibration_json(head_path);
    std::tie(R_a2t_, t_a2t_) = parse_calibration_json(arm_path);
    initialized_ = true;
    RCLCPP_INFO(rclcpp::get_logger("ComputeLookatPose"),
      "Loaded calibration: head=%s arm=%s", head_path.c_str(),
      arm_path.c_str());
    // Print the actual R / t loaded from JSON so downstream debugging can
    // cross-check the math against the stored hand-eye values (R_h2b /
    // R_a2t quaternion order, t signs, etc.). Printed unconditionally —
    // happens once per BT tree instantiation, not per tick.
    {
      tf2::Quaternion qh, qa;
      R_h2b_.getRotation(qh);
      R_a2t_.getRotation(qa);
      RCLCPP_INFO(rclcpp::get_logger("ComputeLookatPose"),
        "calib R_h2b: t=(%.4f, %.4f, %.4f) quat_xyzw=(%.4f, %.4f, %.4f, %.4f)",
        t_h2b_.x(), t_h2b_.y(), t_h2b_.z(),
        qh.x(), qh.y(), qh.z(), qh.w());
      RCLCPP_INFO(rclcpp::get_logger("ComputeLookatPose"),
        "calib R_a2t: t=(%.4f, %.4f, %.4f) quat_xyzw=(%.4f, %.4f, %.4f, %.4f)",
        t_a2t_.x(), t_a2t_.y(), t_a2t_.z(),
        qa.x(), qa.y(), qa.z(), qa.w());
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("ComputeLookatPose"),
      "Calibration load failed (%s); node will return FAILURE forever",
      e.what());
    initialized_ = false;
  }
}

BT::PortsList ComputeLookatPoseNode::providedPorts()
{
  return {
    BT::InputPort<yoloe_interfaces::msg::ObservationResult>(
      "head_observation", "Last CONFIRMED head ObservationResult"),
    BT::InputPort<std::string>(
      "ee_position_world", kDefaultEePosition,
      "Desired arm-camera-origin position in world; 'x;y;z'"),
    BT::InputPort<std::string>(
      "world_frame", kDefaultWorldFrame,
      "frame_id for output PoseStamped (also base for ee_position)"),
    BT::InputPort<std::string>(
      "head_calibration_path", "",
      "Path to head handeye JSON; empty = default share/.../head_calibration.json"),
    BT::InputPort<std::string>(
      "arm_calibration_path", "",
      "Path to arm handeye JSON (must be _tcp.json variant); "
      "empty = default share/.../arm_calibration.json"),
    BT::InputPort<std::string>(
      "output_pose_key", "",
      "Blackboard key override (default 'lookat_pose')"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>(
      "lookat_pose",
      "Computed tcp_link target pose; also written to blackboard"),
    BT::InputPort<std::string>(
      "target_in_head_frame", "",
      "Isolation port: 'x;y;z' in head optical frame. When non-empty, "
      "bypasses head_observation and synthesizes a fake CONFIRMED obs at "
      "that vector. For isolated test scripts only "
      "(start_lgw_lookat_pose_isolated_test.sh)."),
    BT::InputPort<std::string>(
      "xoy_distance_m", "",
      "Target-driven xoy orbit: when non-empty, P_ee is placed on a "
      "circle of radius this value around base_link origin, in the "
      "direction of (P_target.x, P_target.y). Empty = fall back to "
      "ee_position_world (legacy anchor path)."),
    BT::InputPort<std::string>(
      "z_fixed_m", "0.2",
      "When xoy_distance_m is set, P_ee.z = parse(this). Ignored when "
      "xoy_distance_m is empty."),
  };
}

BT::NodeStatus ComputeLookatPoseNode::tick()
{
  if (!initialized_) {
    return BT::NodeStatus::FAILURE;
  }

  // 1. Read head observation — OR use isolation hardcoded target.
  //    target_in_head_frame (InputPort, "x;y;z" in head optical frame):
  //    when non-empty, bypass head_observation and synthesize a fake
  //    CONFIRMED obs at that vector. Used by isolated test scripts
  //    (start_lgw_lookat_pose_isolated_test.sh) to verify MoveArmToPose
  //    IK + lookat math without YOLOE in the loop.
  yoloe_interfaces::msg::ObservationResult obs;
  if (auto p = getInput<std::string>("target_in_head_frame");
      p && !p->empty())
  {
    try {
      tf2::Vector3 pt = parse_semicolon_xyz(*p);
      yoloe_interfaces::msg::YoloeDetection3D det;
      det.center_m.x = pt.x();
      det.center_m.y = pt.y();
      det.center_m.z = pt.z();
      obs.detections.push_back(det);
      obs.status = yoloe_interfaces::msg::ObservationResult::CONFIRMED;
      obs.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
      RCLCPP_INFO(rclcpp::get_logger("ComputeLookatPose"),
        "Isolation mode: target_in_head_frame=(%.4f, %.4f, %.4f)",
        pt.x(), pt.y(), pt.z());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("ComputeLookatPose"),
        "Bad target_in_head_frame '%s': %s", p->c_str(), e.what());
      return BT::NodeStatus::FAILURE;
    }
  } else {
    auto obs_in = getInput<yoloe_interfaces::msg::ObservationResult>(
      "head_observation");
    if (!obs_in || obs_in->detections.empty()) {
      return BT::NodeStatus::FAILURE;
    }
    obs = *obs_in;
  }
  const auto& det = obs.detections.front();

  // 2. Read ee_position + world_frame + (optional) xoy-orbit params.
  std::string ee_str = kDefaultEePosition;
  if (auto p = getInput<std::string>("ee_position_world"); p) {
    ee_str = *p;
  }
  std::string world_frame = kDefaultWorldFrame;
  if (auto p = getInput<std::string>("world_frame"); p) {
    world_frame = *p;
  }
  std::string xoy_dist_str;
  if (auto p = getInput<std::string>("xoy_distance_m"); p && !p->empty()) {
    xoy_dist_str = *p;
  }
  std::string z_fixed_str = "0.2";
  if (auto p = getInput<std::string>("z_fixed_m"); p && !p->empty()) {
    z_fixed_str = *p;
  }

  // 3. Step 1: target in base.
  tf2::Vector3 P_target_head(det.center_m.x, det.center_m.y,
                             det.center_m.z);
  tf2::Vector3 P_target_base = R_h2b_ * P_target_head + t_h2b_;
  // R_h2b @ P_target_head (intermediate, useful when verifying the hand-eye
  // math independently of the translation).
  tf2::Vector3 Rh_p = R_h2b_ * P_target_head;
  RCLCPP_INFO(rclcpp::get_logger("ComputeLookatPose"),
    "step1 target_in_head=(%.4f, %.4f, %.4f)  R_h2b*pt=(%.4f, %.4f, %.4f)  "
    "+t_h2b -> P_target_base=(%.4f, %.4f, %.4f)",
    P_target_head.x(), P_target_head.y(), P_target_head.z(),
    Rh_p.x(), Rh_p.y(), Rh_p.z(),
    P_target_base.x(), P_target_base.y(), P_target_base.z());

  // 3.5 Step 1.5: pick P_ee_in_base.
  //   * xoy-orbit (target-driven): P_ee on circle of radius d_xoy in the
  //     direction of (P_target.x, P_target.y), at fixed z. Used by
  //     head_arm_independent_lookat.xml to follow the target horizontally
  //     while keeping cam height stable.
  //   * legacy anchor (ee_position_world): fixed "where should the EE be"
  //     point. Used by the other 3 BT trees.
  //   r_xy < epsilon means target is on the z-axis (xoy direction
  //   undefined); fall back to legacy anchor in that case.
  tf2::Vector3 P_ee_in_base;
  if (xoy_dist_str.empty()) {
    try {
      P_ee_in_base = parse_semicolon_xyz(ee_str);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("ComputeLookatPose"),
        "Bad ee_position_world '%s': %s", ee_str.c_str(), e.what());
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(rclcpp::get_logger("ComputeLookatPose"),
      "step1.5 P_ee from ee_position_world (legacy): "
      "(%.4f, %.4f, %.4f)",
      P_ee_in_base.x(), P_ee_in_base.y(), P_ee_in_base.z());
  } else {
    double d_xoy = 0.0;
    double z_fixed = 0.2;
    bool parse_ok = true;
    try {
      d_xoy = std::stod(xoy_dist_str);
      z_fixed = std::stod(z_fixed_str);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("ComputeLookatPose"),
        "Bad xoy_distance_m='%s' or z_fixed_m='%s': %s; falling back to "
        "ee_position_world anchor", xoy_dist_str.c_str(),
        z_fixed_str.c_str(), e.what());
      parse_ok = false;
    }
    if (parse_ok && d_xoy <= 0.0) {
      RCLCPP_ERROR(rclcpp::get_logger("ComputeLookatPose"),
        "xoy_distance_m must be > 0, got %.4f; falling back to anchor",
        d_xoy);
      parse_ok = false;
    }
    if (!parse_ok) {
      try {
        P_ee_in_base = parse_semicolon_xyz(ee_str);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("ComputeLookatPose"),
          "Bad ee_position_world '%s': %s", ee_str.c_str(), e.what());
        P_ee_in_base = tf2::Vector3(0, 0, 0);
      }
      RCLCPP_INFO(rclcpp::get_logger("ComputeLookatPose"),
        "step1.5 P_ee from anchor (fallback): (%.4f, %.4f, %.4f)",
        P_ee_in_base.x(), P_ee_in_base.y(), P_ee_in_base.z());
    } else {
      const double r_xy = std::sqrt(
        P_target_base.x() * P_target_base.x() +
        P_target_base.y() * P_target_base.y());
      if (r_xy < 1e-3) {
        RCLCPP_WARN(rclcpp::get_logger("ComputeLookatPose"),
          "step1.5 r_xy=%.6f (target on z-axis, xoy direction undefined); "
          "falling back to ee_position_world anchor", r_xy);
        try {
          P_ee_in_base = parse_semicolon_xyz(ee_str);
        } catch (const std::exception& e) {
          RCLCPP_ERROR(rclcpp::get_logger("ComputeLookatPose"),
            "Bad ee_position_world '%s': %s", ee_str.c_str(), e.what());
          P_ee_in_base = tf2::Vector3(0, 0, 0);
        }
      } else {
        const double ux = P_target_base.x() / r_xy;
        const double uy = P_target_base.y() / r_xy;
        P_ee_in_base.setX(d_xoy * ux);
        P_ee_in_base.setY(d_xoy * uy);
        P_ee_in_base.setZ(z_fixed);
        RCLCPP_INFO(rclcpp::get_logger("ComputeLookatPose"),
          "step1.5 xoy-orbit: d_xoy=%.4f dir=(%.3f,%.3f,0) z_fixed=%.4f "
          "-> P_ee=(%.4f, %.4f, %.4f)",
          d_xoy, ux, uy, z_fixed,
          P_ee_in_base.x(), P_ee_in_base.y(), P_ee_in_base.z());
      }
    }
  }

  // === Diagnostic bypass removed (2026-10-06) ===
  // Restored full lookat math (Step 2-3). Now applying the uprightness
  // constraint: cam_y · base_z must be < 0 (camera must point downward,
  // not upside-down). If build_lookat_orientation produces an R_arm_in_base
  // whose cam_y points upward, apply a 180° rotation around the direction
  // axis (= cam_z) to flip cam_x and cam_y columns. This keeps cam_z =
  // direction (camera still points at target) and preserves the right-
  // handed coordinate frame (det stays +1 since two columns flip).
  //
  //   4. Step 2: direction.
  tf2::Vector3 raw_dir = P_target_base - P_ee_in_base;
  if (raw_dir.length() < 1e-3) {
    RCLCPP_WARN(rclcpp::get_logger("ComputeLookatPose"),
      "target and ee_position coincide; geometric degeneracy");
    return BT::NodeStatus::FAILURE;
  }
  tf2::Vector3 direction = raw_dir.normalized();
  RCLCPP_INFO(rclcpp::get_logger("ComputeLookatPose"),
    "step2 raw_dir=(%.4f, %.4f, %.4f)  |raw|=%.4f  direction_unit=(%.4f, %.4f, %.4f)",
    raw_dir.x(), raw_dir.y(), raw_dir.z(), raw_dir.length(),
    direction.x(), direction.y(), direction.z());

  //   5. Step 3: build arm optical rotation in base + uprightness flip.
  tf2::Matrix3x3 R_arm_in_base;
  try {
    R_arm_in_base = build_lookat_orientation(direction,
                                             tf2::Vector3(0, 0, 1));
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("ComputeLookatPose"),
      "build_lookat_orientation failed: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
  // Uprightness check: cam_y (column 1 of R_arm_in_base) must point
  // downward (cam_y · base_z < 0, base_z = (0,0,1)). If not, flip
  // columns 0 and 1 (cam_x, cam_y); keep column 2 (= direction).
  // Shared with ComputeGraspPose + the Servo lookat-setpoint module via
  // apply_uprightness_flip().
  {
    const tf2::Vector3 base_z(0.0, 0.0, 1.0);
    tf2::Vector3 cam_y(R_arm_in_base[0][1], R_arm_in_base[1][1],
                       R_arm_in_base[2][1]);
    double cam_y_dot_base_z = cam_y.dot(base_z);
    R_arm_in_base = apply_uprightness_flip(R_arm_in_base, base_z);
    if (cam_y_dot_base_z >= 0.0) {
      RCLCPP_INFO(rclcpp::get_logger("ComputeLookatPose"),
        "step3 UPRIGHTNESS FLIP: cam_y·base_z=%.4f (>=0, camera was "
        "upside-down). Applying 180° rotation around direction axis "
        "to flip cam_y downward.",
        cam_y_dot_base_z);
      tf2::Vector3 cam_y_post(R_arm_in_base[0][1], R_arm_in_base[1][1],
                              R_arm_in_base[2][1]);
      RCLCPP_INFO(rclcpp::get_logger("ComputeLookatPose"),
        "step3 after flip: cam_y·base_z=%.4f (should now be <0)",
        cam_y_post.dot(base_z));
    } else {
      RCLCPP_INFO(rclcpp::get_logger("ComputeLookatPose"),
        "step3 no flip needed: cam_y·base_z=%.4f (already <0)",
        cam_y_dot_base_z);
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("ComputeLookatPose"),
    "step3 R_arm_in_base col_x=(%.4f, %.4f, %.4f) col_y=(%.4f, %.4f, %.4f) "
    "col_z=(%.4f, %.4f, %.4f)",
    R_arm_in_base[0][0], R_arm_in_base[1][0], R_arm_in_base[2][0],
    R_arm_in_base[0][1], R_arm_in_base[1][1], R_arm_in_base[2][1],
    R_arm_in_base[0][2], R_arm_in_base[1][2], R_arm_in_base[2][2]);

  //   6. Step 4-5: compute tcp pose via quaternion-only chain
  //      (Hamilton multiplication + sandwich-product rotate, no
  //      Matrix3x3 inverse, no matrix*vector).
  tf2::Quaternion q_arm_in_base, q_a2t;
  R_arm_in_base.getRotation(q_arm_in_base);
  R_a2t_.getRotation(q_a2t);

  tf2::Quaternion q_tcp_in_base = q_arm_in_base * q_a2t.inverse();
  tf2::Vector3 P_tcp_in_base =
    P_ee_in_base - quat_rotate_vector(q_tcp_in_base, t_a2t_);

  // Cross-check: q_cam_in_base = q_tcp ⊗ q_a2t; cam_z · direction ≈ 1
  // (round-trip), cam_y · base_z < 0 (uprightness preserved through chain).
  tf2::Quaternion q_cam_in_base = q_tcp_in_base * q_a2t;
  tf2::Matrix3x3 R_cam_for_check(q_cam_in_base);
  tf2::Vector3 cam_z_in_base(R_cam_for_check[0][2], R_cam_for_check[1][2],
                             R_cam_for_check[2][2]);
  tf2::Vector3 cam_y_in_base(R_cam_for_check[0][1], R_cam_for_check[1][1],
                             R_cam_for_check[2][1]);
  double cos_align = cam_z_in_base.dot(direction);
  double cam_y_dot_base_z_post = cam_y_in_base.dot(tf2::Vector3(0,0,1));

  RCLCPP_INFO(rclcpp::get_logger("ComputeLookatPose"),
    "step4 (quat-only) q_tcp_in_base xyzw=(%.4f, %.4f, %.4f, %.4f)  "
    "cam_z·direction=%.6f (should be 1.0)",
    q_tcp_in_base.x(), q_tcp_in_base.y(), q_tcp_in_base.z(), q_tcp_in_base.w(),
    cos_align);
  RCLCPP_INFO(rclcpp::get_logger("ComputeLookatPose"),
    "step5 (quat-only) P_tcp_in_base=(%.4f, %.4f, %.4f)  "
    "cam_y·base_z_post=%.4f (should be <0)",
    P_tcp_in_base.x(), P_tcp_in_base.y(), P_tcp_in_base.z(),
    cam_y_dot_base_z_post);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "";
  pose.pose.position.x = P_tcp_in_base.x();
  pose.pose.position.y = P_tcp_in_base.y();
  pose.pose.position.z = P_tcp_in_base.z();
  pose.pose.orientation.x = q_tcp_in_base.x();
  pose.pose.orientation.y = q_tcp_in_base.y();
  pose.pose.orientation.z = q_tcp_in_base.z();
  pose.pose.orientation.w = q_tcp_in_base.w();
  pose.header.frame_id = world_frame;
  pose.header.stamp = obs.header.stamp;

  // 7. Write blackboard + output port.
  std::string output_key = kDefaultOutputKey;
  if (auto p = getInput<std::string>("output_pose_key"); p && !p->empty()) {
    output_key = *p;
  }
  setOutput("lookat_pose", pose);
  config().blackboard->set(output_key, pose);

  // Optional debug logging (gated on BT_PLANNER_DEBUG=1, matching
  // MoveArmToPoseActionNode convention).
  if (const char* env = std::getenv("BT_PLANNER_DEBUG");
      env && std::string(env) == "1")
  {
    RCLCPP_INFO(rclcpp::get_logger("ComputeLookatPose"),
      "P_ee_in_base=(%.4f, %.4f, %.4f) P_target_base=(%.4f, %.4f, %.4f) "
      "P_tcp=(%.4f, %.4f, %.4f)",
      P_ee_in_base.x(), P_ee_in_base.y(), P_ee_in_base.z(),
      P_target_base.x(), P_target_base.y(), P_target_base.z(),
      pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  }
  return BT::NodeStatus::SUCCESS;
}

}  // namespace lgw_bt_planner
