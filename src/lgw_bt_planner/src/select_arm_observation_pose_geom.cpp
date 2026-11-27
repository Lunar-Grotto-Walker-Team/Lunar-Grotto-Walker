// select_arm_observation_pose_geom.cpp — see header for math and semantics.
// Pure module: no rclcpp / no BT.CPP / no MoveIt. Compiles into
// lgw_bt_planner_nodes SHARED lib (re-used by both BT plugin and
// service_node executable); also linkable from gtest without ROS runtime.

#include "lgw_bt_planner/select_arm_observation_pose_geom.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lgw_bt_planner
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;
constexpr double kRad2Deg = 180.0 / kPi;
constexpr double kUnitNormEpsilon = 1e-6;

double l2_norm(const std::vector<double>& a, const std::vector<double>& b)
{
  if (a.size() != b.size()) return std::numeric_limits<double>::infinity();
  double sq = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    const double d = a[i] - b[i];
    sq += d * d;
  }
  return std::sqrt(sq);
}

double near_joint_limit(const std::vector<double>& q,
                         const PoseSelector::Config& cfg)
{
  if (q.empty() || cfg.q_max_per_joint.empty()) return 0.0;
  const std::size_t n = std::min(q.size(), cfg.q_max_per_joint.size());
  double over = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    const double bound = cfg.joint_limit_fraction * cfg.q_max_per_joint[i];
    const double exc = std::abs(q[i]) - bound;
    if (exc > 0.0 && cfg.q_max_per_joint[i] > 0.0) {
      over += exc / cfg.q_max_per_joint[i];
    }
  }
  return over;
}
}  // namespace

PoseSelector::PoseSelector(
  const geometry_msgs::msg::Point& target,
  double d_head,
  const std::string& arm_base_frame,
  const tf2::Matrix3x3& R_a2t,
  const tf2::Vector3& t_a2t)
: target_(target),
  d_head_(d_head),
  arm_base_frame_(arm_base_frame),
  R_a2t_(R_a2t),
  t_a2t_(t_a2t)
{
}

geometry_msgs::msg::Pose PoseSelector::compute_candidate_pose_static(
  const geometry_msgs::msg::Point& target,
  double d_head,
  double alpha_rad,
  double ratio,
  const std::string& arm_base_frame,
  const tf2::Matrix3x3& R_a2t,
  const tf2::Vector3& t_a2t)
{
  // arm_base_frame: currently informational. Future TF-aware variant would
  // look up its origin; we assume (0,0,0) for now (matches `world →
  // base_link` identity fixed joint in this codebase's URDF — see spec
  // §3.3).
  (void)arm_base_frame;

  if (d_head < 0.0) {
    throw std::invalid_argument("PoseSelector: d_head < 0");
  }
  if (ratio < 0.0) {
    throw std::invalid_argument("PoseSelector: ratio < 0");
  }
  // (alpha_rad unbounded; negative is valid — it'd flip the camera)
  // (target is just (x,y,z) — no validation)

  const double Tx = target.x;
  const double Ty = target.y;
  const double Tz = target.z;

  // Unit vector in xoy plane (yaw direction).
  const double xoy_norm = std::hypot(Tx, Ty);
  if (xoy_norm < kUnitNormEpsilon) {
    throw std::runtime_error(
      "PoseSelector: target lies on arm_base z-axis (||xoy|| ≈ 0); "
      "cannot define yaw direction");
  }
  const double ux = Tx / xoy_norm;
  const double uy = Ty / xoy_norm;

  const double d_arm  = ratio * d_head;
  const double cos_a  = std::cos(alpha_rad);
  const double sin_a  = std::sin(alpha_rad);
  const double h_disp = d_arm * cos_a;
  const double v_disp = d_arm * sin_a;

  // === Camera-frame pose (where the camera should sit) ===
  geometry_msgs::msg::Pose pose;
  pose.position.x = Tx - h_disp * ux;
  pose.position.y = Ty - h_disp * uy;
  pose.position.z = Tz + v_disp;

  const tf2::Vector3 cam(pose.position.x, pose.position.y, pose.position.z);
  const tf2::Vector3 t_target(Tx, Ty, Tz);
  tf2::Vector3 forward = t_target - cam;
  const double fwd_len = forward.length();
  if (fwd_len < kUnitNormEpsilon) {
    // cam == target — IK will definitely fail, but we still emit the pose.
    // Use a fallback identity orientation so downstream doesn't choke on NaN.
    pose.orientation.x = 0.0; pose.orientation.y = 0.0;
    pose.orientation.z = 0.0; pose.orientation.w = 1.0;
    return pose;
  }
  forward /= fwd_len;

  // Camera optical axis convention: link +Z points toward target (this
  // matches compute_lookat_pose_node::build_lookat_orientation so /compute_ik
  // can solve; the gripper +Z is "out of the tool" so it also points toward
  // the target once we apply the handeye transform). Previously this code
  // used z_axis = -forward (REP-103 optical convention), which made IK
  // fail with -31 because the tcp_link +Z ended up pointing AWAY from
  // target. See hpp file-top comment for derivation.
  const tf2::Vector3 z_axis = forward;
  const tf2::Vector3 world_up(0.0, 0.0, 1.0);

  // When forward is near-parallel to world_up (top-down or bottom-up look),
  // use world X as a degenerate fallback to avoid cross-product degeneracy.
  tf2::Vector3 x_axis;
  if (std::abs(forward.dot(world_up)) > 0.999) {
    x_axis = tf2::Vector3(1.0, 0.0, 0.0);
    // Orthogonalize against z_axis if needed.
    x_axis = x_axis - z_axis * (x_axis.dot(z_axis));
    const double xlen = x_axis.length();
    if (xlen < kUnitNormEpsilon) {
      // Pure top-down with forward·up = ±1 — pick world Y as a final fallback
      x_axis = tf2::Vector3(0.0, 1.0, 0.0);
    } else {
      x_axis /= xlen;
    }
  } else {
    x_axis = z_axis.cross(world_up).normalized();
  }
  tf2::Vector3 y_axis = z_axis.cross(x_axis).normalized();
  // Re-orthogonalize x to be safe against numerical drift.
  x_axis = y_axis.cross(z_axis).normalized();

  // Uprightness flip: if cam_y points UP (cam_y · base_z > 0), the camera
  // is upside-down. Flip columns 0 and 1 (= swap cam_x and cam_y); this
  // preserves det = +1, keeps z_axis = +forward, but cam_y now points
  // down — natural "image right-side up" orientation. Mirrors
  // compute_lookat_pose_node.cpp uprightness check.
  if (y_axis.z() > 0.0) {
    const tf2::Vector3 tmp = x_axis;
    x_axis = y_axis;
    y_axis = tmp;
  }

  // Build R_cam_in_base with columns (x_axis, y_axis, z_axis). tf2::Matrix3x3
  // ctor takes row-major 9 elements: row_i = (x_i, y_i, z_i).
  tf2::Matrix3x3 R_cam(
    x_axis.x(), y_axis.x(), z_axis.x(),
    x_axis.y(), y_axis.y(), z_axis.y(),
    x_axis.z(), y_axis.z(), z_axis.z());

  // === Hand-eye transform: camera→tcp_link ===
  // R_tcp_in_base = R_cam_in_base · R_a2t⁻¹
  // P_tcp_in_base = P_cam_in_base − R_tcp · t_a2t
  // (R_a2t rotates camera-frame vectors into tcp-frame vectors; t_a2t is
  //  camera origin expressed in tcp frame. This is the same convention as
  //  compute_lookat_pose_node::compute_tcp_pose.)
  // For unit tests with identity handeye, R_tcp = R_cam and P_tcp = P_cam
  // (since t_a2t = 0), so the returned pose is the camera-frame pose.
  const tf2::Matrix3x3 R_a2t_inv = R_a2t.inverse();
  const tf2::Matrix3x3 R_tcp = R_cam * R_a2t_inv;
  // P_tcp = P_cam - R_tcp * t_a2t
  const tf2::Vector3 R_tcp_t_a2t(
    R_tcp[0][0] * t_a2t.x() + R_tcp[0][1] * t_a2t.y() + R_tcp[0][2] * t_a2t.z(),
    R_tcp[1][0] * t_a2t.x() + R_tcp[1][1] * t_a2t.y() + R_tcp[1][2] * t_a2t.z(),
    R_tcp[2][0] * t_a2t.x() + R_tcp[2][1] * t_a2t.y() + R_tcp[2][2] * t_a2t.z());
  pose.position.x -= R_tcp_t_a2t.x();
  pose.position.y -= R_tcp_t_a2t.y();
  pose.position.z -= R_tcp_t_a2t.z();

  tf2::Quaternion q;
  R_tcp.getRotation(q);
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

geometry_msgs::msg::Pose PoseSelector::compute_candidate_pose(
  double alpha_rad, double ratio) const
{
  return compute_candidate_pose_static(
    target_, d_head_, alpha_rad, ratio, arm_base_frame_, R_a2t_, t_a2t_);
}

std::vector<PoseCandidate> PoseSelector::enumerate(const Config& cfg) const
{
  std::vector<PoseCandidate> out;
  out.reserve(cfg.alphas_deg.size() * cfg.ratios.size());
  for (const double& alpha_deg : cfg.alphas_deg) {
    const double alpha_rad = alpha_deg * kDeg2Rad;
    for (const double& ratio : cfg.ratios) {
      PoseCandidate c;
      c.alpha_rad = alpha_rad;
      c.ratio = ratio;
      try {
        c.tcp_pose = compute_candidate_pose(alpha_rad, ratio);
      } catch (const std::exception&) {
        // Skip degenerate candidates (target on z-axis); service logs them.
        continue;
      }
      out.push_back(std::move(c));
    }
  }
  return out;
}

void PoseSelector::score(PoseCandidate& candidate, const Config& cfg) const
{
  // 1. joint displacement (only when q_current provided AND ik succeeded)
  if (!cfg.q_current.empty() && candidate.ik_q_solution.size() == cfg.q_current.size()) {
    candidate.joint_displacement_norm = l2_norm(
      candidate.ik_q_solution, cfg.q_current);
  } else {
    candidate.joint_displacement_norm = 0.0;
  }

  // 2. r deviation from nominal (cfg.ratios.front() = 0.7 by default)
  const double r_nominal = cfg.ratios.empty() ? 0.7 : cfg.ratios.front();
  const double dr = std::abs(candidate.ratio - r_nominal);

  // 3. α deviation from preference (cfg.alphas_deg.front() = 90 by default)
  const double a_pref = cfg.alphas_deg.empty() ? 90.0 : cfg.alphas_deg.front();
  const double da_deg = std::abs(candidate.alpha_rad * kRad2Deg - a_pref);

  // 4. near joint limit penalty
  const double lim = near_joint_limit(candidate.ik_q_solution, cfg);

  candidate.score =
      cfg.weight_displacement * candidate.joint_displacement_norm
    + cfg.weight_ratio_deviation * dr
    + cfg.weight_alpha_deviation_deg * (da_deg / 90.0)
    + cfg.weight_joint_limit * lim;
}

std::optional<PoseCandidate> PoseSelector::select_best(
  std::vector<PoseCandidate>& candidates, const Config& cfg) const
{
  std::optional<PoseCandidate> best;
  double best_score = std::numeric_limits<double>::infinity();
  for (auto& c : candidates) {
    if (c.ik_q_solution.empty()) continue;
    score(c, cfg);
    if (c.score < best_score) {
      best_score = c.score;
      best = c;
    }
  }
  return best;
}

}  // namespace lgw_bt_planner
