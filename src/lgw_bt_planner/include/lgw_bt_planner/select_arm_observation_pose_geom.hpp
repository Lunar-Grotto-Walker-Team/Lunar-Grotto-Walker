// select_arm_observation_pose_geom.hpp — Pure (no-rclcpp, no-ROS-runtime)
// geometry + scoring for self-selecting arm observation pose selection.
//
// This module has zero rclcpp dependencies; it only links against tf2
// (header-only quaternion/matrix math) and geometry_msgs (header-only
// message structs). gtest can call `enumerate` / `select_best` directly
// without spinning up an executor.
//
// Math (per candidate (α_rad, r)):
//   d_arm = r · d_head
//   u_xoy = normalize((Tx - Bx, Ty - By))             (assumes B = (0,0,0);
//                                                     arm_base_frame param
//                                                     reserved for future
//                                                     TF-aware variant)
//   h_disp = d_arm · cos(α)
//   v_disp = d_arm · sin(α)
//   cam.x  = Tx - h_disp · u_xoy.x                    (along base→target
//   cam.y  = Ty - h_disp · u_xoy.y                     extended line, target
//   cam.z  = Tz + v_disp                               outer end + raise)
//   forward = normalize(target − cam)                 (camera optical axis
//                                                     pointing toward T)
//   z_axis_cam = +forward                              (z toward target —
//                                                     match compute_lookat_pose_node
//                                                     convention so /compute_ik
//                                                     can solve)
//   x_axis_cam ≈ world_up × z_axis_cam
//   y_axis_cam = z_axis_cam × x_axis_cam
//   Uprightness flip: if cam_y · base_z > 0, swap columns 0 and 1 (keeps
//   det = +1 and z_axis_cam unchanged, but cam_y now points down — natural
//   camera-right-side-up pose).
//   R_cam_in_base = [x_cam | y_cam | z_cam]
//   R_tcp_in_base = R_cam_in_base · R_a2t⁻¹          (handeye: camera→tcp)
//   P_tcp_in_base = P_cam_in_base − R_tcp · t_a2t
//   quat = R_tcp → quaternion
//
// Hand-eye transform (R_a2t, t_a2t) is the camera→tcp rotation/translation
// loaded from arm_calibration.json. Default = identity (camera ≡ tcp), used
// by pure geometry unit tests; service_node passes the real calibration.
//
// Scoring (see compute_score()):
//   score(c) = w_d · joint_displacement_norm
//            + w_r · |r − r_nominal|
//            + w_α · |α_deg − α_pref| / 90
//            + w_lim · near_joint_limit(q)
//
// Used by:
//   - src/select_arm_observation_pose_service_node.cpp  (service handler)
//   - tests/test_select_arm_observation_pose_geom.cpp    (unit tests)

#pragma once

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <optional>
#include <string>
#include <vector>

namespace lgw_bt_planner
{

// tf2::Matrix3x3() and tf2::Vector3() default ctors leave members
// uninitialized (verified — they hold garbage doubles, NOT identity/zero).
// `inverse()` of a garbage matrix returns NaN, which propagates through
// the whole pose math. So whenever a "no-op" hand-eye transform is
// required (unit tests), pass these explicit identity/zero sentinels.
inline const tf2::Matrix3x3& identity_R_a2t()
{
  static const tf2::Matrix3x3 m(1, 0, 0,  0, 1, 0,  0, 0, 1);
  return m;
}
inline const tf2::Vector3& zero_t_a2t()
{
  static const tf2::Vector3 v(0, 0, 0);
  return v;
}

/// One candidate observation pose with metadata. Pre-IK fields populated
/// by `enumerate`; post-IK fields (`ik_q_solution`) populated by an external
/// IK caller. `score` is set by `score()`.
struct PoseCandidate
{
  geometry_msgs::msg::Pose tcp_pose;        //!< pose without header
  double alpha_rad = 0.0;                   //!< tilt angle from horizontal
  double ratio = 0.0;                       //!< d_arm = ratio · d_head
  std::vector<double> ik_q_solution;        //!< 6-dim; empty = IK failed
  double joint_displacement_norm = 0.0;     //!< ||q_solution − q_current||₂
  double score = 0.0;                       //!< lower = better
};

class PoseSelector
{
public:
  /// Tunable knobs. Defaults match spec §4.7.
  struct Config
  {
    // α candidates in degrees. Iteration order (enumerate) is the priority
    // order — earlier candidates are produced first and `select_best` then
    // picks among IK-reachable ones using these values as preferences.
    std::vector<double> alphas_deg = {90.0, 60.0, 45.0};

    // r candidates (= d_arm / d_head). Same priority semantics.
    std::vector<double> ratios = {0.7, 0.85, 1.0, 1.2};

    // Current 6-dim joint state. Empty ⇒ skip joint_displacement scoring
    // (still use r + α + joint_limit). Service-side subscription fills this
    // before calling score().
    std::vector<double> q_current;

    // Per-joint soft-limit fraction of full range. Real URDF limits per
    // joint are smaller than ±π; this default is conservative and only used
    // when q_max_per_joint is left at defaults.
    double joint_limit_fraction = 0.9;
    std::vector<double> q_max_per_joint = {
      3.141592653589793, 3.141592653589793, 3.141592653589793,
      3.141592653589793, 3.141592653589793, 3.141592653589793};

    // Scoring weights (default 1.0/2.0/1.0/1.0 per spec §4.7).
    double weight_displacement         = 1.0;
    double weight_ratio_deviation      = 2.0;
    double weight_alpha_deviation_deg  = 1.0; //!< divided by 90 inside score
    double weight_joint_limit          = 1.0;
  };

  /// Construct with target world-frame point, head→target distance, the
  /// arm base frame name (currently informational only; reserved for future
  /// TF-aware variant that queries base↔world), and the camera→tcp hand-eye
  /// transform loaded from arm_calibration.json (R_a2t rotates camera-frame
  /// vectors into tcp-frame vectors; t_a2t is camera origin in tcp frame).
  /// Defaults = identity, which collapses camera frame = tcp frame (only
  /// valid for unit tests; real use MUST pass the real calibration so
  /// /compute_ik solves for a tcp pose that actually places the camera at
  /// the designed position).
  PoseSelector(const geometry_msgs::msg::Point& target,
               double d_head,
               const std::string& arm_base_frame = "base_link",
               const tf2::Matrix3x3& R_a2t = identity_R_a2t(),
               const tf2::Vector3& t_a2t = zero_t_a2t());

  /// Member-form candidate pose computation. Uses stored
  /// `target_`/`d_head_`/`R_a2t_`/`t_a2t_`.
  geometry_msgs::msg::Pose compute_candidate_pose(
    double alpha_rad, double ratio) const;

  /// Static-form candidate pose computation. Useful for ad-hoc tests
  /// without an instance. Pass R_a2t/t_a2t to convert from camera-frame
  /// pose to tcp_link pose; defaults = identity (camera = tcp, only valid
  /// as a test fixture).
  static geometry_msgs::msg::Pose compute_candidate_pose_static(
    const geometry_msgs::msg::Point& target,
    double d_head,
    double alpha_rad,
    double ratio,
    const std::string& arm_base_frame,
    const tf2::Matrix3x3& R_a2t = identity_R_a2t(),
    const tf2::Vector3& t_a2t = zero_t_a2t());

  /// Build the candidate list in priority order (α outer, r inner) per spec
  /// §4.5. Does NOT populate `ik_q_solution`. Total size = `alphas.size() *
  /// ratios.size()`.
  std::vector<PoseCandidate> enumerate(const Config& cfg) const;

  /// Score one candidate (mutates score + joint_displacement_norm). Skips
  /// displacement term if `cfg.q_current` is empty.
  void score(PoseCandidate& candidate, const Config& cfg) const;

  /// Pick the lowest-score candidate from `candidates` whose
  /// `ik_q_solution` is non-empty (= IK reachable). Side effect: scores all
  /// reachable candidates. Returns std::nullopt if pool is empty.
  std::optional<PoseCandidate> select_best(
    std::vector<PoseCandidate>& candidates,
    const Config& cfg) const;

  /// Accessors for stored parameters (used by service handler to echo back
  /// debug info if needed).
  const geometry_msgs::msg::Point& target() const { return target_; }
  double d_head() const { return d_head_; }
  const std::string& arm_base_frame() const { return arm_base_frame_; }
  const tf2::Matrix3x3& R_a2t() const { return R_a2t_; }
  const tf2::Vector3& t_a2t() const { return t_a2t_; }

private:
  geometry_msgs::msg::Point target_;
  double d_head_;
  std::string arm_base_frame_;
  tf2::Matrix3x3 R_a2t_;   //!< camera→tcp rotation
  tf2::Vector3   t_a2t_;   //!< camera origin in tcp frame
};

}  // namespace lgw_bt_planner
