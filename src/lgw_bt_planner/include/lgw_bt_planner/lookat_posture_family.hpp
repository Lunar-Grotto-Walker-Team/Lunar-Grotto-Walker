// include/lgw_bt_planner/lookat_posture_family.hpp
//
// Pinned-posture lookat family (spec 2026-10-30 rev2, plan 2026-10-31).
//
// The legacy lookat rules pick a CAMERA POSITION and then solve a full 6-DOF
// pose for it, so nothing stops the chosen solution from sitting on the wrist
// singularity or against a joint limit — measured 2026-10-30: joint4 walked
// from 0.71 to 1.59 rad and halted Servo mid-task. This module inverts the
// problem: pin j3/j5 (posture), solve the remaining joints, and let j2 move
// only inside a guarded band.
//
// Public surface is tf2-only; the KDL/Eigen machinery stays behind a pimpl so
// including this header does not drag KDL into every translation unit.
//
// Frame contract: `cameraPose()` returns the CAMERA OPTICAL FRAME, not
// tcp_link. The caller composes the hand-eye calibration the same way
// compute_tcp_pose() does, so both lookat paths agree on the tcp frame.

#ifndef LGW_BT_PLANNER__LOOKAT_POSTURE_FAMILY_HPP_
#define LGW_BT_PLANNER__LOOKAT_POSTURE_FAMILY_HPP_

#include <array>
#include <memory>
#include <string>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include "lgw_bt_planner/lookat_solver.hpp"

namespace lgw_bt_planner
{

class PostureFamily : public LookatSolver
{
public:
  PostureFamily();
  ~PostureFamily() override;

  // LookatSolver: forwards to compute(); identical behaviour and status.
  PostureResult solve(
    const tf2::Vector3 & p_target,
    const tf2::Matrix3x3 & R_cam_in_tip,
    const tf2::Vector3 & t_cam_in_tip,
    const PostureParams & params) override;

  // Build the kinematic chain from a URDF file or from a URDF string (the
  // `robot_description` parameter). `tip_link` is tcp_link at runtime and
  // link6 in unit tests. Returns false and fills `error` on any mismatch —
  // a silently empty chain would produce a wrong camera pose every tick.
  bool initialize(
    const std::string & urdf_path_or_xml, bool is_file,
    const std::string & base_link, const std::string & tip_link,
    std::string & error);

  // Condition number of the full 6-joint chain at `joints` — the same metric
  // solveFree() reports for its own solutions. The QP solver needs it to
  // apply the identical cond_max guard after its own solve.
  bool cond(
    const std::array<double, 6> & joints,
    double & cond, std::string & error) const;

  // Forward kinematics: joints -> tip (link6 / tcp_link) pose in the base
  // link. Used by the QP solver to seed itself from the current pose.
  bool tipPose(
    const std::array<double, 6> & joints,
    tf2::Vector3 & p_tip, tf2::Matrix3x3 & R_tip,
    std::string & error) const;

  // TCP twist from measured joint velocities: (v, w) = J(q) * dq, both
  // expressed in the base frame, at the chain tip. Used by the servo control
  // law's damping term (the driver publishes clean 200 Hz joint velocities,
  // which is a far better source than differentiating the lagging TF pose).
  bool tcpTwist(
    const std::array<double, 6> & joints,
    const std::array<double, 6> & joint_vel,
    tf2::Vector3 & v, tf2::Vector3 & w, std::string & error) const;

  // Forward kinematics: joints -> camera optical frame expressed in the base
  // link. R_cam_in_tip / t_cam_in_tip are the hand-eye calibration.
  bool cameraPose(
    const std::array<double, 6> & joints,
    const tf2::Matrix3x3 & R_cam_in_tip,
    const tf2::Vector3 & t_cam_in_tip,
    tf2::Vector3 & p_cam, tf2::Matrix3x3 & R_cam,
    std::string & error) const;

  // Solve j1/j4/j6 so the camera looks at p_target with a level image, with
  // j2/j3/j5 held exactly at the seed's values. Damped least squares on a
  // 4-vector error (3 aim + weighted image roll), finite-difference Jacobian,
  // at most 8 iterations.
  //
  // The three joints are solved TOGETHER on purpose: j4 swings the optical
  // axis azimuth by up to 60 deg across its range, so "j1 for azimuth, j4 for
  // pitch" breaks the azimuth the moment j4 moves.
  bool solveFree(
    const tf2::Vector3 & p_target,
    const tf2::Matrix3x3 & R_cam_in_tip,
    const tf2::Vector3 & t_cam_in_tip,
    const std::array<double, 6> & seed,
    const PostureParams & params,
    std::array<double, 6> & q_out,
    double & aim_error_rad, double & cond,
    std::string & error) const;

  // Full solve: choose j2 from a fixed candidate order, run solveFree() for
  // each, and return the first pose that passes every guard (aim, condition
  // number, camera height, camera radius). Stateful on purpose — the previous
  // solution is both the warm start and the fallback.
  //
  // Throws std::runtime_error when the look direction is undefined (target on
  // the base z axis). Callers must treat that as HOLD, exactly like
  // compute_lookat_setpoint().
  PostureResult compute(
    const tf2::Vector3 & p_target,
    const tf2::Matrix3x3 & R_cam_in_tip,
    const tf2::Vector3 & t_cam_in_tip,
    const PostureParams & params);

private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
  // Last pose that satisfied every guard. Seeds the next solve and is what
  // gets republished when a tick cannot find a legal pose at all.
  bool have_previous_{false};
  PostureResult previous_;
};

}  // namespace lgw_bt_planner

#endif  // LGW_BT_PLANNER__LOOKAT_POSTURE_FAMILY_HPP_
