#pragma once

#include <cmath>
#include <string>

#include <boost/make_shared.hpp>
#include <boost/optional.hpp>

#include <Eigen/Geometry>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace simple_loop_closure {

// 1-D world-z prior.  GTSAM Pose3::retract applies a *body-frame* update
// (T * Expmap(xi)), so d(world z)/d(Tx,Ty,Tz) is the third row of R, not
// [0,0,1].  The rotation block is zero.  (Only under identity rotation do
// the two coincide; numerically verified against GTSAM 4.2.)
class ZPriorFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  ZPriorFactor(gtsam::Key key, double measured_z,
               const gtsam::SharedNoiseModel& model)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, key),
        measured_z_(measured_z) {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose,
      boost::optional<gtsam::Matrix&> H = boost::none) const override {
    if (H) {
      gtsam::Matrix J(1, 6);
      J.setZero();
      J.block<1, 3>(0, 3) = pose.rotation().matrix().row(2);
      *H = J;
    }
    gtsam::Vector e(1);
    e(0) = pose.z() - measured_z_;
    return e;
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::make_shared<ZPriorFactor>(*this);
  }

 private:
  double measured_z_;
};

// 1-D relative-z factor: (z2 - z1) - measured_dz.  The arbitrary Go2 z
// origin cancels, so only the vertical *shape* is constrained.  This is
// much gentler than an absolute ZPriorFactor, which fights the trajectory's
// z datum and (measured on the corridor) distorts xy.
class ZDiffFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
 public:
  ZDiffFactor(gtsam::Key key1, gtsam::Key key2, double measured_dz,
              const gtsam::SharedNoiseModel& model)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(model, key1,
                                                            key2),
        measured_dz_(measured_dz) {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3& p1, const gtsam::Pose3& p2,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const override {
    if (H1) {
      gtsam::Matrix J(1, 6);
      J.setZero();
      J.block<1, 3>(0, 3) = -p1.rotation().matrix().row(2);
      *H1 = J;
    }
    if (H2) {
      gtsam::Matrix J(1, 6);
      J.setZero();
      J.block<1, 3>(0, 3) = p2.rotation().matrix().row(2);
      *H2 = J;
    }
    gtsam::Vector e(1);
    e(0) = (p2.z() - p1.z()) - measured_dz_;
    return e;
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::make_shared<ZDiffFactor>(*this);
  }

 private:
  double measured_dz_;
};

// 3-D world-frame translation difference: (p2 - p1) - measured_dp.
// The odometry edge's body-frame translation couples attitude corrections
// into world xy: gravity priors change R1, and R1 * dp_body rotates the
// whole subsequent path (measured: 5.9 m of xy bend on the corridor).
// Constraining the world difference directly keeps attitude corrections
// from moving positions; the rotation part is carried by a separate
// rotation-only BetweenFactor.  Jacobians follow Pose3::retract
// (p' = p + R v), same convention as ZDiffFactor.
class WorldTransDiffFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
 public:
  WorldTransDiffFactor(gtsam::Key key1, gtsam::Key key2,
                       const gtsam::Vector3& measured_dp,
                       const gtsam::SharedNoiseModel& model)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(model, key1,
                                                            key2),
        measured_dp_(measured_dp) {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3& p1, const gtsam::Pose3& p2,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const override {
    if (H1) {
      gtsam::Matrix J(3, 6);
      J.setZero();
      J.block<3, 3>(0, 3) = -p1.rotation().matrix();
      *H1 = J;
    }
    if (H2) {
      gtsam::Matrix J(3, 6);
      J.setZero();
      J.block<3, 3>(0, 3) = p2.rotation().matrix();
      *H2 = J;
    }
    return (p2.translation() - p1.translation()) - measured_dp_;
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::make_shared<WorldTransDiffFactor>(*this);
  }

 private:
  gtsam::Vector3 measured_dp_;
};

// True projected 3-DOF loop factor: residual [world yaw, world x, world y].
// z/roll/pitch never appear in the residual or the Jacobian, so a yaw or
// attitude correction cannot move world z even when the robot is pitched.
// GTSAM Pose3::retract is right-invariant (T * Expmap(xi)), so the world
// translation derivatives are R.topRows<2>() and the yaw derivative is the
// change of atan2(R10, R00) under a body-frame rotation increment.
class WorldXYPhiFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
 public:
  WorldXYPhiFactor(gtsam::Key key1, gtsam::Key key2,
                   const Eigen::Vector2d& measured_xy, double measured_yaw,
                   const gtsam::SharedNoiseModel& model)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(model, key1,
                                                            key2),
        measured_xy_(measured_xy),
        measured_yaw_(measured_yaw) {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3& p1, const gtsam::Pose3& p2,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const override {
    const gtsam::Matrix3 R1 = p1.rotation().matrix();
    const gtsam::Matrix3 R2 = p2.rotation().matrix();
    const double yaw1 = std::atan2(R1(1, 0), R1(0, 0));
    const double yaw2 = std::atan2(R2(1, 0), R2(0, 0));

    if (H1) {
      gtsam::Matrix J(3, 6);
      J.setZero();
      J.block<1, 3>(0, 0) = -yawJacobian(R1);
      J.block<2, 3>(1, 3) = -R1.topRows<2>();
      *H1 = J;
    }
    if (H2) {
      gtsam::Matrix J(3, 6);
      J.setZero();
      J.block<1, 3>(0, 0) = yawJacobian(R2);
      J.block<2, 3>(1, 3) = R2.topRows<2>();
      *H2 = J;
    }

    gtsam::Vector e(3);
    const double dyaw = yaw2 - yaw1 - measured_yaw_;
    e(0) = std::atan2(std::sin(dyaw), std::cos(dyaw));
    e(1) = (p2.x() - p1.x()) - measured_xy_(0);
    e(2) = (p2.y() - p1.y()) - measured_xy_(1);
    return e;
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::make_shared<WorldXYPhiFactor>(*this);
  }

 private:
  // d(atan2(R10, R00)) / d(omega) for a right-multiplied rotation increment.
  static Eigen::Matrix<double, 1, 3> yawJacobian(const gtsam::Matrix3& R) {
    const double den = R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0);
    Eigen::Matrix<double, 1, 3> J;
    J << 0.0, -(R(0, 0) * R(1, 2) - R(1, 0) * R(0, 2)) / den,
        (R(0, 0) * R(1, 1) - R(1, 0) * R(0, 1)) / den;
    return J;
  }

  Eigen::Vector2d measured_xy_;
  double measured_yaw_;
};

// Absolute 3-DOF (world xy + yaw) constraint on a single pose.  Prior-map
// registration yields the same measurement shape as a loop edge, but the
// target is a constant: expressing it as a unary factor on the query
// keyframe keeps `optimization_result_` dense (0..N-1) instead of adding a
// sparse prior node that every index-based traversal of the Values would
// trip over.  Residual order matches WorldXYPhiFactor: [yaw, x, y].
class WorldXYPhiPriorFactor
    : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  WorldXYPhiPriorFactor(gtsam::Key key, const Eigen::Vector2d& measured_xy,
                        double measured_yaw,
                        const gtsam::SharedNoiseModel& model)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, key),
        measured_xy_(measured_xy),
        measured_yaw_(measured_yaw) {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3& p,
      boost::optional<gtsam::Matrix&> H = boost::none) const override {
    const gtsam::Matrix3 R = p.rotation().matrix();
    const double yaw = std::atan2(R(1, 0), R(0, 0));

    if (H) {
      gtsam::Matrix J(3, 6);
      J.setZero();
      J.block<1, 3>(0, 0) = yawJacobian(R);
      J.block<2, 3>(1, 3) = R.topRows<2>();
      *H = J;
    }

    gtsam::Vector e(3);
    const double dyaw = yaw - measured_yaw_;
    e(0) = std::atan2(std::sin(dyaw), std::cos(dyaw));
    e(1) = p.x() - measured_xy_(0);
    e(2) = p.y() - measured_xy_(1);
    return e;
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::make_shared<WorldXYPhiPriorFactor>(*this);
  }

 private:
  // d(atan2(R10, R00)) / d(omega) for a right-multiplied rotation increment.
  static Eigen::Matrix<double, 1, 3> yawJacobian(const gtsam::Matrix3& R) {
    const double den = R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0);
    Eigen::Matrix<double, 1, 3> J;
    J << 0.0, -(R(0, 0) * R(1, 2) - R(1, 0) * R(0, 2)) / den,
        (R(0, 0) * R(1, 1) - R(1, 0) * R(0, 1)) / den;
    return J;
  }

  Eigen::Vector2d measured_xy_;
  double measured_yaw_;
};

// Relative floor-normal continuity: e = R1*n1 - R2*n2, where n1/n2 are the
// fitted up-normals in each keyframe's own body frame.  A common world yaw
// applied to both poses leaves the residual unchanged, so this constrains
// roll/pitch differences only - no yaw, z or translation information.  The
// residual is 3-D but the component along the normal is identically zero for
// unit normals; the isotropic sigma effectively weights the two tilt
// components.
class PoseNormalFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
 public:
  PoseNormalFactor(gtsam::Key key1, gtsam::Key key2,
                   const Eigen::Vector3d& n1_body,
                   const Eigen::Vector3d& n2_body,
                   const gtsam::SharedNoiseModel& model)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(model, key1,
                                                            key2),
        n1_(n1_body.normalized()),
        n2_(n2_body.normalized()) {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3& p1, const gtsam::Pose3& p2,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const override {
    const gtsam::Matrix3 R1 = p1.rotation().matrix();
    const gtsam::Matrix3 R2 = p2.rotation().matrix();
    if (H1) {
      gtsam::Matrix J(3, 6);
      J.setZero();
      J.block<3, 3>(0, 0) = -R1 * skew(n1_);
      *H1 = J;
    }
    if (H2) {
      gtsam::Matrix J(3, 6);
      J.setZero();
      J.block<3, 3>(0, 0) = R2 * skew(n2_);
      *H2 = J;
    }
    gtsam::Vector e(3);
    e = R1 * n1_ - R2 * n2_;
    return e;
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::make_shared<PoseNormalFactor>(*this);
  }

 private:
  static gtsam::Matrix3 skew(const Eigen::Vector3d& v) {
    gtsam::Matrix3 S;
    S << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
    return S;
  }

  Eigen::Vector3d n1_;
  Eigen::Vector3d n2_;
};

// The projected factor only carries 3 DOF, so it is valid only when neither
// z nor attitude is constrained.  Any other combination keeps the P0
// BetweenFactor<Pose3> path exactly as it was.
inline bool useProjected3Dof(const std::string& mode, bool constrain_z,
                             bool constrain_attitude) {
  return mode == "projected_3dof" && !constrain_z && !constrain_attitude;
}

// World-z continuity must not run together with either Go2 z constraint:
// - the attitude prior pulls roll/pitch toward a 2.5D attitude while the z
//   factor asks the same edges to preserve LIO's world z; on a 466 s corridor
//   the two together made the graph diverge by >30 m in xy;
// - the Go2 relative-z factor is a 2.5D body-height measurement, so on a
//   slope it flattens terrain, which is the failure the z factor prevents.
// Each is stable on its own profile.
inline bool useOdomZDiff(double var, bool attitude_prior_enabled,
                         bool go2_z_enabled, bool gravity_prior_enabled = false,
                         bool allow_gravity_coexist = false) {
  return var > 0.0 && !attitude_prior_enabled && !go2_z_enabled &&
         (!gravity_prior_enabled || allow_gravity_coexist);
}

// Leg z supersedes the LIO world-z continuity factor: both constrain the
// same keyframe pair, and running both double-weights the same direction.
inline bool useOdomZDiffWithLegZ(bool leg_z_enabled, double var,
                                 bool attitude_prior_enabled,
                                 bool go2_z_enabled,
                                 bool gravity_prior_enabled = false,
                                 bool allow_gravity_coexist = false) {
  return !leg_z_enabled &&
         useOdomZDiff(var, attitude_prior_enabled, go2_z_enabled,
                      gravity_prior_enabled, allow_gravity_coexist);
}

// The IMU gravity prior and the Go2 lidar attitude prior are two independent
// estimates of the same roll/pitch datum; running both double-constrains it
// and they fight through the shared rotation tangent.  The gravity prior
// wins when it is enabled.
inline bool useGo2Attitude(bool enable_go2_attitude,
                           bool enable_gravity_prior) {
  return enable_go2_attitude && !enable_gravity_prior;
}

// `robust_scale` is in whitened-residual units (residual / sigma).  The old
// hard-coded 1.0 treats anything beyond ~1 sigma as an outlier, which is too
// aggressive for the Go2 z difference: a fast gait accumulates several cm
// between 1 Hz optimisations, the factor gets down-weighted, and z drifts.
// 5.0 keeps normal inlier residuals active while still bounding real
// outliers (Go2 SLAM resets, dropped samples).
inline gtsam::SharedNoiseModel robustNoise1(double var, double robust_scale = 1.0) {
  return gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Cauchy::Create(robust_scale),
      gtsam::noiseModel::Diagonal::Variances(
          (gtsam::Vector(1) << var).finished()));
}

// Isotropic 3-D robust noise, for the pose-normal residual.
inline gtsam::SharedNoiseModel robustNoise3(double var,
                                            double robust_scale = 1.0) {
  return gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Cauchy::Create(robust_scale),
      gtsam::noiseModel::Isotropic::Variances(
          (gtsam::Vector(3) << var, var, var).finished()));
}

// Roll/pitch-only prior: free yaw and translation, pin attitude to the
// target pose.  Variance order is [Rx, Ry, Rz, Tx, Ty, Tz].
inline gtsam::SharedNoiseModel makeRollPitchPriorNoise(double rp_var,
                                                       double free_var,
                                                       double robust_scale = 1.0) {
  gtsam::Vector v(6);
  v << rp_var, rp_var, free_var, free_var, free_var, free_var;
  return gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Cauchy::Create(robust_scale),
      gtsam::noiseModel::Diagonal::Variances(v));
}

// Build the target pose for the roll/pitch prior: take roll/pitch from the
// Go2 lidar attitude (plus the calibrated constant offset) and keep the LIO
// yaw and translation.  The prior's yaw/translation have free variance, so
// only roll/pitch are affected.
inline gtsam::Pose3 targetPoseFromGo2Attitude(
    const gtsam::Pose3& lio_pose, const Eigen::Quaterniond& go2_q,
    double roll_offset, double pitch_offset) {
  const gtsam::Vector3 lio_rpy = lio_pose.rotation().rpy();
  const gtsam::Vector3 go2_rpy =
      gtsam::Rot3(go2_q.toRotationMatrix()).rpy();
  return gtsam::Pose3(
      gtsam::Rot3::Ypr(lio_rpy(2), go2_rpy(1) + pitch_offset,
                       go2_rpy(0) + roll_offset),
      lio_pose.translation());
}

// Target pose for the IMU gravity prior: roll/pitch come from the measured
// specific-force direction in the LIO body frame (z up), yaw and translation
// are taken from the current LIO estimate.  Because gtsam's rpy/ypr are the
// standard ZYX decomposition and the body frame is z-up, "gravity roll/pitch"
// and gtsam roll/pitch are the same quantities.  Offsets absorb a constant
// mounting/calibration bias.
inline gtsam::Pose3 targetPoseFromGravity(const gtsam::Pose3& lio_pose,
                                          const Eigen::Vector3d& up_body,
                                          double roll_offset_deg,
                                          double pitch_offset_deg) {
  const double n = up_body.norm();
  if (!std::isfinite(n) || n < 1e-3 || up_body.normalized().z() <= 0.3) {
    return lio_pose;  // invalid gravity: leave the pose untouched
  }
  const Eigen::Vector3d g = up_body / n;
  const double roll =
      (std::atan2(g.y(), g.z()) * 180.0 / M_PI) + roll_offset_deg;
  const double pitch =
      (std::atan2(-g.x(), g.z()) * 180.0 / M_PI) + pitch_offset_deg;
  const double lio_yaw = lio_pose.rotation().rpy()(2);
  return gtsam::Pose3(
      gtsam::Rot3::Ypr(lio_yaw, pitch * M_PI / 180.0, roll * M_PI / 180.0),
      lio_pose.translation());
}

}  // namespace simple_loop_closure
