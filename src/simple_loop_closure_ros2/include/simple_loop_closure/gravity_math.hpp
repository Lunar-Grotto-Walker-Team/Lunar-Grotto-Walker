#pragma once

#include <cmath>

#include <Eigen/Geometry>

namespace simple_loop_closure {

// Roll/pitch implied by a body-frame gravity vector.  The Point-LIO body
// frame is the /livox/imu frame (x forward, y left, z UP); roll is a
// rotation about x and pitch about y, matching gtsam::Rot3::rpy().  The
// vector must already be expressed in that body frame.
//
// 2026-11-13 correction: an earlier revision assumed the camera-style
// y-down frame.  Measured on data_20261111_132032, /livox/imu reads
// (0, 0, +1) g at rest and the registered cloud's floor plane is +z with
// the sensor 0.41 m above it, so the frame is standard z-up.
struct GravityAttitude {
  bool valid{false};
  Eigen::Vector3d g_body{Eigen::Vector3d(0.0, 0.0, 1.0)};  // specific force (up)
  double roll_deg{0.0};
  double pitch_deg{0.0};
};

inline GravityAttitude gravityToRollPitch(
    const Eigen::Vector3d& specific_force_body) {
  GravityAttitude out;
  const double n = specific_force_body.norm();
  if (!std::isfinite(n) || n < 1e-3) {
    return out;
  }
  out.g_body = specific_force_body / n;
  // g.z is the cosine of the tilt; <= 0.3 means >~72 deg from upright,
  // where roll/pitch are no longer a meaningful decomposition.
  if (out.g_body.z() <= 0.3) {
    return out;
  }
  out.roll_deg = std::atan2(out.g_body.y(), out.g_body.z()) * 180.0 / M_PI;
  out.pitch_deg = std::atan2(-out.g_body.x(), out.g_body.z()) * 180.0 / M_PI;
  out.valid = true;
  return out;
}

// When the source publishes an orientation, the LIO world frame equals the
// body frame at t0 (Point-LIO aligns the world to the initial IMU frame).
inline Eigen::Matrix3d extrinsicFromWorldAttitude(
    const Eigen::Quaterniond& base_to_world) {
  return base_to_world.toRotationMatrix();
}

// Accel-only sources: the extrinsic absorbs the mounting rotation, so the
// rest specific force maps to body up (+z).  The world vertical is then the
// frame Point-LIO initialized in; a non-level start shifts it and is the
// documented limitation of this path.
inline Eigen::Matrix3d extrinsicFromRestGravity(
    const Eigen::Vector3d& rest_specific_force) {
  const double n = rest_specific_force.norm();
  if (!std::isfinite(n) || n < 1e-3) {
    return Eigen::Matrix3d::Identity();
  }
  const Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(
      rest_specific_force / n, Eigen::Vector3d(0.0, 0.0, 1.0));
  return q.toRotationMatrix();
}

// autonomy_stack_go2 calibrate_imu.cpp transform for /utlidar/imu:
// y/z sign flip plus a 15.1 deg rotation about y.
inline Eigen::Vector3d utlidarAccelToZUp(const Eigen::Vector3d& raw) {
  const double theta = 15.1 * M_PI / 180.0;
  const double c = std::cos(theta);
  const double s = std::sin(theta);
  return Eigen::Vector3d(raw.x() * c + raw.z() * s, -raw.y(),
                         raw.x() * s - raw.z() * c);
}

// The calibrated /utlidar/imu accelerometer is in a gravity-DOWN convention:
// a standing robot reads (1.41, 0.05, -9.79) after the transform above, and
// calibrate_imu's acc_bias_z is computed as mean_z - 9.81.  The gravity prior
// needs the specific force (up), so negate.  Without this the utlidar source
// is silently rejected by the g_body.z() > 0.3 validity check.
inline Eigen::Vector3d utlidarSpecificForce(const Eigen::Vector3d& raw) {
  return -utlidarAccelToZUp(raw);
}

}  // namespace simple_loop_closure
