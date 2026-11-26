#ifndef SIMPLE_LOOP_CLOSURE__GROUND_DIAGNOSTICS_HPP_
#define SIMPLE_LOOP_CLOSURE__GROUND_DIAGNOSTICS_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <random>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

namespace simple_loop_closure
{

// Local ground plane fitted from one keyframe cloud expressed in the
// Point-LIO IMU body frame (x forward, y left, z up).
//
// 2026-11-13: the first two revisions assumed the floor is a horizontal z
// (or y) band in the body frame.  That is false whenever the sensor is
// pitched relative to the floor - and it usually is: the L2 is mounted with
// a ~15 deg pitch and the LIO attitude drifts by another ~10 deg after a
// low-observability segment.  Measured on the corridor, the floor is a
// plane tilted ~15 deg in the body frame, so a z histogram smears it over
// ~2 m and only ~5% of frames produced a (wrong) fit.  The fit is now a
// deterministic constrained RANSAC plane: normals within 35 deg of vertical
// (up), sensor height 0.20-1.5 m, >= min_inliers.  On the corridor this
// recovers the floor (236 inliers, normal 11 deg off z, height 0.49 m).
//
// pitch_err/roll_err rotate the normal into the LIO world frame with the
// keyframe pose.  The world frame equals the initial IMU frame, so a flat
// floor with a correct pose gives ~0; a map/pose tilt shows up as a step.
struct GroundPlane
{
  bool valid{false};
  Eigen::Vector3d normal_body{Eigen::Vector3d(0.0, 0.0, 1.0)};
  Eigen::Vector3d normal_world{Eigen::Vector3d(0.0, 0.0, 1.0)};
  double height{0.0};          // sensor height above the plane, m
  double pitch_err_deg{0.0};   // world tilt about the left axis
  double roll_err_deg{0.0};    // world tilt about the forward axis
  std::size_t inliers{0};
};

// `band_height` is reused as the inlier slab thickness (tol = 0.2 *
// band_height, i.e. 6 cm with the default).  `world_gate` rejects fits whose
// world normal is >25 deg from vertical; it is for diagnostics only, because
// the estimator path must keep a keyframe whose current pose is tilted (that
// pose is exactly what needs correcting).
inline GroundPlane fitGroundPlane(
  const std::vector<Eigen::Vector3d> & points,
  const Eigen::Matrix3d & body_to_world,
  double band_height = 0.30,
  std::size_t min_inliers = 80,
  bool world_gate = true)
{
  GroundPlane out;
  if (points.size() < min_inliers) {
    return out;
  }

  // Deterministic subsample: RANSAC needs a bounded point count per keyframe.
  const std::size_t kMaxPoints = 600;
  const std::size_t stride = std::max<std::size_t>(1, points.size() / kMaxPoints);
  std::vector<Eigen::Vector3d> pts;
  pts.reserve(std::min(points.size(), kMaxPoints) + 1);
  for (std::size_t i = 0; i < points.size(); i += stride) {
    if (points[i].allFinite()) {
      pts.push_back(points[i]);
    }
  }
  if (pts.size() < min_inliers) {
    return out;
  }

  const double tol = std::max(0.03, 0.2 * band_height);
  const double min_up = 0.82;  // within 35 deg of vertical
  std::mt19937 rng(42u);
  std::uniform_int_distribution<std::size_t> pick(0, pts.size() - 1);

  std::size_t best = 0;
  Eigen::Vector3d best_n = Eigen::Vector3d::UnitZ();
  double best_d = 0.0;  // plane n.p = d
  for (int it = 0; it < 400; ++it) {
    const Eigen::Vector3d a = pts[pick(rng)];
    const Eigen::Vector3d b = pts[pick(rng)];
    const Eigen::Vector3d c = pts[pick(rng)];
    Eigen::Vector3d n = (b - a).cross(c - a);
    const double nn = n.norm();
    if (!(nn > 1e-9)) {
      continue;
    }
    n /= nn;
    if (n.z() < 0.0) {
      n = -n;  // up-normal
    }
    if (n.z() < min_up) {
      continue;
    }
    const double d = n.dot(a);
    const double h = -d;  // sensor is at the origin
    if (h < 0.20 || h > 1.5) {
      continue;
    }
    std::size_t cnt = 0;
    for (const auto & p : pts) {
      if (std::abs(n.dot(p) - d) < tol) {
        ++cnt;
      }
    }
    if (cnt > best) {
      best = cnt;
      best_n = n;
      best_d = d;
    }
  }
  if (best < min_inliers) {
    return out;
  }

  // Least-squares refinement over the winning plane's inliers.
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  std::size_t n_in = 0;
  for (const auto & p : pts) {
    if (std::abs(best_n.dot(p) - best_d) < tol) {
      centroid += p;
      ++n_in;
    }
  }
  if (n_in < min_inliers) {
    return out;
  }
  centroid /= static_cast<double>(n_in);

  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  for (const auto & p : pts) {
    if (std::abs(best_n.dot(p) - best_d) < tol) {
      const Eigen::Vector3d d = p - centroid;
      cov += d * d.transpose();
    }
  }
  cov /= static_cast<double>(n_in);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(cov);
  if (eig.info() != Eigen::Success) {
    return out;
  }
  Eigen::Vector3d normal = eig.eigenvectors().col(0).normalized();
  if (normal.z() < 0.0) {
    normal = -normal;
  }
  const double height = -normal.dot(centroid);
  if (height < 0.20 || height > 1.5) {
    return out;
  }
  if (normal.z() < min_up) {
    return out;
  }

  const Eigen::Vector3d nw = (body_to_world * normal).normalized();
  if (world_gate && nw.z() < 0.90) {
    return out;
  }

  out.valid = true;
  out.normal_body = normal;
  out.normal_world = nw;
  out.height = height;
  out.pitch_err_deg = std::atan2(-nw.x(), nw.z()) * 180.0 / M_PI;
  out.roll_err_deg = std::atan2(nw.y(), nw.z()) * 180.0 / M_PI;
  out.inliers = n_in;
  return out;
}

}  // namespace simple_loop_closure

#endif  // SIMPLE_LOOP_CLOSURE__GROUND_DIAGNOSTICS_HPP_
