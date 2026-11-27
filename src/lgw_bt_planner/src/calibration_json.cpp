// src/calibration_json.cpp
//
// Definition of parse_calibration_json(), moved out of
// compute_lookat_pose_node.cpp so it can live in the light `lgw_posture`
// library: that TU is heavy (rclcpp, BT) and the pure modules' tests must not
// drag it in. The declaration stays in compute_lookat_pose_node.hpp.

#include <cmath>
#include <fstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <nlohmann/json.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include "lgw_bt_planner/calibration_json.hpp"

namespace lgw_bt_planner
{

std::pair<tf2::Matrix3x3, tf2::Vector3> parse_calibration_json(
  const std::string& path)
{
  std::ifstream f(path);
  if (!f.is_open()) {
    throw std::runtime_error(
      "parse_calibration_json: cannot open " + path);
  }

  nlohmann::json j;
  try {
    f >> j;
  } catch (const std::exception& e) {
    throw std::runtime_error(
      std::string("parse_calibration_json: JSON parse error in ") +
      path + ": " + e.what());
  }

  // Required fields
  if (!j.contains("position") || !j.contains("orientation")) {
    throw std::runtime_error(
      "parse_calibration_json: missing 'position' or 'orientation' in " +
      path);
  }

  const auto& pos = j["position"];
  const auto& q   = j["orientation"];
  if (pos.size() != 3 || q.size() != 4) {
    throw std::runtime_error(
      "parse_calibration_json: position must be 3-vector, orientation 4-vector");
  }

  // Quaternion in file is xyzw (x, y, z, w) per
  // handeye_calibration_ros/handeye_calibration.py:42-51, 59 (ROS
  // geometry_msgs convention; tf_transformations.quaternion_from_matrix
  // returns [x, y, z, w]).
  const double qx = q[0].get<double>();
  const double qy = q[1].get<double>();
  const double qz = q[2].get<double>();
  const double qw = q[3].get<double>();

  const double qnorm = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
  if (qnorm < 1e-9) {
    throw std::runtime_error(
      "parse_calibration_json: quaternion norm ≈ 0 in " + path);
  }
  // Normalize so tf2::Matrix3x3 sees a unit quaternion.
  const double k = 1.0 / qnorm;

  tf2::Quaternion tfq;
  // tf2::Quaternion(x, y, z, w) constructor takes (x, y, z, w).
  tfq.setX(qx * k);
  tfq.setY(qy * k);
  tfq.setZ(qz * k);
  tfq.setW(qw * k);

  tf2::Matrix3x3 R(tfq);
  tf2::Vector3 t(pos[0].get<double>(), pos[1].get<double>(),
                 pos[2].get<double>());

  return {R, t};
}

}  // namespace lgw_bt_planner
