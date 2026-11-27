// tests/test_compute_lookat_pose.cpp
// Pure-function unit tests for ComputeLookatPose.
//
// No ROS runtime needed (math only).
//
// Run directly from build/lgw_bt_planner/test_compute_lookat_pose
// (colcon test not available in this container — see CLAUDE.md).

#include "lgw_bt_planner/compute_lookat_pose_node.hpp"

#include <gtest/gtest.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>

namespace
{
// Verify a 3x3 rotation matrix is a valid rotation: R*R^T = I,
// det(R) = +1.
void ExpectRotation(const tf2::Matrix3x3& R)
{
  tf2::Matrix3x3 rt = R.transpose();
  tf2::Matrix3x3 rr = R * rt;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      const double expected = (i == j) ? 1.0 : 0.0;
      EXPECT_NEAR(rr[i][j], expected, 1e-9)
        << "R*R^T[" << i << "][" << j << "]";
    }
  }
  EXPECT_NEAR(R.determinant(), 1.0, 1e-9);
}

// Write a JSON literal to a temp file and return its path. Caller
// must unlink().
std::string write_temp_json(const std::string& body)
{
  std::string path = std::string(std::tmpnam(nullptr)) + ".json";
  std::ofstream f(path);
  f << body;
  f.close();
  return path;
}
}  // namespace

// === parse_calibration_json tests (TC7-10) ==========================

// TC7: 合法 JSON → 数值正确解出,quat 模 = 1
TEST(ParseCalibration, ValidJsonParsesCorrectly)
{
  const std::string body = R"({
    "position":    [-0.076, -0.0005, -0.0815],
    "orientation": [-0.122, 0.123, -0.683, 0.709],
    "rpy":         [[-0.348, 0.0075, -1.534]]
  })";
  const std::string path = write_temp_json(body);

  auto [R, t] = lgw_bt_planner::parse_calibration_json(path);

  EXPECT_NEAR(t.x(), -0.076, 1e-6);
  EXPECT_NEAR(t.y(), -0.0005, 1e-6);
  EXPECT_NEAR(t.z(), -0.0815, 1e-6);
  ExpectRotation(R);

  std::remove(path.c_str());
}

// TC8: 缺 orientation 字段 → 抛异常
TEST(ParseCalibration, MissingOrientationThrows)
{
  const std::string body = R"({"position": [0.0, 0.0, 0.0]})";
  const std::string path = write_temp_json(body);

  EXPECT_THROW(
    lgw_bt_planner::parse_calibration_json(path),
    std::runtime_error);

  std::remove(path.c_str());
}

// TC9: orientation 模 = 0(全 0 四元数) → 抛异常
TEST(ParseCalibration, ZeroQuaternionThrows)
{
  const std::string body = R"({
    "position":    [0.0, 0.0, 0.0],
    "orientation": [0.0, 0.0, 0.0, 0.0]
  })";
  const std::string path = write_temp_json(body);

  EXPECT_THROW(
    lgw_bt_planner::parse_calibration_json(path),
    std::runtime_error);

  std::remove(path.c_str());
}

// TC10: 文件不存在 → 抛异常
TEST(ParseCalibration, NonexistentFileThrows)
{
  EXPECT_THROW(
    lgw_bt_planner::parse_calibration_json("/tmp/__no_such_calibration__.json"),
    std::runtime_error);
}

// === build_lookat_orientation tests (TC1-4) ========================

// TC1: direction = (0, 1, 0) (沿 +y), world_up = (0, 0, 1) →
//   z_axis = direction = (0, 1, 0)
//   x_axis = direction × world_up = (0,1,0) × (0,0,1)
//         = (1·1 - 0·0, 0·0 - 0·1, 0·0 - 1·0) = (1, 0, 0)
//   y_axis = z_axis × x_axis = (0,1,0) × (1,0,0)
//         = (1·0 - 0·0, 0·1 - 0·0, 0·0 - 1·1) = (0, 0, -1)
// (NOTE: 早期 TC1 用 direction=(0,0,-1)/world_up=(0,0,1) 是退化 case,
// 触发 world_x fallback,不是标准 case,Task 3 implementer 报告后已修)
TEST(BuildLookatOrientation, DirectionAlongYGivesCanonicalBasis)
{
  const tf2::Vector3 dir(0, 1, 0);
  const tf2::Vector3 up(0, 0, 1);

  tf2::Matrix3x3 R = lgw_bt_planner::build_lookat_orientation(dir, up);

  // tf2::Matrix3x3 element access via operator[](row) returns row-vector
  // of 3. R has columns [x_axis, y_axis, z_axis].
  tf2::Vector3 x_axis(R[0][0], R[1][0], R[2][0]);
  tf2::Vector3 y_axis(R[0][1], R[1][1], R[2][1]);
  tf2::Vector3 z_axis(R[0][2], R[1][2], R[2][2]);

  EXPECT_NEAR(x_axis.x(), 1.0, 1e-9);
  EXPECT_NEAR(x_axis.y(), 0.0, 1e-9);
  EXPECT_NEAR(x_axis.z(), 0.0, 1e-9);

  EXPECT_NEAR(y_axis.x(),  0.0, 1e-9);
  EXPECT_NEAR(y_axis.y(),  0.0, 1e-9);
  EXPECT_NEAR(y_axis.z(), -1.0, 1e-9);

  EXPECT_NEAR(z_axis.x(), 0.0, 1e-9);
  EXPECT_NEAR(z_axis.y(), 1.0, 1e-9);
  EXPECT_NEAR(z_axis.z(), 0.0, 1e-9);

  ExpectRotation(R);
}

// TC2: direction = (1, 0, 0), world_up = (0, 0, 1) →
//   x_axis = direction × world_up = (0·1 - 0·0, 0·0 - 1·1, 1·0 - 0·0)
//       = (0, -1, 0)
//   y_axis = z_axis × x_axis = (1,0,0) × (0,-1,0) = (0·0 - 0·(-1),
//                                                   0·0 - 1·0,
//                                                   1·(-1) - 0·0)
//       = (0, 0, -1)
TEST(BuildLookatOrientation, XDirectionYieldsOrthonormalBasis)
{
  const tf2::Vector3 dir(1, 0, 0);
  const tf2::Vector3 up(0, 0, 1);

  tf2::Matrix3x3 R = lgw_bt_planner::build_lookat_orientation(dir, up);

  tf2::Vector3 x_axis(R[0][0], R[1][0], R[2][0]);
  tf2::Vector3 y_axis(R[0][1], R[1][1], R[2][1]);
  tf2::Vector3 z_axis(R[0][2], R[1][2], R[2][2]);

  EXPECT_NEAR(x_axis.y(), -1.0, 1e-9);
  EXPECT_NEAR(y_axis.z(), -1.0, 1e-9);
  EXPECT_NEAR(z_axis.x(),  1.0, 1e-9);

  ExpectRotation(R);
}

// TC3: direction ≈ world_up (平行) → Gram-Schmidt 退化,fallback
// 用 world_x = (1, 0, 0)。验证 x_axis ≠ 0 向量且 R 仍合法旋转。
TEST(BuildLookatOrientation, DegenerateCaseUsesFallback)
{
  const tf2::Vector3 dir(0, 0, 1);   // 与 world_up 完全平行
  const tf2::Vector3 up(0, 0, 1);

  tf2::Matrix3x3 R = lgw_bt_planner::build_lookat_orientation(dir, up);

  tf2::Vector3 x_axis(R[0][0], R[1][0], R[2][0]);
  EXPECT_GT(x_axis.length(), 1e-6);  // 不是零向量
  ExpectRotation(R);
}

// TC4: direction 范数 0 → 抛异常(防除零)
TEST(BuildLookatOrientation, ZeroDirectionThrows)
{
  const tf2::Vector3 dir(0, 0, 0);
  const tf2::Vector3 up(0, 0, 1);

  EXPECT_THROW(
    lgw_bt_planner::build_lookat_orientation(dir, up),
    std::runtime_error);
}

// === compute_tcp_pose tests (TC5-6) ===============================

// TC5: R_arm_in_base = I, R_arm_in_tcp = I, t_arm_in_tcp = (0,0,-0.12),
//   P_ee = (0.2, 0, 0.3) →
//   R_tcp = I * I = I,
//   P_tcp = P_ee - I * (0,0,-0.12) = (0.2, 0, 0.3) - (0,0,-0.12)
//         = (0.2, 0, 0.42)
// 验证:R_tcp 是 identity 四元数(0,0,0,1),P_tcp = (0.2, 0, 0.42)。
TEST(ComputeTcpPose, IdentityCaseSolvesOffset)
{
  // tf2::Matrix3x3 default ctor leaves m_el[] uninitialized;
  // explicit setIdentity() is mandatory.
  tf2::Matrix3x3 R_arm_in_base;
  R_arm_in_base.setIdentity();
  tf2::Matrix3x3 R_arm_in_tcp;
  R_arm_in_tcp.setIdentity();
  tf2::Vector3 t_arm_in_tcp(0, 0, -0.12);
  tf2::Vector3 P_ee(0.2, 0, 0.3);

  auto pose = lgw_bt_planner::compute_tcp_pose(
    P_ee, R_arm_in_base, R_arm_in_tcp, t_arm_in_tcp);

  EXPECT_NEAR(pose.pose.position.x, 0.2,  1e-9);
  EXPECT_NEAR(pose.pose.position.y, 0.0,  1e-9);
  EXPECT_NEAR(pose.pose.position.z, 0.42, 1e-9);

  // orientation = identity → (x=0, y=0, z=0, w=1)
  EXPECT_NEAR(pose.pose.orientation.x, 0.0, 1e-9);
  EXPECT_NEAR(pose.pose.orientation.y, 0.0, 1e-9);
  EXPECT_NEAR(pose.pose.orientation.z, 0.0, 1e-9);
  EXPECT_NEAR(pose.pose.orientation.w, 1.0, 1e-9);
}

// TC6: R_arm_in_base = 绕 z 旋转 90°,R_arm_in_tcp = I,
//   t_arm_in_tcp = (0, 0.1, 0),P_ee = (0.5, 0, 0.3) →
//   R_tcp = R_arm_in_base (因为 R_arm_in_tcp = I),
//   P_tcp = P_ee - R_tcp * (0, 0.1, 0)
//   R_z(90°) 把 +y 转到 -x,所以 (0, 0.1, 0) → (-0.1, 0, 0)
//        = (0.5, 0, 0.3) - (-0.1, 0, 0)
//        = (0.6, 0, 0.3)
// 验证:R_tcp 四元数 = (0, 0, sin(45°), cos(45°)) = (0, 0, 0.7071, 0.7071)。
TEST(ComputeTcpPose, YRotatedBaseInvertsAtTcp)
{
  // R_arm_in_base = rotation 90° around z
  tf2::Matrix3x3 R_arm_in_base;
  R_arm_in_base.setRPY(0, 0, M_PI / 2);

  // tf2::Matrix3x3 default ctor leaves m_el[] uninitialized;
  // explicit setIdentity() is mandatory.
  tf2::Matrix3x3 R_arm_in_tcp;
  R_arm_in_tcp.setIdentity();
  tf2::Vector3 t_arm_in_tcp(0, 0.1, 0);
  tf2::Vector3 P_ee(0.5, 0, 0.3);

  auto pose = lgw_bt_planner::compute_tcp_pose(
    P_ee, R_arm_in_base, R_arm_in_tcp, t_arm_in_tcp);

  EXPECT_NEAR(pose.pose.position.x,  0.6, 1e-9);
  EXPECT_NEAR(pose.pose.position.y,  0.0, 1e-9);
  EXPECT_NEAR(pose.pose.position.z,  0.3, 1e-9);

  // orientation: 90° around z → (x=0, y=0, z=sin(45°), w=cos(45°))
  EXPECT_NEAR(pose.pose.orientation.x, 0.0,     1e-9);
  EXPECT_NEAR(pose.pose.orientation.y, 0.0,     1e-9);
  EXPECT_NEAR(pose.pose.orientation.z, M_SQRT1_2, 1e-9);
  EXPECT_NEAR(pose.pose.orientation.w, M_SQRT1_2, 1e-9);
}

