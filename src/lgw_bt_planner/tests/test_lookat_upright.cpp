// tests/test_lookat_upright.cpp
// Pure tests for camera_upright(): the "画面不许上下倒转" hard guard used by
// both look-at solvers (2026-11-19).  No ROS runtime needed.

#include "lgw_bt_planner/lookat_solver.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace
{

using lgw_bt_planner::camera_upright;

// 正立基准: 光轴 u = +x(水平看向世界 +x)。按 optical 约定, 正立时
//   图像右向 x_cam = -y_w, 图像下向 y_cam = -z_w。
// 于是 R 的列: x = (0,-1,0), y = (0,0,-1), z = (1,0,0)。
tf2::Matrix3x3 upright_looking_along_x()
{
  return tf2::Matrix3x3(
    0.0, 0.0, 1.0,      // row0: x.x=0,  y.x=0,  z.x=1
    -1.0, 0.0, 0.0,     // row1: x.y=-1, y.y=0,  z.y=0
    0.0, -1.0, 0.0);    // row2: x.z=0,  y.z=-1, z.z=0
}

tf2::Matrix3x3 flipped_about_optical_axis(const tf2::Matrix3x3 & R)
{
  // 绕光轴(+z 列)滚 180 度: x 列和 y 列取反, z 列不变。
  const tf2::Vector3 x = -R.getColumn(0);
  const tf2::Vector3 y = -R.getColumn(1);
  const tf2::Vector3 z = R.getColumn(2);
  return tf2::Matrix3x3(
    x.x(), y.x(), z.x(),
    x.y(), y.y(), z.y(),
    x.z(), y.z(), z.z());
}

const tf2::Vector3 kCam(0.0, 0.0, 0.0);
const tf2::Vector3 kTarget(1.0, 0.0, 0.0);

TEST(LookatUpright, AcceptsUprightCamera)
{
  EXPECT_TRUE(camera_upright(kCam, kTarget, upright_looking_along_x()));
}

TEST(LookatUpright, RejectsCameraRolledBy180Deg)
{
  const auto rolled = flipped_about_optical_axis(upright_looking_along_x());
  EXPECT_FALSE(camera_upright(kCam, kTarget, rolled));
}

TEST(LookatUpright, MinCosTightensTheGuard)
{
  const auto R = upright_looking_along_x();
  EXPECT_TRUE(camera_upright(kCam, kTarget, R, 0.9));      // 完全正立
  // min_cos > 1 不可能满足 -> 证明阈值确实参与比较
  EXPECT_FALSE(camera_upright(kCam, kTarget, R, 1.01));
}

TEST(LookatUpright, SkipsCheckWhenLookIsVertical)
{
  // 光轴竖直向下: x_des 退化(roll 无定义), 任何滚角都视为通过。
  const auto rolled = flipped_about_optical_axis(upright_looking_along_x());
  const tf2::Vector3 straight_down(0.0, 0.0, 1.0);
  EXPECT_TRUE(camera_upright(kCam, straight_down, rolled));
  const tf2::Vector3 straight_up(0.0, 0.0, -1.0);
  EXPECT_TRUE(camera_upright(kCam, straight_up, rolled));
}

TEST(LookatUpright, DegenerateTargetAtCameraIsAccepted)
{
  const auto rolled = flipped_about_optical_axis(upright_looking_along_x());
  EXPECT_TRUE(camera_upright(kCam, kCam, rolled));
}

}  // namespace
