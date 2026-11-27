// tests/test_lookat_solver_select.cpp
//
// Solver interface + startup selection (plan 2026-11-05, Task 1). The solver
// is chosen once at startup; there is no runtime fallback.

#include <gtest/gtest.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <array>
#include <string>
#include <tuple>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

#include "lgw_bt_planner/calibration_json.hpp"
#include "lgw_bt_planner/lookat_posture_family.hpp"
#include "lgw_bt_planner/lookat_solver.hpp"

using lgw_bt_planner::LookatSolver;
using lgw_bt_planner::PostureFamily;
using lgw_bt_planner::PostureParams;
using lgw_bt_planner::SolverKind;

namespace
{

std::string piperUrdf()
{
  return ament_index_cpp::get_package_share_directory("agx_arm_description") +
         "/agx_arm_urdf/piper/urdf/piper_description.urdf";
}

std::pair<tf2::Matrix3x3, tf2::Vector3> camInLink6()
{
  tf2::Matrix3x3 R;
  tf2::Vector3 t;
  std::tie(R, t) = lgw_bt_planner::parse_calibration_json(
    ament_index_cpp::get_package_share_directory("lgw_bt_planner") +
    "/calibration/arm_calibration.json");
  return {R, t + tf2::Vector3(0.0, 0.0, 0.12)};
}

}  // namespace

TEST(SolverSelect, ParsesKnownValues)
{
  EXPECT_EQ(lgw_bt_planner::parse_solver_kind("qp", SolverKind::kDls),
    SolverKind::kQp);
  EXPECT_EQ(lgw_bt_planner::parse_solver_kind("dls", SolverKind::kQp),
    SolverKind::kDls);
}

TEST(SolverSelect, UnknownValueFallsBackToDls)
{
  EXPECT_EQ(lgw_bt_planner::parse_solver_kind("qpp", SolverKind::kDls),
    SolverKind::kDls);
  // An empty string is what an unset parameter looks like; the caller passes
  // kDls as the fallback, so the arm still starts.
  EXPECT_EQ(lgw_bt_planner::parse_solver_kind("", SolverKind::kDls),
    SolverKind::kDls);
}

TEST(SolverSelect, DlsImplementsTheInterface)
{
  PostureFamily fam;
  std::string err;
  ASSERT_TRUE(fam.initialize(piperUrdf(), /*is_file=*/true,
    "base_link", "link6", err)) << err;
  tf2::Matrix3x3 R;
  tf2::Vector3 t;
  std::tie(R, t) = camInLink6();

  LookatSolver * solver = &fam;
  const auto r = solver->solve(tf2::Vector3(0.75, -0.2, 0.0), R, t,
      PostureParams{});
  EXPECT_TRUE(r.ok) << "status=" << r.status;
  EXPECT_DOUBLE_EQ(r.joints[4], PostureParams{}.posture_j5_rad);
}
