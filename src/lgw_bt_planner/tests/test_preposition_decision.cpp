// tests/test_preposition_decision.cpp
//
// Unit tests for the takeover pre-position criteria (spec 2026-10-30 §3.8
// rev3). Pure math — no ROS graph, no hardware.

#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include "lgw_bt_planner/preposition_decision.hpp"

using lgw_bt_planner::decide_preposition;
using lgw_bt_planner::Pose6;
using lgw_bt_planner::PrepositionDecision;
using lgw_bt_planner::PrepositionParams;

namespace
{

// The posture family's nominal solution for a ground target, measured on the
// real chain (scripts/lookat_posture_proto.py).
const std::array<double, 6> kFamily{-0.48, 0.90, -0.484, -1.10, 0.600, 1.30};

Pose6 tcpAt(double x, double y, double z)
{
  Pose6 p;
  p.p.setValue(x, y, z);
  p.R.setIdentity();
  return p;
}

}  // namespace

TEST(PrepositionDecision, AllCriteriaPassNeedsNothing)
{
  const Pose6 tcp = tcpAt(0.2, 0.0, 0.3);
  const PrepositionDecision d = decide_preposition(
    kFamily, kFamily, tcp, tcp, PrepositionParams{});
  EXPECT_FALSE(d.needed);
  EXPECT_EQ(d.reason, "ok");
  EXPECT_DOUBLE_EQ(d.max_joint_delta, 0.0);
}

TEST(PrepositionDecision, JointDeltaTriggers)
{
  std::array<double, 6> now = kFamily;
  now[0] += 0.16;                       // one joint past the 0.15 threshold
  const Pose6 tcp = tcpAt(0.2, 0.0, 0.3);
  const PrepositionDecision d = decide_preposition(
    kFamily, now, tcp, tcp, PrepositionParams{});
  EXPECT_TRUE(d.needed);
  EXPECT_EQ(d.reason, "joint_delta");
  EXPECT_NEAR(d.max_joint_delta, 0.16, 1e-9);
}

TEST(PrepositionDecision, JointDeltaBoundaryDoesNotTrigger)
{
  std::array<double, 6> now = kFamily;
  now[3] -= 0.15;                       // exactly the threshold
  const Pose6 tcp = tcpAt(0.2, 0.0, 0.3);
  const PrepositionDecision d = decide_preposition(
    kFamily, now, tcp, tcp, PrepositionParams{});
  EXPECT_FALSE(d.needed) << "a value ON the threshold must not count as over it";
}

TEST(PrepositionDecision, BranchDeltaTriggers)
{
  // q5 = -0.4 while the family pins +0.6: the other wrist branch. Servo would
  // integrate straight through q5 = 0 on the way there.
  std::array<double, 6> now = kFamily;
  now[4] = -0.4;
  const Pose6 tcp = tcpAt(0.2, 0.0, 0.3);
  const PrepositionDecision d = decide_preposition(
    kFamily, now, tcp, tcp, PrepositionParams{});
  EXPECT_TRUE(d.needed);
  EXPECT_EQ(d.reason, "branch");
  EXPECT_NEAR(d.q5_delta, 1.0, 1e-9);
}

TEST(PrepositionDecision, LimitMarginTriggers)
{
  // Both the family pose and the actual pose sit near the j4 limit, so the
  // joint-delta criterion stays quiet and only the limit margin fires.
  std::array<double, 6> family = kFamily;
  family[3] = 1.72;
  std::array<double, 6> now = family;
  const Pose6 tcp = tcpAt(0.2, 0.0, 0.3);
  const PrepositionDecision d = decide_preposition(
    family, now, tcp, tcp, PrepositionParams{});
  EXPECT_TRUE(d.needed);
  EXPECT_EQ(d.reason, "limit");
  EXPECT_NEAR(d.min_limit_margin, 1.7453292 - 1.72, 1e-6);
}

TEST(PrepositionDecision, TcpDistanceTriggers)
{
  std::array<double, 6> now = kFamily;
  now[1] += 0.05;                       // stay inside the joint threshold
  const Pose6 tcp_f = tcpAt(0.2, 0.0, 0.3);
  const Pose6 tcp_n = tcpAt(0.28, 0.0, 0.3);
  const PrepositionDecision d = decide_preposition(
    kFamily, now, tcp_f, tcp_n, PrepositionParams{});
  EXPECT_TRUE(d.needed);
  EXPECT_EQ(d.reason, "tcp");
  EXPECT_NEAR(d.tcp_distance, 0.08, 1e-9);
}

TEST(PrepositionDecision, TcpAngleTriggers)
{
  std::array<double, 6> now = kFamily;
  now[1] += 0.05;
  const Pose6 tcp_f = tcpAt(0.2, 0.0, 0.3);
  const Pose6 tcp_n = tcpAt(0.2, 0.0, 0.3);
  // 0.3 rad about z
  tf2::Matrix3x3 R;
  R.setEulerYPR(0.3, 0.0, 0.0);
  Pose6 rotated = tcp_n;
  rotated.R = R;
  const PrepositionDecision d = decide_preposition(
    kFamily, now, tcp_f, rotated, PrepositionParams{});
  EXPECT_TRUE(d.needed);
  EXPECT_EQ(d.reason, "tcp");
  EXPECT_NEAR(d.tcp_angle, 0.3, 1e-6);
}

TEST(PrepositionDecision, DisabledNeverNeedsPreposition)
{
  std::array<double, 6> now = kFamily;
  now[4] = -0.4;                        // would otherwise trigger "branch"
  now[0] += 1.0;
  const Pose6 tcp = tcpAt(0.2, 0.0, 0.3);
  PrepositionParams params;
  params.enable = false;
  const PrepositionDecision d = decide_preposition(
    kFamily, now, tcp, tcp, params);
  EXPECT_FALSE(d.needed);
  EXPECT_EQ(d.reason, "disabled");
  // The measurements are still filled in for the log.
  EXPECT_GE(d.max_joint_delta, 1.0);
}

TEST(PrepositionDecision, ReasonNamesFirstFailingCriterion)
{
  // Both the branch and the joint-delta criteria fail; the branch is checked
  // first because it is the dangerous one.
  std::array<double, 6> now = kFamily;
  now[4] = -0.4;
  now[0] += 0.5;
  const Pose6 tcp = tcpAt(0.2, 0.0, 0.3);
  const PrepositionDecision d = decide_preposition(
    kFamily, now, tcp, tcp, PrepositionParams{});
  EXPECT_EQ(d.reason, "branch");
}
