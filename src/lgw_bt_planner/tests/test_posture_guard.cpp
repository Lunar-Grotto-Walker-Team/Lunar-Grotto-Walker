// tests/test_posture_guard.cpp
//
// Guards the 2026-11-07 close-range swing: a best-effort posture that sits on
// the other wrist branch must never be published as if it were a legal one.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "lgw_bt_planner/servo/posture_guard.hpp"

namespace
{

using lgw_bt_planner::kBestEffortMaxDeltaRad;
using lgw_bt_planner::posture_continuous;

std::array<double, 6> joints(double j1, double j2, double j3, double j4,
  double j5, double j6)
{
  return {j1, j2, j3, j4, j5, j6};
}

}  // namespace

TEST(PostureGuard, AcceptsAPoseBarelyMovedFromTheLastCommand)
{
  const auto last = joints(0.0, 0.9, -0.7, 0.0, 0.6, 0.0);
  const auto next = joints(0.02, 0.92, -0.71, 0.01, 0.62, 0.02);

  EXPECT_TRUE(posture_continuous(next, last, kBestEffortMaxDeltaRad));
}

TEST(PostureGuard, RejectsTheBranchJumpSeenOnTheRobot)
{
  // Measured: best-effort pose j5=-0.600 alternating with the legal +0.851.
  const auto last = joints(0.0, 0.9, -0.7, 0.0, 0.851, 0.0);
  const auto best_effort = joints(0.0, 0.9, -0.7, 0.0, -0.600, 0.0);

  EXPECT_FALSE(posture_continuous(best_effort, last, kBestEffortMaxDeltaRad));
}

TEST(PostureGuard, RejectsASingleJointOverTheLimit)
{
  const auto last = joints(0.0, 0.9, -0.7, 0.0, 0.6, 0.0);
  const auto next = joints(0.0, 0.9, -0.7, 0.30, 0.6, 0.0);   // j4 only

  EXPECT_FALSE(posture_continuous(next, last, kBestEffortMaxDeltaRad));
}

TEST(PostureGuard, RejectsANonFiniteCandidate)
{
  auto next = joints(0.0, 0.9, -0.7, 0.0, 0.6, 0.0);
  next[3] = std::numeric_limits<double>::quiet_NaN();

  EXPECT_FALSE(posture_continuous(next, joints(0.0, 0.9, -0.7, 0.0, 0.6, 0.0),
    kBestEffortMaxDeltaRad));
}

TEST(PostureGuard, TheLimitIsWideEnoughForHonestMotion)
{
  // A 10 Hz tick at 2 rad/s of joint speed is 0.2 rad -- fast, but real.
  const auto last = joints(0.0, 0.9, -0.7, 0.0, 0.6, 0.0);
  const auto next = joints(0.0, 0.9, -0.7, 0.0, 0.6 + 0.2, 0.0);

  EXPECT_TRUE(posture_continuous(next, last, kBestEffortMaxDeltaRad));
}
