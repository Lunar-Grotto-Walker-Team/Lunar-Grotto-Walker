#include <gtest/gtest.h>

#include <cmath>

#include "go2_transit/transit_core.hpp"

using go2_transit::FailReason;
using go2_transit::MachineCommand;
using go2_transit::Pose2D;
using go2_transit::TaskMode;
using go2_transit::TransitParams;
using go2_transit::TransitPhase;
using go2_transit::TransitStateMachine;

namespace
{

TransitParams default_params()
{
  TransitParams p;
  p.default_xy_tolerance = 0.25;
  p.default_yaw_tolerance = 0.17;
  p.path_spacing_m = 0.25;
  p.pre_align_threshold_rad = 0.5;
  p.pre_align_fallback = true;
  p.max_retries = 2;
  return p;
}

}  // namespace

TEST(StateMachine, WaitsForReadyBeforeMoving)
{
  TransitStateMachine m(default_params(), TaskMode::TRANSIT);
  m.set_transit_goal(Pose2D{10.0, 0.0, 0.0}, 0.25, false, 0.17);
  m.set_pose(Pose2D{0.0, 0.0, 0.0});

  const auto cmd = m.next_command();
  EXPECT_EQ(cmd.kind, MachineCommand::Kind::WAIT_READY);
  EXPECT_EQ(cmd.id, 0u);
  EXPECT_EQ(m.phase(), TransitPhase::WAIT_READY);
  EXPECT_FALSE(m.finished());
}

TEST(StateMachine, PreAlignsWhenHeadingFarOff)
{
  TransitStateMachine m(default_params(), TaskMode::TRANSIT);
  m.set_transit_goal(Pose2D{0.0, 10.0, 0.0}, 0.25, false, 0.17);
  m.set_pose(Pose2D{0.0, 0.0, 0.0});
  m.set_ready(true);

  const auto cmd = m.next_command();
  EXPECT_EQ(cmd.kind, MachineCommand::Kind::SPIN);
  EXPECT_NEAR(cmd.spin_target_yaw, M_PI_2, 1e-9);
  EXPECT_EQ(m.phase(), TransitPhase::PRE_ALIGN);
}

TEST(StateMachine, SkipsPreAlignWhenHeadingClose)
{
  TransitStateMachine m(default_params(), TaskMode::TRANSIT);
  m.set_transit_goal(Pose2D{10.0, 0.0, 0.0}, 0.25, false, 0.17);
  m.set_pose(Pose2D{0.0, 0.0, 0.0});
  m.set_ready(true);

  const auto cmd = m.next_command();
  EXPECT_EQ(cmd.kind, MachineCommand::Kind::FOLLOW_PATH);
  EXPECT_GE(cmd.path.size(), 2u);
  EXPECT_NEAR(cmd.path.back().x, 10.0, 1e-9);
  EXPECT_EQ(m.phase(), TransitPhase::TRAVERSE);
}

TEST(StateMachine, NextCommandIsIdempotent)
{
  TransitStateMachine m(default_params(), TaskMode::TRANSIT);
  m.set_transit_goal(Pose2D{10.0, 0.0, 0.0}, 0.25, false, 0.17);
  m.set_pose(Pose2D{0.0, 0.0, 0.0});
  m.set_ready(true);

  const auto first = m.next_command();
  const auto second = m.next_command();
  EXPECT_EQ(first.kind, second.kind);
  EXPECT_EQ(first.id, second.id);
}

TEST(StateMachine, FallbackGoesStraightWhenPreAlignFails)
{
  TransitStateMachine m(default_params(), TaskMode::TRANSIT);
  m.set_transit_goal(Pose2D{0.0, 10.0, 0.0}, 0.25, false, 0.17);
  m.set_pose(Pose2D{0.0, 0.0, 0.0});
  m.set_ready(true);

  ASSERT_EQ(m.next_command().kind, MachineCommand::Kind::SPIN);
  m.report_action_done(false);

  const auto after_fallback = m.next_command();
  ASSERT_EQ(after_fallback.kind, MachineCommand::Kind::FOLLOW_PATH);

  // 回退不消耗重试次数：max_retries=2 时仍应允许 3 次 FOLLOW_PATH
  int follow_path_count = 1;
  while (!m.finished()) {
    m.report_action_done(false);
    if (m.next_command().kind == MachineCommand::Kind::FOLLOW_PATH) {
      ++follow_path_count;
    }
  }
  EXPECT_EQ(follow_path_count, 3);
  EXPECT_EQ(m.result_reason(), FailReason::UNREACHABLE);
}

TEST(StateMachine, FollowPathFailureRetriesThenFails)
{
  TransitStateMachine m(default_params(), TaskMode::TRANSIT);
  m.set_transit_goal(Pose2D{10.0, 0.0, 0.0}, 0.25, false, 0.17);
  m.set_pose(Pose2D{0.0, 0.0, 0.0});
  m.set_ready(true);

  int attempts = 0;
  while (!m.finished()) {
    ASSERT_EQ(m.next_command().kind, MachineCommand::Kind::FOLLOW_PATH);
    ++attempts;
    m.report_action_done(false);
  }
  EXPECT_EQ(attempts, 3);  // max_retries=2 → 3 次尝试
  EXPECT_EQ(m.result_reason(), FailReason::UNREACHABLE);
  EXPECT_EQ(m.next_command().kind, MachineCommand::Kind::FAIL);
}

TEST(StateMachine, VerifyFailureTriggersRetry)
{
  TransitStateMachine m(default_params(), TaskMode::TRANSIT);
  m.set_transit_goal(Pose2D{10.0, 0.0, 0.0}, 0.25, false, 0.17);
  m.set_pose(Pose2D{0.0, 0.0, 0.0});
  m.set_ready(true);

  ASSERT_EQ(m.next_command().kind, MachineCommand::Kind::FOLLOW_PATH);
  m.report_action_done(true);  // 报成功但位姿没动

  const auto cmd = m.next_command();
  EXPECT_EQ(cmd.kind, MachineCommand::Kind::FOLLOW_PATH);
  EXPECT_FALSE(m.finished());
}

TEST(StateMachine, AlignYawAddsFinalSpin)
{
  TransitStateMachine m(default_params(), TaskMode::TRANSIT);
  m.set_transit_goal(Pose2D{10.0, 0.0, 0.3}, 0.25, true, 0.17);
  m.set_pose(Pose2D{0.0, 0.0, 0.0});
  m.set_ready(true);

  ASSERT_EQ(m.next_command().kind, MachineCommand::Kind::FOLLOW_PATH);
  m.set_pose(Pose2D{10.0, 0.0, 0.0});  // 位置到位，朝向还差 0.3
  m.report_action_done(true);

  const auto cmd = m.next_command();
  EXPECT_EQ(cmd.kind, MachineCommand::Kind::SPIN);
  EXPECT_NEAR(cmd.spin_target_yaw, 0.3, 1e-9);
  EXPECT_EQ(m.phase(), TransitPhase::FINAL_ALIGN);
}

TEST(StateMachine, SkipsFinalSpinWhenYawAlreadyAligned)
{
  // 2026-11-27 真机: FollowPath 成功后无条件 FINAL_ALIGN，即使 yaw 已经在
  // 容差内也要转一下；四足原地转 5° 会把 base_footprint 平移 ~2 cm，把
  // 刚达标的 XY 撞出容差 -> verify 失败 -> 重试。yaw 已达标时不应再转。
  TransitStateMachine m(default_params(), TaskMode::TRANSIT);
  m.set_transit_goal(Pose2D{10.0, 0.0, 0.05}, 0.25, true, 0.17);  // yaw 误差 0.05 < 0.17
  m.set_pose(Pose2D{0.0, 0.0, 0.0});
  m.set_ready(true);

  ASSERT_EQ(m.next_command().kind, MachineCommand::Kind::FOLLOW_PATH);
  m.set_pose(Pose2D{10.0, 0.0, 0.02});  // 位置到位，朝向也早已在容差内
  m.report_action_done(true);

  const auto cmd = m.next_command();
  EXPECT_TRUE(m.finished());
  EXPECT_EQ(cmd.kind, MachineCommand::Kind::SUCCEED);
  EXPECT_EQ(m.result_reason(), FailReason::OK);
  EXPECT_NE(m.phase(), TransitPhase::FINAL_ALIGN);
}

TEST(StateMachine, SkipsFinalSpinWhenPositionNotReached)
{
  // 位置没到就别先转：转了也过不了 verify，白花一次 spin 还会把位姿带偏。
  // 应直接 verify -> 失败 -> 重试 traverse。
  TransitStateMachine m(default_params(), TaskMode::TRANSIT);
  m.set_transit_goal(Pose2D{10.0, 0.0, 0.3}, 0.25, true, 0.17);
  m.set_pose(Pose2D{0.0, 0.0, 0.0});
  m.set_ready(true);

  ASSERT_EQ(m.next_command().kind, MachineCommand::Kind::FOLLOW_PATH);
  m.set_pose(Pose2D{5.0, 0.0, 0.0});  // 只走了一半
  m.report_action_done(true);

  const auto cmd = m.next_command();
  EXPECT_EQ(cmd.kind, MachineCommand::Kind::FOLLOW_PATH);  // 直接重试 traverse
  EXPECT_FALSE(m.finished());
}

TEST(StateMachine, RotateTaskOnlySpins)
{
  TransitStateMachine m(default_params(), TaskMode::ROTATE);
  m.set_rotate_goal(1.2, 0.17);
  m.set_pose(Pose2D{0.0, 0.0, 0.0});
  m.set_ready(true);

  const auto cmd = m.next_command();
  ASSERT_EQ(cmd.kind, MachineCommand::Kind::SPIN);
  EXPECT_NEAR(cmd.spin_target_yaw, 1.2, 1e-9);

  m.set_pose(Pose2D{0.0, 0.0, 1.2});
  m.report_action_done(true);
  EXPECT_TRUE(m.finished());
  EXPECT_EQ(m.result_reason(), FailReason::OK);
}

TEST(StateMachine, AlreadyAtGoalSucceedsImmediately)
{
  TransitStateMachine m(default_params(), TaskMode::TRANSIT);
  m.set_transit_goal(Pose2D{1.0, 2.0, 0.3}, 0.25, false, 0.17);
  m.set_pose(Pose2D{1.0, 2.0, 0.3});
  m.set_ready(true);

  const auto cmd = m.next_command();
  EXPECT_TRUE(m.finished());
  EXPECT_EQ(m.result_reason(), FailReason::OK);
  EXPECT_EQ(cmd.kind, MachineCommand::Kind::SUCCEED);
}

TEST(StateMachine, CancelEndsWithCanceled)
{
  TransitStateMachine m(default_params(), TaskMode::TRANSIT);
  m.set_transit_goal(Pose2D{10.0, 0.0, 0.0}, 0.25, false, 0.17);
  m.set_pose(Pose2D{0.0, 0.0, 0.0});
  m.set_ready(true);
  ASSERT_EQ(m.next_command().kind, MachineCommand::Kind::FOLLOW_PATH);

  m.cancel();
  EXPECT_TRUE(m.finished());
  EXPECT_EQ(m.result_reason(), FailReason::CANCELED);
  const auto cmd = m.next_command();
  EXPECT_EQ(cmd.kind, MachineCommand::Kind::FAIL);
  EXPECT_EQ(cmd.fail_reason, FailReason::CANCELED);
}

TEST(StateMachine, TimeoutEndsWithTimeout)
{
  TransitStateMachine m(default_params(), TaskMode::TRANSIT);
  m.set_transit_goal(Pose2D{10.0, 0.0, 0.0}, 0.25, false, 0.17);
  m.set_ready(true);

  m.report_timeout();
  EXPECT_TRUE(m.finished());
  EXPECT_EQ(m.result_reason(), FailReason::TIMEOUT);
}

TEST(StateMachine, ExternalFailReasonIsReported)
{
  TransitStateMachine m(default_params(), TaskMode::TRANSIT);
  m.set_transit_goal(Pose2D{10.0, 0.0, 0.0}, 0.25, false, 0.17);

  m.fail(FailReason::TF_UNAVAILABLE);
  EXPECT_TRUE(m.finished());
  EXPECT_EQ(m.result_reason(), FailReason::TF_UNAVAILABLE);
  EXPECT_EQ(m.next_command().fail_reason, FailReason::TF_UNAVAILABLE);
}
