// tests/test_servo_session.cpp
//
// Task 8 of the 2026-11-07 node-split plan, Review Focus #4: the lifecycle
// services must be idempotent and must never make a blocking Servo call they
// do not need. Those are the contracts an operator feels when a release is
// retried or a pause arrives twice, and they were untestable while the
// sequencing lived inside the node.

#include <gtest/gtest.h>

#include <array>
#include <functional>
#include <string>

#include "lgw_bt_planner/servo/servo_session.hpp"
#include "lgw_bt_planner/servo_state_machine.hpp"

namespace
{

using lgw_bt_planner::HoldReason;
using lgw_bt_planner::RpcOutcome;
using lgw_bt_planner::ServoRpc;
using lgw_bt_planner::ServoSession;
using lgw_bt_planner::ServoEvent;
using lgw_bt_planner::ServoState;
using lgw_bt_planner::ServoStateMachine;
using lgw_bt_planner::SessionHost;
using lgw_bt_planner::SessionResult;

// Stands in for the node: a real state machine, counters instead of Servo.
class FakeHost : public SessionHost
{
public:
  void under_lock(const std::function<void()> & critical) override
  {
    ++lock_entries;
    critical();
  }
  ServoStateMachine & sm() override {return sm_;}
  void enter_hold(HoldReason reason) override
  {
    ++hold_calls;
    last_hold_reason = reason;
  }
  void clear_session() override {++clear_calls;}
  void arm_saturation_window() override {++saturation_arms;}
  void mark_dirty() override {++dirty_calls;}
  void cancel_preposition_unless_recovery() override {++preposition_cancels;}
  void arm_recovery_cancel() override {++recovery_cancel_arms;}
  std::array<double, 6> measured_positions() override {return {};}
  bool verify_seed(
    const std::array<double, 6> &, double, std::string &) override
  {
    return seed_ok;
  }

  RpcOutcome rpc(ServoRpc which, const char *) override
  {
    switch (which) {
      case ServoRpc::kPause:
        ++pause_calls;
        return {!pause_fails, pause_fails ? "pause failed" : ""};
      case ServoRpc::kUnpause:
        ++unpause_calls;
        return {true, ""};
      case ServoRpc::kReseed:
        ++reseed_calls;
        if (on_reseed) {
          on_reseed();
        }
        return {true, ""};
    }
    return {};
  }

  void log_info(const std::string &) override {}
  void log_warn(const std::string & message) override {last_log = message;}
  void log_error(const std::string & message) override {last_log = message;}

  ServoStateMachine sm_;
  int lock_entries{0};
  int pause_calls{0};
  int unpause_calls{0};
  int reseed_calls{0};
  int hold_calls{0};
  int clear_calls{0};
  int saturation_arms{0};
  int dirty_calls{0};
  int preposition_cancels{0};
  int recovery_cancel_arms{0};
  bool seed_ok{true};
  bool pause_fails{false};
  HoldReason last_hold_reason{HoldReason::kNone};
  std::string last_log;
  std::function<void()> on_reseed;
};

// Drive the state machine to TRACKING the same way the node does.
void drive_to_tracking(FakeHost & host)
{
  ASSERT_TRUE(host.sm_.handle(ServoEvent::TAKEOVER));
  ASSERT_TRUE(host.sm_.handle(ServoEvent::UNPAUSE_OK));
  ASSERT_EQ(host.sm_.state(), ServoState::TRACKING);
}

}  // namespace

TEST(ServoSession, RepeatedReleaseIsIdempotent)
{
  FakeHost host;
  drive_to_tracking(host);
  ServoSession session(host);

  const SessionResult first = session.release();
  EXPECT_TRUE(first.success);
  EXPECT_EQ(first.message, "released");
  EXPECT_EQ(host.pause_calls, 1);       // Servo paused exactly once
  EXPECT_EQ(host.clear_calls, 1);
  EXPECT_EQ(host.sm_.state(), ServoState::IDLE);

  const SessionResult second = session.release();
  EXPECT_TRUE(second.success);
  EXPECT_EQ(second.message, "already released");
  EXPECT_EQ(host.pause_calls, 1);       // no second stop
  EXPECT_EQ(host.clear_calls, 1);       // session cleared once
}

TEST(ServoSession, ReleaseAlwaysArmsTheRecoveryCancel)
{
  // Even a release that ends in "already released" must stop a recovery
  // worker from re-seeding after the session has gone.
  FakeHost host;
  drive_to_tracking(host);
  ServoSession session(host);
  session.release();
  session.release();
  EXPECT_EQ(host.recovery_cancel_arms, 2);
}

TEST(ServoSession, PauseIsRefusedOutsideTracking)
{
  FakeHost host;
  ServoSession session(host);
  ASSERT_EQ(host.sm_.state(), ServoState::IDLE);

  const SessionResult result = session.pause();

  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("pause refused"), std::string::npos);
  EXPECT_EQ(host.pause_calls, 0);       // no Servo call for a refused pause
  EXPECT_EQ(host.hold_calls, 0);
}

TEST(ServoSession, PauseEntersHoldAsAnOperatorPause)
{
  FakeHost host;
  drive_to_tracking(host);
  ServoSession session(host);

  const SessionResult result = session.pause();

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.message, "hold");
  EXPECT_EQ(host.sm_.state(), ServoState::HOLD);
  EXPECT_EQ(host.pause_calls, 1);
  // kOperatorPause is what keeps an automated recovery out of a human pause.
  EXPECT_EQ(host.hold_calls, 1);
  EXPECT_EQ(host.last_hold_reason, HoldReason::kOperatorPause);
}

TEST(ServoSession, PauseFailureFaultsAndReportsTheDetail)
{
  FakeHost host;
  drive_to_tracking(host);
  host.pause_fails = true;
  ServoSession session(host);

  const SessionResult result = session.pause();

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.message, "pause failed");
  EXPECT_EQ(host.sm_.state(), ServoState::FAULT);
}

TEST(ServoSession, ResumeIsRefusedWithoutAFreshTarget)
{
  FakeHost host;
  drive_to_tracking(host);
  ServoSession session(host);
  ASSERT_TRUE(session.pause().success);
  // HOLD, but the control tick has not armed TARGET_OK.
  ASSERT_FALSE(host.sm_.target_ok());

  const SessionResult result = session.resume();

  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("no usable target"), std::string::npos);
  EXPECT_EQ(host.reseed_calls, 0);      // nothing blocking was attempted
  EXPECT_EQ(host.unpause_calls, 0);
}

TEST(ServoSession, ResumeReSeedsBeforeUnpausing)
{
  FakeHost host;
  drive_to_tracking(host);
  ServoSession session(host);
  ASSERT_TRUE(session.pause().success);
  ASSERT_TRUE(host.sm_.handle(ServoEvent::TARGET_OK));

  const SessionResult result = session.resume();

  EXPECT_TRUE(result.success);
  EXPECT_EQ(result.message, "tracking");
  EXPECT_EQ(host.reseed_calls, 1);      // cache seeded from the current pose
  EXPECT_EQ(host.unpause_calls, 1);
  EXPECT_EQ(host.sm_.state(), ServoState::TRACKING);
}

TEST(ServoSession, ResumeIsCancelledWhenTheStateChangesDuringTheSeedingCall)
{
  // A release can arrive while start_servo blocks. The transition must not be
  // committed on top of it, and Servo must be parked again.
  FakeHost host;
  drive_to_tracking(host);
  ServoSession session(host);
  ASSERT_TRUE(session.pause().success);
  ASSERT_TRUE(host.sm_.handle(ServoEvent::TARGET_OK));
  host.on_reseed = [&host] {host.sm_.handle(ServoEvent::RELEASE);};

  const SessionResult result = session.resume();

  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("cancelled"), std::string::npos);
  EXPECT_EQ(host.unpause_calls, 0);
  EXPECT_EQ(host.pause_calls, 2);       // the pause from pause() + the cleanup
}

// 2026-11-16: 播种校验失败不再一次就 FAULT。前 9 次只延后(保持 HOLD,
// 不 unpause), 第 10 次仍失败才 FAULT —— 瞬态不该打死整个任务, 真失败仍停。
TEST(ServoSession, ResumeDefersThenFaultsWhenTheSeedCannotBeProven)
{
  FakeHost host;
  drive_to_tracking(host);
  ServoSession session(host);
  ASSERT_TRUE(session.pause().success);
  ASSERT_TRUE(host.sm_.handle(ServoEvent::TARGET_OK));
  host.seed_ok = false;

  for (int attempt = 1; attempt < 10; ++attempt) {
    const SessionResult r = session.resume();
    EXPECT_FALSE(r.success) << "attempt " << attempt;
    EXPECT_EQ(host.sm_.state(), ServoState::HOLD) << "attempt " << attempt;
  }
  EXPECT_EQ(host.unpause_calls, 0);     // never handed control back

  const SessionResult last = session.resume();
  EXPECT_FALSE(last.success);
  EXPECT_EQ(host.sm_.state(), ServoState::FAULT);
  EXPECT_EQ(host.unpause_calls, 0);
}

// 播种成功时计数要清零: 一次失败后再成功, 后续失败重新从 1 开始计。
TEST(ServoSession, ResumeClearsTheSeedRetryBudgetAfterASuccess)
{
  FakeHost host;
  drive_to_tracking(host);
  ServoSession session(host);
  ASSERT_TRUE(session.pause().success);
  ASSERT_TRUE(host.sm_.handle(ServoEvent::TARGET_OK));

  host.seed_ok = false;
  for (int i = 0; i < 5; ++i) {
    EXPECT_FALSE(session.resume().success);
  }
  EXPECT_EQ(host.sm_.state(), ServoState::HOLD);

  host.seed_ok = true;
  const SessionResult ok = session.resume();
  EXPECT_TRUE(ok.success);

  // 再进 HOLD 之后, 预算应重新是满的(不会因为刚才那 5 次更容易 FAULT)。
  ASSERT_TRUE(session.pause().success);
  ASSERT_TRUE(host.sm_.handle(ServoEvent::TARGET_OK));
  host.seed_ok = false;
  for (int i = 0; i < 5; ++i) {
    EXPECT_FALSE(session.resume().success);
  }
  EXPECT_EQ(host.sm_.state(), ServoState::HOLD) << "预算没有被上一次失败消耗掉";
}
