#include <gtest/gtest.h>
#include "simple_loop_closure/loop_noise.hpp"

TEST(LoopNoise, ThreeDofFreesZAndAttitude) {
  // fitness 0.01 -> xy/yaw variance 0.01; z/roll/pitch -> 1e4
  const auto v = simple_loop_closure::makeLoopVariances(
      0.01, 1.0, 1e-4, /*constrain_z=*/false, /*constrain_attitude=*/false, 1e4);
  ASSERT_EQ(v.size(), 6);
  EXPECT_DOUBLE_EQ(v(0), 1e4);   // Rx roll  free
  EXPECT_DOUBLE_EQ(v(1), 1e4);   // Ry pitch free
  EXPECT_DOUBLE_EQ(v(2), 0.01);  // Rz yaw   loop
  EXPECT_DOUBLE_EQ(v(3), 0.01);  // Tx x     loop
  EXPECT_DOUBLE_EQ(v(4), 0.01);  // Ty y     loop
  EXPECT_DOUBLE_EQ(v(5), 1e4);   // Tz z     free
}

TEST(LoopNoise, SixDofConstrainsEverything) {
  const auto v = simple_loop_closure::makeLoopVariances(
      0.01, 1.0, 1e-4, /*constrain_z=*/true, /*constrain_attitude=*/true, 1e4);
  for (int i = 0; i < 6; ++i) EXPECT_DOUBLE_EQ(v(i), 0.01);
}

TEST(LoopNoise, FitnessBelowFloorUsesFloor) {
  const auto v = simple_loop_closure::makeLoopVariances(
      1e-9, 1.0, 1e-4, false, false, 1e4);
  EXPECT_DOUBLE_EQ(v(2), 1e-4);
  EXPECT_DOUBLE_EQ(v(3), 1e-4);
}

TEST(LoopNoise, DistanceProportionalGate) {
  // A false match 18 m along the path with a 7.3 m ICP shift must be
  // rejected; the true ring closure at ~100 m keeps a wider gate.
  EXPECT_DOUBLE_EQ(simple_loop_closure::makeLoopGate(18.0, 0.3, 0.10), 1.8);
  EXPECT_DOUBLE_EQ(simple_loop_closure::makeLoopGate(100.0, 0.3, 0.10), 10.0);
  EXPECT_DOUBLE_EQ(simple_loop_closure::makeLoopGate(0.0, 0.3, 0.10), 0.3);
  EXPECT_DOUBLE_EQ(simple_loop_closure::makeLoopGate(18.0, 0.3, 0.0), 0.3);
}

TEST(LoopNoise, HardCapClampsLongBaselineGate) {
  // 2026-11-17: a 100 m baseline would allow 10 m; the cap must win.
  EXPECT_DOUBLE_EQ(simple_loop_closure::makeLoopGate(100.0, 0.3, 0.10, 1.0),
                   1.0);
  // A 45 m corridor loop: ratio gate 2.7 m, capped to 1.0 m.
  EXPECT_DOUBLE_EQ(simple_loop_closure::makeLoopGate(45.0, 0.2, 0.06, 1.0),
                   1.0);
}

TEST(LoopNoise, HardCapLeavesShortBaselineAlone) {
  // Below the cap the ratio/floor result is unchanged.
  EXPECT_DOUBLE_EQ(simple_loop_closure::makeLoopGate(18.0, 0.3, 0.10, 2.0),
                   1.8);
  EXPECT_DOUBLE_EQ(simple_loop_closure::makeLoopGate(5.0, 0.3, 0.10, 1.0),
                   0.5);
}

TEST(LoopNoise, NonPositiveCapDisablesCap) {
  EXPECT_DOUBLE_EQ(simple_loop_closure::makeLoopGate(100.0, 0.3, 0.10, 0.0),
                   10.0);
  EXPECT_DOUBLE_EQ(simple_loop_closure::makeLoopGate(100.0, 0.3, 0.10, -1.0),
                   10.0);
}

TEST(LoopNoise, HardCapBeatsFloor) {
  // The cap is a hard limit: even the floor is clamped by it.
  EXPECT_DOUBLE_EQ(simple_loop_closure::makeLoopGate(0.0, 0.3, 0.10, 0.2),
                   0.2);
}
