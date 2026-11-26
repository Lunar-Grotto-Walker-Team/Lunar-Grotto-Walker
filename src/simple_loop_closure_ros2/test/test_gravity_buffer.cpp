#include <gtest/gtest.h>

#include "simple_loop_closure/gravity_buffer.hpp"

namespace {

simple_loop_closure::GravitySample makeSample(double stamp,
                                              const Eigen::Vector3d& force) {
  simple_loop_closure::GravitySample s;
  s.stamp = stamp;
  s.specific_force = force;
  return s;
}

}  // namespace

TEST(GravityBuffer, ReturnsNearestWithinTimeout) {
  std::deque<simple_loop_closure::GravitySample> buf;
  buf.push_back(makeSample(0.0, {0, 0, 1}));
  buf.push_back(makeSample(0.1, {0, 0, 1}));
  buf.push_back(makeSample(0.5, {0, 0, 1}));

  simple_loop_closure::GravitySample out;
  ASSERT_TRUE(simple_loop_closure::nearestGravitySample(buf, 0.12, 0.30, out));
  EXPECT_DOUBLE_EQ(out.stamp, 0.1);
}

TEST(GravityBuffer, ReturnsNoneBeyondTimeout) {
  std::deque<simple_loop_closure::GravitySample> buf;
  buf.push_back(makeSample(0.0, {0, 0, 1}));
  buf.push_back(makeSample(0.5, {0, 0, 1}));

  simple_loop_closure::GravitySample out;
  EXPECT_FALSE(simple_loop_closure::nearestGravitySample(buf, 0.9, 0.30, out));
}

TEST(GravityBuffer, SkipsNonFiniteSample) {
  std::deque<simple_loop_closure::GravitySample> buf;
  buf.push_back(
      makeSample(0.1, {std::nan(""), std::nan(""), std::nan("")}));
  buf.push_back(makeSample(0.2, {0, 0, 1}));

  simple_loop_closure::GravitySample out;
  ASSERT_TRUE(simple_loop_closure::nearestGravitySample(buf, 0.1, 0.30, out));
  EXPECT_DOUBLE_EQ(out.stamp, 0.2);
}

// A single accelerometer sample under gait swings by tens of degrees, so the
// prior must average a window around the keyframe time.
TEST(GravityBuffer, AveragesWindowAroundStamp) {
  std::deque<simple_loop_closure::GravitySample> buf;
  buf.push_back(makeSample(0.0, {0, 0, 1}));
  buf.push_back(makeSample(0.1, {0, 0, 3}));
  buf.push_back(makeSample(0.2, {0, 0, 5}));
  buf.push_back(makeSample(0.3, {0, 0, 7}));

  simple_loop_closure::GravitySample out;
  ASSERT_TRUE(simple_loop_closure::averageGravitySample(buf, 0.2, 0.15, out));
  EXPECT_NEAR(out.specific_force.z(), 5.0, 1e-12);  // 0.1, 0.2, 0.3
}

// Stairs/jumps: the impact lasts 0.1-0.3 s, so the 0.30 s mean still
// deviates 20-40% from 1 g and would inject a bogus attitude correction.
TEST(GravityBuffer, NormGateRejectsImpactWindows) {
  EXPECT_TRUE(simple_loop_closure::gravityNormAccepted(1.05, 1.0, 0.2));
  EXPECT_FALSE(simple_loop_closure::gravityNormAccepted(1.35, 1.0, 0.2));
  EXPECT_FALSE(simple_loop_closure::gravityNormAccepted(0.70, 1.0, 0.2));
  // tol <= 0 disables the gate; ema <= 0 means "not initialised yet".
  EXPECT_TRUE(simple_loop_closure::gravityNormAccepted(1.35, 1.0, 0.0));
  EXPECT_TRUE(simple_loop_closure::gravityNormAccepted(1.35, 0.0, 0.2));
}

// Source filtering: gravity_source=auto prefers lowstate but falls back to
// livox when the window has no lowstate sample (bag replay).
TEST(GravityBuffer, AveragesOnlyTheRequestedSource) {
  std::deque<simple_loop_closure::GravitySample> buf;
  auto ls = makeSample(0.1, {0, 0, 9.8});
  ls.source = simple_loop_closure::kGravityLowstate;
  auto lx = makeSample(0.1, {0, 0, 1.0});
  lx.source = simple_loop_closure::kGravityLivox;
  buf.push_back(ls);
  buf.push_back(lx);

  simple_loop_closure::GravitySample out;
  ASSERT_TRUE(simple_loop_closure::averageGravitySample(
      buf, 0.1, 0.15, out, simple_loop_closure::kGravityLivox));
  EXPECT_NEAR(out.specific_force.z(), 1.0, 1e-12);
  ASSERT_TRUE(simple_loop_closure::averageGravitySample(
      buf, 0.1, 0.15, out, simple_loop_closure::kGravityLowstate));
  EXPECT_NEAR(out.specific_force.z(), 9.8, 1e-12);
  // No sample of that source -> false, so the caller can fall back.
  EXPECT_FALSE(simple_loop_closure::averageGravitySample(
      buf, 0.1, 0.15, out, simple_loop_closure::kGravityUtlidar));
}

TEST(GravityBuffer, AverageEmptyWindowReturnsFalse) {
  std::deque<simple_loop_closure::GravitySample> buf;
  buf.push_back(makeSample(0.0, {0, 0, 1}));

  simple_loop_closure::GravitySample out;
  EXPECT_FALSE(simple_loop_closure::averageGravitySample(buf, 0.5, 0.15, out));
}
