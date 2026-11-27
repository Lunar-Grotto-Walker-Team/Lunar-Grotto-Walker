#include <array>
#include <cmath>
#include <limits>

#include <Eigen/Geometry>
#include <gtest/gtest.h>

#include "simple_loop_closure/leg_z_integrator.hpp"

using simple_loop_closure::LegAttitude;
using simple_loop_closure::LegSample;
using simple_loop_closure::LegZConfig;
using simple_loop_closure::LegZIntegrator;

namespace {

// Feet fixed in the world; the body translates.  All four feet loaded.
LegSample makeSample(double t, const Eigen::Vector3d& body,
                     double force = 40.0) {
  const std::array<Eigen::Vector3d, 4> world = {
      Eigen::Vector3d(0, 0.15, -0.30), Eigen::Vector3d(0, -0.15, -0.30),
      Eigen::Vector3d(-0.19, 0.15, -0.30),
      Eigen::Vector3d(-0.19, -0.15, -0.30)};
  LegSample s;
  s.stamp = t;
  for (int i = 0; i < 4; ++i) {
    s.foot[i] = world[i] - body;
    s.force[i] = force;
  }
  return s;
}

LegAttitude identityAtt(double t) {
  LegAttitude a;
  a.stamp = t;
  a.R = Eigen::Matrix3d::Identity();
  a.valid = true;
  return a;
}

// Synthetic samples are in a frame with no sensor-mount offset.  The
// production default (1.3 deg pitch) is a calibrated hardware constant;
// these geometry tests set it to zero so they isolate the math.
LegZConfig noMountCfg() {
  LegZConfig c;
  c.pitch_offset_deg = 0.0;
  c.roll_offset_deg = 0.0;
  return c;
}

}  // namespace

TEST(LegZIntegrator, FlatForwardKeepsZ) {
  LegZIntegrator z(noMountCfg());
  z.update(makeSample(0.00, Eigen::Vector3d(0, 0, 0)), identityAtt(0.00));
  z.update(makeSample(0.05, Eigen::Vector3d(0.01, 0, 0)), identityAtt(0.05));
  EXPECT_NEAR(z.z(), 0.0, 1e-9);
}

TEST(LegZIntegrator, ClimbAccumulatesZ) {
  LegZIntegrator z(noMountCfg());
  z.update(makeSample(0.00, Eigen::Vector3d(0, 0, 0)), identityAtt(0.00));
  z.update(makeSample(0.05, Eigen::Vector3d(0, 0, 0.01)), identityAtt(0.05));
  EXPECT_NEAR(z.z(), 0.01, 1e-9);
}

TEST(LegZIntegrator, ConsensusRejectsMovingFoot) {
  LegZIntegrator z(noMountCfg());
  z.update(makeSample(0.00, Eigen::Vector3d(0, 0, 0)), identityAtt(0.00));
  auto s = makeSample(0.05, Eigen::Vector3d(0.01, 0, 0));
  s.foot[2].z() += 0.10;  // one foot disagrees
  z.update(s, identityAtt(0.05));
  EXPECT_NEAR(z.z(), 0.0, 1e-9);
  EXPECT_GT(z.diagnostics().last_consensus_spread, 0.09);
}

TEST(LegZIntegrator, SingleFootNeedsSettleAndForce) {
  LegZIntegrator z(noMountCfg());
  auto s0 = makeSample(0.00, Eigen::Vector3d(0, 0, 0));
  for (int i = 1; i < 4; ++i) s0.force[i] = 0.0;
  z.update(s0, identityAtt(0.00));
  auto s1 = makeSample(0.05, Eigen::Vector3d(0.01, 0, 0));
  for (int i = 1; i < 4; ++i) s1.force[i] = 0.0;
  z.update(s1, identityAtt(0.05));
  EXPECT_EQ(z.diagnostics().steps_used, 0);  // contact age < settle_s
  auto s2 = makeSample(0.15, Eigen::Vector3d(0.03, 0, 0));
  for (int i = 1; i < 4; ++i) s2.force[i] = 0.0;
  z.update(s2, identityAtt(0.15));
  EXPECT_EQ(z.diagnostics().steps_used, 1);
}

TEST(LegZIntegrator, PitchOffsetProjectsForwardMotion) {
  LegZConfig cfg;
  cfg.pitch_offset_deg = 1.3;
  LegZIntegrator z(cfg);
  z.update(makeSample(0.00, Eigen::Vector3d(0, 0, 0)), identityAtt(0.00));
  z.update(makeSample(0.05, Eigen::Vector3d(0.01, 0, 0)), identityAtt(0.05));
  const double expected = -0.01 * std::sin(1.3 * M_PI / 180.0);
  EXPECT_NEAR(z.z(), expected, 1e-6);
}

TEST(LegZIntegrator, MissingAttitudeSkips) {
  LegZIntegrator z(noMountCfg());
  LegAttitude bad;
  bad.stamp = 0.05;
  bad.valid = false;
  z.update(makeSample(0.00, Eigen::Vector3d(0, 0, 0)), identityAtt(0.00));
  z.update(makeSample(0.05, Eigen::Vector3d(0.01, 0, 0)), bad);
  EXPECT_EQ(z.z(), 0.0);
  EXPECT_EQ(z.diagnostics().skip_no_attitude, 1);
}

TEST(LegZIntegrator, BigStepIsRejected) {
  LegZIntegrator z(noMountCfg());
  z.update(makeSample(0.00, Eigen::Vector3d(0, 0, 0)), identityAtt(0.00));
  z.update(makeSample(0.05, Eigen::Vector3d(0, 0, 0.10)), identityAtt(0.05));
  EXPECT_EQ(z.z(), 0.0);
  EXPECT_EQ(z.diagnostics().skip_big_dz, 1);
}

TEST(LegZIntegrator, RollOffsetProjectsLateralMotion) {
  LegZConfig cfg;
  cfg.roll_offset_deg = 1.0;
  LegZIntegrator z(cfg);
  z.update(makeSample(0.00, Eigen::Vector3d(0, 0, 0)), identityAtt(0.00));
  z.update(makeSample(0.05, Eigen::Vector3d(0, 0.01, 0)), identityAtt(0.05));
  EXPECT_NEAR(z.z(), 0.01 * std::sin(1.0 * M_PI / 180.0), 1e-6);
}

TEST(LegZIntegrator, NonFiniteInputSkips) {
  LegZIntegrator z(noMountCfg());
  z.update(makeSample(0.00, Eigen::Vector3d(0, 0, 0)), identityAtt(0.00));
  auto s = makeSample(0.05, Eigen::Vector3d(0.01, 0, 0));
  s.foot[0].x() = std::nan("");
  z.update(s, identityAtt(0.05));
  EXPECT_EQ(z.z(), 0.0);
  EXPECT_EQ(z.diagnostics().skip_non_finite, 1);
}

TEST(LegZIntegrator, AttitudeJumpSkips) {
  LegZIntegrator z(noMountCfg());
  z.update(makeSample(0.00, Eigen::Vector3d(0, 0, 0)), identityAtt(0.00));
  auto a = identityAtt(0.05);
  a.R = Eigen::AngleAxisd(20.0 * M_PI / 180.0,
                          Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
  z.update(makeSample(0.05, Eigen::Vector3d(0.01, 0, 0)), a);
  EXPECT_EQ(z.z(), 0.0);
  EXPECT_EQ(z.diagnostics().skip_attitude_jump, 1);
}

TEST(LegZKeyframe, RequiresValidTrackingAndNewStep) {
  EXPECT_TRUE(simple_loop_closure::legZKeyframeUsable(true, 10, 5));
  EXPECT_FALSE(simple_loop_closure::legZKeyframeUsable(false, 10, 5));
  EXPECT_FALSE(simple_loop_closure::legZKeyframeUsable(true, 5, 5));
}

TEST(LegZKeyframe, StaleOrInvalidBecomesNaN) {
  EXPECT_TRUE(std::isnan(simple_loop_closure::legZAtKeyframe(
      9.80, true, -0.2, 10.0, 0.10)));
  EXPECT_DOUBLE_EQ(simple_loop_closure::legZAtKeyframe(
                       9.95, true, -0.2, 10.0, 0.10),
                   -0.2);
  EXPECT_TRUE(std::isnan(simple_loop_closure::legZAtKeyframe(
      9.95, false, -0.2, 10.0, 0.10)));
}
