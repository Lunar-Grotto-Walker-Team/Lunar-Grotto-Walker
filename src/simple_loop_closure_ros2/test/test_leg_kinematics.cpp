#include <gtest/gtest.h>

#include "simple_loop_closure/leg_kinematics.hpp"

using simple_loop_closure::LegContactDetector;
using simple_loop_closure::LegGeometry;
using simple_loop_closure::legFootPosition;

TEST(LegKinematics, StandingPoseMatchesUrdf) {
  LegGeometry g;
  const auto fl = legFootPosition(g, 0, 0.045, 0.735, -1.555);
  EXPECT_NEAR(fl.x(), 0.206, 0.005);
  EXPECT_NEAR(fl.y(), 0.156, 0.005);
  EXPECT_NEAR(fl.z(), -0.299, 0.005);
  const auto fr = legFootPosition(g, 1, -0.014, 0.811, -1.561);
  EXPECT_NEAR(fr.x(), 0.184, 0.005);
  EXPECT_NEAR(fr.y(), -0.146, 0.005);
  EXPECT_NEAR(fr.z(), -0.301, 0.005);
}

TEST(LegKinematics, LegOrderAndSides) {
  LegGeometry g;
  const auto fl = legFootPosition(g, 0, 0.0, 0.7, -1.4);
  const auto fr = legFootPosition(g, 1, 0.0, 0.7, -1.4);
  const auto rl = legFootPosition(g, 2, 0.0, 0.7, -1.4);
  EXPECT_GT(fl.x(), 0.0);
  EXPECT_LT(rl.x(), 0.0);
  EXPECT_GT(fl.y(), 0.0);
  EXPECT_LT(fr.y(), 0.0);
}

TEST(LegContactDetector, HysteresisAndStartTime) {
  LegContactDetector d(25.0, 12.0);
  d.update(0.00, {10, 10, 10, 10});
  EXPECT_FALSE(d.contact()[0]);
  d.update(0.05, {26, 10, 10, 10});
  EXPECT_TRUE(d.contact()[0]);
  EXPECT_DOUBLE_EQ(d.contactStart()[0], 0.05);
  d.update(0.10, {24, 10, 10, 10});
  EXPECT_TRUE(d.contact()[0]);
  d.update(0.15, {11, 10, 10, 10});
  EXPECT_FALSE(d.contact()[0]);
}
