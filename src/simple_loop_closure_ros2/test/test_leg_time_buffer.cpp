#include <cmath>
#include <deque>

#include <gtest/gtest.h>

#include "simple_loop_closure/leg_time_buffer.hpp"

using simple_loop_closure::LegTimeSample;
using simple_loop_closure::legLowStateDeviceStamp;
using simple_loop_closure::nearestLegTimeSample;

TEST(LegTimeBuffer, PicksNearestArrivalAndReturnsDeviceStamp) {
  std::deque<LegTimeSample> buf = {
      {10.00, 100.00}, {10.05, 100.05}, {10.10, 100.10}};
  LegTimeSample out;
  ASSERT_TRUE(nearestLegTimeSample(buf, 10.06, 0.10, out));
  EXPECT_DOUBLE_EQ(out.arrival, 10.05);
  EXPECT_DOUBLE_EQ(out.device_stamp, 100.05);
}

TEST(LegTimeBuffer, RejectsOutsideTimeout) {
  std::deque<LegTimeSample> buf = {{10.00, 100.00}};
  LegTimeSample out;
  EXPECT_FALSE(nearestLegTimeSample(buf, 10.11, 0.10, out));
  EXPECT_TRUE(nearestLegTimeSample(buf, 10.10, 0.10, out));
}

TEST(LegTimeBuffer, SkipsNonFiniteDeviceStamp) {
  std::deque<LegTimeSample> buf = {
      {10.00, std::nan("")}, {10.05, 100.05}};
  LegTimeSample out;
  ASSERT_TRUE(nearestLegTimeSample(buf, 10.00, 0.10, out));
  EXPECT_DOUBLE_EQ(out.device_stamp, 100.05);
}

TEST(LegTimeBuffer, ShiftRecoversLowStateDeviceTime) {
  // LowState arrives 15 ms before the sportmodestate of the same cycle.
  const LegTimeSample pair{10.000, 100.000};
  EXPECT_DOUBLE_EQ(legLowStateDeviceStamp(pair, 9.985), 99.985);
}
