#include <deque>

#include <gtest/gtest.h>

#include "simple_loop_closure/go2_buffer.hpp"

using simple_loop_closure::Go2Sample;
using simple_loop_closure::nearestGo2Sample;
using simple_loop_closure::nearestGo2SampleByDevice;
using simple_loop_closure::validQuaternion;

TEST(Go2Buffer, EmptyReturnsFalse) {
  std::deque<Go2Sample> buf;
  Go2Sample out;
  EXPECT_FALSE(nearestGo2Sample(buf, 10.0, 0.1, out));
}

TEST(Go2Buffer, PicksNearestWithinTimeout) {
  std::deque<Go2Sample> buf;
  buf.push_back({9.90, 1.0, Eigen::Quaterniond::Identity()});
  buf.push_back({10.00, 2.0, Eigen::Quaterniond::Identity()});
  buf.push_back({10.10, 3.0, Eigen::Quaterniond::Identity()});
  Go2Sample out;
  ASSERT_TRUE(nearestGo2Sample(buf, 10.04, 0.1, out));
  EXPECT_DOUBLE_EQ(out.z, 2.0);
}

TEST(Go2Buffer, RejectsOutsideTimeout) {
  std::deque<Go2Sample> buf;
  buf.push_back({10.0, 2.0, Eigen::Quaterniond::Identity()});
  Go2Sample out;
  EXPECT_FALSE(nearestGo2Sample(buf, 10.11, 0.1, out));
  EXPECT_TRUE(nearestGo2Sample(buf, 10.10, 0.1, out));  // boundary inclusive
}

TEST(Go2Buffer, QuaternionValidity) {
  EXPECT_TRUE(validQuaternion(Eigen::Quaterniond::Identity()));
  EXPECT_FALSE(validQuaternion(Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0)));
  EXPECT_FALSE(validQuaternion(
      Eigen::Quaterniond(std::nan(""), 0.0, 0.0, 0.0)));
}

TEST(Go2Buffer, PicksNearestByDeviceStamp) {
  std::deque<Go2Sample> buf;
  Go2Sample a;
  a.stamp = 1.0;
  a.z = 0.0;
  a.q = Eigen::Quaterniond::Identity();
  a.device_stamp = 100.00;
  Go2Sample b;
  b.stamp = 2.0;
  b.z = 0.0;
  b.q = Eigen::Quaterniond::Identity();
  b.device_stamp = 100.05;
  buf.push_back(a);
  buf.push_back(b);
  Go2Sample out;
  ASSERT_TRUE(nearestGo2SampleByDevice(buf, 100.06, 0.10, out));
  EXPECT_DOUBLE_EQ(out.device_stamp, 100.05);
}
