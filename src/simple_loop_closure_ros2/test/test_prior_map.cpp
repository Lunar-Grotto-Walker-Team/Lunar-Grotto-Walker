#include <gtest/gtest.h>

#include <sstream>
#include <string>
#include <vector>

#include "simple_loop_closure/prior_map.hpp"

TEST(PriorMap, ParsesHeaderAndRows) {
  std::istringstream in(
      "# frame: body\nindex, timestamp, x, y, z, qx, qy, qz, qw\n"
      "0, 100.0, 1.0, 2.0, 3.0, 0, 0, 0, 1\n"
      "1, 100.5, 4.0, 5.0, 6.0, 0, 0, 0, 1\n");
  std::string first;
  std::getline(in, first);
  EXPECT_TRUE(simple_loop_closure::priorHeaderIsBodyFrame(first));
  EXPECT_FALSE(simple_loop_closure::priorHeaderIsBodyFrame(
      "index, timestamp, x, y, z, qx, qy, qz, qw"));

  std::vector<simple_loop_closure::PriorPose> poses;
  ASSERT_TRUE(simple_loop_closure::parsePriorPoses(in, poses));
  ASSERT_EQ(poses.size(), 2u);
  EXPECT_EQ(poses[1].index, 1);
  EXPECT_DOUBLE_EQ(poses[1].t.y(), 5.0);
  EXPECT_DOUBLE_EQ(poses[1].q.w(), 1.0);
}

TEST(PriorMap, SelectAppliesTranslationBeforeCrop) {
  // Mouth at (2.37,-2.43) in the old base; after (+0.06,+1.91) it must land
  // at (2.43,-0.52) in the current base.  A 0.5 m crop around the current
  // mouth keeps it -- and would drop it if the translation were skipped or
  // applied with the wrong sign.  Spacing is off so the assertion is about
  // the crop only.
  std::vector<simple_loop_closure::PriorPose> poses(3);
  poses[0].t = Eigen::Vector3d(0.0, 0.0, 0.0);      // old base, outside crop
  poses[1].t = Eigen::Vector3d(2.37, -2.43, -0.5);  // cave mouth
  poses[2].t = Eigen::Vector3d(2.40, -2.45, -0.7);  // deeper inside
  const auto kept = simple_loop_closure::selectPriorPoses(
      poses, 0.06, 1.91, /*crop_cx=*/2.43, /*crop_cy=*/-0.52,
      /*crop_radius=*/0.5, /*min_spacing=*/0.0);
  ASSERT_EQ(kept.size(), 2u);
  EXPECT_EQ(kept[0], 1u);
  EXPECT_EQ(kept[1], 2u);
}

TEST(PriorMap, SelectRespectsMinSpacing) {
  std::vector<simple_loop_closure::PriorPose> poses(3);
  poses[0].t = Eigen::Vector3d(10.0, 0.0, 0.0);
  poses[1].t = Eigen::Vector3d(10.2, 0.0, 0.0);  // 0.2 m away, dropped
  poses[2].t = Eigen::Vector3d(10.6, 0.0, 0.0);  // 0.6 m away, kept
  const auto kept = simple_loop_closure::selectPriorPoses(
      poses, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5);
  ASSERT_EQ(kept.size(), 2u);
  EXPECT_EQ(kept[0], 0u);
  EXPECT_EQ(kept[1], 2u);
}

TEST(PriorMap, NearestRespectsRadius) {
  std::vector<simple_loop_closure::PriorPose> poses(2);
  poses[0].t = Eigen::Vector3d(0.0, 0.0, 0.0);
  poses[1].t = Eigen::Vector3d(5.0, 0.0, 0.0);
  simple_loop_closure::PriorIndex idx;
  idx.build(poses, {0, 1});
  EXPECT_EQ(idx.size(), 2u);
  EXPECT_EQ(idx.nearest(0.5, 0.0, 1.5), 0);
  EXPECT_EQ(idx.nearest(2.5, 0.0, 1.5), -1);
  EXPECT_EQ(idx.nearest(4.4, 0.0, 1.5), 1);
}

TEST(PriorMap, SequenceNeedsTwoNeighbouringFrames) {
  using simple_loop_closure::priorSequenceAgrees;
  EXPECT_FALSE(priorSequenceAgrees(-1, -1, 10, 40, 8, 3));  // first frame waits
  EXPECT_TRUE(priorSequenceAgrees(10, 40, 14, 42, 8, 3));   // adjacent step
  EXPECT_FALSE(priorSequenceAgrees(10, 40, 14, 60, 8, 3));  // prior jumps
  EXPECT_FALSE(priorSequenceAgrees(10, 40, 40, 42, 8, 3));  // query too far
}
