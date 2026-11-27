// tests/test_scan_publish.cpp
// The SCANNING worker must hand the JTC ONE multi-point trajectory (not a
// plan per waypoint), and it must stop that trajectory with a constant-pose
// hold message when the session goes away mid-scan.  Both are observed on the
// real topic the JTC listens to.

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "lgw_bt_planner/servo/servo_node.hpp"

namespace
{

constexpr char kTrajTopic[] = "/arm_controller/joint_trajectory";

// Six-joint chain base_link -> tcp_link.  The node refuses to leave IDLE
// without a parseable model, and the scan itself only needs the joint names.
constexpr char kTestUrdf[] = R"(
<robot name="scan_test_arm">
  <link name="base_link"/>
  <link name="link1"/><joint name="joint1" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="5"/></joint>
  <link name="link2"/><joint name="joint2" type="revolute">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0 0 0.10" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="5"/></joint>
  <link name="link3"/><joint name="joint3" type="revolute">
    <parent link="link2"/><child link="link3"/>
    <origin xyz="0 0 0.10" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="5"/></joint>
  <link name="link4"/><joint name="joint4" type="revolute">
    <parent link="link3"/><child link="link4"/>
    <origin xyz="0 0 0.10" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="5"/></joint>
  <link name="link5"/><joint name="joint5" type="revolute">
    <parent link="link4"/><child link="link5"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/><axis xyz="0 1 0"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="5"/></joint>
  <link name="link6"/><joint name="joint6" type="revolute">
    <parent link="link5"/><child link="link6"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="5"/></joint>
  <link name="tcp_link"/><joint name="tcp_fixed" type="fixed">
    <parent link="link6"/><child link="tcp_link"/>
    <origin xyz="0 0 0.10" rpy="0 0 0"/></joint>
</robot>
)";

// A tiny, fast figure so a whole trajectory takes well under a second: the
// loop test needs to see the NEXT trajectory, not wait out a minute.
rclcpp::NodeOptions short_cycle_options()
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
    rclcpp::Parameter("frame_prefix", "arm1/"),
    rclcpp::Parameter("robot_description", kTestUrdf),
    rclcpp::Parameter("scan_j4_amplitude_rad", 0.05),
    rclcpp::Parameter("scan_j5_amplitude_rad", 0.02),
    rclcpp::Parameter("scan_period_s", 0.4),
    rclcpp::Parameter("scan_traj_duration_s", 0.8),
    rclcpp::Parameter("scan_traj_max_velocity_rad_s", 10.0),
  });
  return opts;
}

rclcpp::NodeOptions idle_options()
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
    rclcpp::Parameter("frame_prefix", "arm1/"),
    rclcpp::Parameter("robot_description", kTestUrdf),
    // A short figure that fits three whole periods into 6 s, so the
    // duration assertion stays deterministic.
    rclcpp::Parameter("scan_period_s", 2.0),
    rclcpp::Parameter("scan_j4_amplitude_rad", 0.1),
    rclcpp::Parameter("scan_j5_amplitude_rad", 0.1),
    rclcpp::Parameter("scan_j1_amplitude_rad", 0.02),
    rclcpp::Parameter("scan_traj_max_velocity_rad_s", 10.0),
    rclcpp::Parameter("scan_traj_duration_s", 6.0),
  });
  return opts;
}

double seconds_of(const builtin_interfaces::msg::Duration & d)
{
  return static_cast<double>(d.sec) + 1e-9 * static_cast<double>(d.nanosec);
}

bool wait_for(
  const std::shared_ptr<rclcpp::Node> & sub,
  const std::function<bool()> & done, double timeout_s)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(static_cast<int64_t>(timeout_s * 1000.0));
  while (std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(sub);
    if (done()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return done();
}

}  // namespace

TEST(ScanPublish, PublishesOneMultiPointTrajectory)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<lgw_bt_planner::ServoNode>(idle_options());
  node->skip_ingress_for_test();

  auto sub = std::make_shared<rclcpp::Node>("scan_publish_probe");
  std::vector<trajectory_msgs::msg::JointTrajectory> received;
  auto subscription = sub->create_subscription<
    trajectory_msgs::msg::JointTrajectory>(
    kTrajTopic, rclcpp::SystemDefaultsQoS(),
    [&received](trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
      received.push_back(*msg);
    });

  bool scan_only = false;
  ASSERT_TRUE(node->takeover_for_test(0, "", scan_only));
  ASSERT_TRUE(scan_only);

  ASSERT_TRUE(wait_for(sub, [&received]() {return !received.empty();}, 6.0))
    << "the scan never published a trajectory";

  const auto & traj = received.front();
  EXPECT_EQ(traj.joint_names.size(), 6u);
  EXPECT_GE(traj.points.size(), 100u)
    << "a continuous scan is one dense trajectory, not a few waypoints";
  EXPECT_NEAR(seconds_of(traj.points.front().time_from_start), 0.0, 1e-6);
  EXPECT_NEAR(seconds_of(traj.points.back().time_from_start), 6.0, 0.1)
    << "one figure at the shipped period";
}

TEST(ScanPublish, StopsTheTrajectoryWhenTheSessionLeaves)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<lgw_bt_planner::ServoNode>(idle_options());
  node->skip_ingress_for_test();

  auto sub = std::make_shared<rclcpp::Node>("scan_stop_probe");
  std::vector<trajectory_msgs::msg::JointTrajectory> received;
  auto subscription = sub->create_subscription<
    trajectory_msgs::msg::JointTrajectory>(
    kTrajTopic, rclcpp::SystemDefaultsQoS(),
    [&received](trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
      received.push_back(*msg);
    });

  bool scan_only = false;
  ASSERT_TRUE(node->takeover_for_test(0, "", scan_only));
  ASSERT_TRUE(wait_for(sub, [&received]() {return !received.empty();}, 6.0));

  // Leave SCANNING the way the release handler does: clear_session cancels
  // the scan (a release must not leave the arm sweeping).
  node->clear_session();

  ASSERT_TRUE(
    wait_for(sub, [&received]() {return received.size() >= 2u;}, 3.0))
  << "no stop trajectory: the JTC would keep running the scan";

  const auto & stop = received.back();
  ASSERT_EQ(stop.points.size(), 2u);
  for (std::size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(stop.points[0].positions[i], stop.points[1].positions[i], 1e-9)
      << "the stop trajectory must hold one pose";
    EXPECT_NEAR(stop.points[1].velocities[i], 0.0, 1e-9);
  }
}

TEST(ScanPublish, KeepsSweepingWhileTheStateStaysScanning)
{
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  auto node = std::make_shared<lgw_bt_planner::ServoNode>(short_cycle_options());
  node->skip_ingress_for_test();

  auto sub = std::make_shared<rclcpp::Node>("scan_loop_probe");
  std::vector<trajectory_msgs::msg::JointTrajectory> received;
  auto subscription = sub->create_subscription<
    trajectory_msgs::msg::JointTrajectory>(
    kTrajTopic, rclcpp::SystemDefaultsQoS(),
    [&received](trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
      received.push_back(*msg);
    });

  std::string message;
  ASSERT_TRUE(node->takeover_service_for_test(0, "", message));
  ASSERT_TRUE(wait_for(sub, [&received]() {return !received.empty();}, 6.0));

  // SCANNING is a state, not one bounded attempt: the next sweep must follow
  // on its own, without a stop message and without leaving the state.
  ASSERT_TRUE(
    wait_for(sub, [&received]() {return received.size() >= 2u;}, 4.0))
  << "the scan stopped after one sweep; SCANNING must keep running";

  const auto & second = received[1];
  EXPECT_GE(second.points.size(), 10u)
    << "a full figure, not a 2-point stop trajectory";
  EXPECT_NEAR(
    seconds_of(second.points.front().time_from_start), 0.0, 1e-6);
  // The figure is periodic and snapped to whole periods: consecutive
  // trajectories start at the same pose, so the seam is continuous.
  for (std::size_t j = 0; j < 6; ++j) {
    EXPECT_NEAR(
      second.points.front().positions[j],
      received[0].points.front().positions[j], 1e-9)
      << "joint " << j << " must chain across trajectories";
  }
}
