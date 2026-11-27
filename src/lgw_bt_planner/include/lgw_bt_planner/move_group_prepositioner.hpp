// include/lgw_bt_planner/move_group_prepositioner.hpp
//
// Blocking "move the arm to this joint configuration" helper used by the servo
// node at takeover (spec 2026-10-30 §3.8 rev3).
//
// It owns an action client, its own callback group and a local executor, so the
// caller can block on it without starving the 100 Hz control timer:
//
//     // inside handle_takeover, NOT holding the node mutex
//     std::string error;
//     if (!prepositioner_.move_to(q, names, 0.3, 0.02, 5s, error)) { ... }
//
// The service/pause clients in servo_node.cpp use the same pattern for the same
// reason (servo_node.cpp, "Servo lifecycle" comment block).

#ifndef LGW_BT_PLANNER__MOVE_GROUP_PREPOSITIONER_HPP_
#define LGW_BT_PLANNER__MOVE_GROUP_PREPOSITIONER_HPP_

#include <array>
#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/action/move_group.hpp>

namespace lgw_bt_planner
{

// Joint-space goal. Both scalings are written verbatim: 0 means "let MoveIt
// use joint_limits.yaml's default_*_scaling_factor".
//
// Velocity and acceleration are separate ON PURPOSE. Time-optimal
// parameterisation is acceleration-limited for any move shorter than
// v^2 / a, which covers every preposition/recovery move this node makes
// (<= 1.5 rad). Tying acceleration to velocity (the old behaviour) made a
// 0.46 rad recovery take 1.9 s at a = 0.5 rad/s^2; with a = 2.5 rad/s^2 the
// same move takes 0.85 s.
moveit_msgs::action::MoveGroup::Goal make_joint_goal(
  const std::array<double, 6> & joints,
  const std::array<std::string, 6> & names,
  double velocity_scaling,
  double accel_scaling,
  double tolerance_rad,
  const std::string & group_name = "arm");

class MoveGroupPrepositioner
{
public:
  using MoveGroup = moveit_msgs::action::MoveGroup;

  // `group_name` is the MoveIt planning group to command (arm).
  MoveGroupPrepositioner(
    rclcpp::Node & node,
    const std::string & action_name,
    const std::string & group_name);
  ~MoveGroupPrepositioner();

  MoveGroupPrepositioner(const MoveGroupPrepositioner &) = delete;
  MoveGroupPrepositioner & operator=(const MoveGroupPrepositioner &) = delete;

  // Block until the goal succeeds, fails or times out. Returns false and fills
  // `error` on every non-SUCCESS outcome. Must NOT be called while holding a
  // lock the control timer needs.
  bool move_to(
    const std::array<double, 6> & joints,
    const std::array<std::string, 6> & names,
    double velocity_scaling,
    double accel_scaling,
    double tolerance_rad,
    std::chrono::milliseconds timeout,
    std::string & error);

  // Ask the in-flight goal to stop (safe to call from another thread).
  void cancel();

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace lgw_bt_planner

#endif  // LGW_BT_PLANNER__MOVE_GROUP_PREPOSITIONER_HPP_
