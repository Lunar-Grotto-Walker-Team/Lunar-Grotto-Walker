#!/usr/bin/env python3
"""Verify /lgw/servo/resume re-seeds Servo's cached command after a HOLD move.

Background — docs/servo/2026-11-02-servo-state-monitor-stuck.md §12:

moveit_servo seeds `last_sent_command_` only in `ServoCalcs::start()` and
replays it verbatim on every all-zero twist. While the session is in HOLD,
MoveGroup may move the arm; resuming with only `unpause_servo` would replay
the pre-move pose on the first deadband tick and yank the arm back.

The fix is to call `start_servo` before `unpause_servo` in
`lgw_servo_node::handle_resume`. This script proves it end to end.

Preconditions (inside self_l4t):
  - mock stack up (follow:=false), no agx_arm_ctrl;
  - fake track publisher + static TFs running;
  - /lgw/servo/takeover already succeeded (state TRACKING).

Usage:
    python3 scripts/check_servo_resume_reseed.py \
        --joint-topic /control/joint_states --delta 0.15 --tolerance 0.05
"""
import argparse
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
FRAME = "base_link"


class Check(Node):
    def __init__(self, joint_topic: str) -> None:
        super().__init__("servo_resume_reseed_check")
        self.q = None
        self.first_cmd = None
        self.create_subscription(JointState, joint_topic, self._on_joints, 50)
        self.create_subscription(
            JointTrajectory, "/arm_controller/joint_trajectory", self._on_traj, 50)
        self.move = ActionClient(self, MoveGroup, "/move_action")
        self.pause = self.create_client(Trigger, "/lgw/servo/pause")
        self.resume = self.create_client(Trigger, "/lgw/servo/resume")
        self.twist = self.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.get_params = self.create_client(
            GetParameters, "/lgw_servo_node/get_parameters")

    def _on_joints(self, msg: JointState) -> None:
        by_name = dict(zip(msg.name, msg.position))
        if all(j in by_name for j in JOINTS):
            self.q = [by_name[j] for j in JOINTS]

    def _on_traj(self, msg: JointTrajectory) -> None:
        if self.first_cmd is not None or not msg.points:
            return
        by_name = dict(zip(msg.joint_names, msg.points[0].positions))
        if all(j in by_name for j in JOINTS):
            self.first_cmd = [by_name[j] for j in JOINTS]

    def spin_for(self, seconds: float) -> None:
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def call(self, client, name: str, timeout: float = 10.0) -> bool:
        if not client.wait_for_service(timeout_sec=3.0):
            print(f"FAIL: {name}: service not available")
            return False
        future = client.call_async(Trigger.Request())
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if future.done():
                ok = future.result().success
                print(f"  {name}: success={ok}")
                return ok
        print(f"FAIL: {name}: timeout")
        return False

    def get_double(self, name: str) -> float:
        """Read one double parameter from /lgw_servo_node (-1 on failure)."""
        if not self.get_params.wait_for_service(timeout_sec=3.0):
            print(f"FAIL: get_parameters unavailable ({name})")
            return -1.0
        req = GetParameters.Request()
        req.names = [name]
        future = self.get_params.call_async(req)
        deadline = time.time() + 3.0
        while not future.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done() or not future.result().values:
            print(f"FAIL: get_parameters timeout ({name})")
            return -1.0
        return float(future.result().values[0].double_value)

    def move_to(self, q, velocity_scaling: float = 0.10) -> bool:
        if not self.move.wait_for_server(timeout_sec=5.0):
            print("FAIL: /move_action not available")
            return False
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = velocity_scaling
        goal.request.max_acceleration_scaling_factor = velocity_scaling
        constraints = Constraints()
        for name, pos in zip(JOINTS, q):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = 0.02
            jc.tolerance_below = 0.02
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        goal.request.goal_constraints.append(constraints)

        send = self.move.send_goal_async(goal)
        deadline = time.time() + 10.0
        while not send.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not send.done():
            print("FAIL: move goal not accepted in 10 s")
            return False
        handle = send.result()
        if not handle.accepted:
            print("FAIL: move goal rejected")
            return False
        result = handle.get_result_async()
        deadline = time.time() + 30.0
        while not result.done() and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not result.done():
            print("FAIL: move result timeout")
            return False
        status = result.result().status
        print(f"  move_to: status={status}")
        return status == 4  # GoalStatus.STATUS_SUCCEEDED


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--joint-topic", default="/control/joint_states")
    ap.add_argument("--delta", type=float, default=0.15,
                    help="j2 offset applied by the HOLD-time MoveGroup move.")
    ap.add_argument("--tolerance", type=float, default=0.05,
                    help="Max per-joint |first command - moved pose| (rad).")
    ap.add_argument("--wait-converge-timeout", type=float, default=0.0,
                    help="On the real arm, wait this long for /feedback to reach "
                         "the MoveGroup target before resuming (0 = mock).")
    args = ap.parse_args()

    rclpy.init()
    node = Check(args.joint_topic)
    deadline = time.time() + 5.0
    while node.q is None and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    if node.q is None:
        print(f"FAIL: no joint states on {args.joint_topic}")
        return 2

    q0 = list(node.q)
    q1 = list(q0)
    q1[1] += args.delta
    print("q0:", [round(v, 4) for v in q0])
    print("q1:", [round(v, 4) for v in q1])

    # The control law reads v_max/w_max once at startup (no dynamic parameter
    # callback), so the deterministic zero-command setup MUST come from the
    # launch arguments, not `ros2 param set`:
    #
    #   ros2 launch lgw_bt_planner lgw_full.launch.py ... \
    #       servo_v_max:=0.0 servo_w_max:=0.0
    #
    # With v_max = w_max = 0 every tick is an EXACT zero twist, which is the
    # only path that makes Servo replay last_sent_command_. Otherwise the
    # first post-resume command is a real Cartesian delta computed from the
    # live state monitor, and the cache bug cannot be observed.
    v_max = node.get_double("v_max")
    w_max = node.get_double("w_max")
    if v_max > 0.0 or w_max > 0.0:
        print(f"REFUSED: v_max={v_max} w_max={w_max}; relaunch with "
              "servo_v_max:=0.0 servo_w_max:=0.0 to force the zero-command path.")
        return 2

    if not node.call(node.pause, "lgw/servo/pause"):
        return 1
    node.spin_for(0.3)
    if not node.move_to(q1):
        return 1
    node.spin_for(0.3)
    if args.wait_converge_timeout > 0.0:
        # follow:=true: Servo's cache is seeded from /feedback, so the real
        # arm must actually reach the MoveGroup target before resume.
        deadline = time.time() + args.wait_converge_timeout
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.q is not None and max(
                    abs(a - b) for a, b in zip(node.q, q1)) <= args.tolerance:
                break
        gap = (max(abs(a - b) for a, b in zip(node.q, q1))
               if node.q is not None else float("nan"))
        print(f"  real arm vs q1 gap: {gap:.3f} rad")

    node.first_cmd = None
    if not node.call(node.resume, "lgw/servo/resume"):
        return 1

    # Keep a fresh zero twist in Servo's inbox as well (belt and braces).
    deadline = time.time() + 2.0
    while time.time() < deadline:
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = FRAME
        node.twist.publish(msg)
        node.spin_for(0.05)

    if node.first_cmd is None:
        print("FAIL: Servo produced no output after resume")
        return 1

    deltas = [abs(a - b) for a, b in zip(node.first_cmd, q1)]
    print("first command:", [round(v, 4) for v in node.first_cmd])
    print("|cmd - q1|   :", [round(v, 4) for v in deltas])
    if max(deltas) <= args.tolerance:
        print("PASS: resume re-seeded Servo's cached command.")
        return 0
    print("FAIL: resume replayed the PRE-MOVE command "
          "(cache not re-seeded after the HOLD move).")
    return 1


if __name__ == "__main__":
    sys.exit(main())
