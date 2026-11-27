#!/usr/bin/env python3
"""Regression bench: a zero twist must hold the CURRENT pose, not a stale one.

Background — docs/servo/2026-11-02-servo-state-monitor-stuck.md:

moveit_servo fills `last_sent_command_` exactly once per
`ServoCalcs::start()` from the state monitor, and republishes it verbatim on
every all-zero twist. Our takeover path used to start Servo at boot (arm at
home) and only `unpause_servo` at takeover, so the cache was the all-zero home
command: the first zero twist after a prepositioned takeover commanded zeros
and dragged the arm into the wrist singularity.

`lgw_servo_node` now re-runs `start_servo` after the preposition so the cache
is re-seeded from the post-preposition pose. This script proves the invariant
that makes that fix work, and catches a MoveIt upgrade that changes the
seeding semantics.

What it does (mock or real; zero twist means "hold", so it never commands
motion):
  1. read the measured joints from --joint-topic and refuse to run near the
     all-zero (singular) pose,
  2. pause Servo, re-run start_servo (re-seed), start recording Servo's
     output, unpause,
  3. publish a zero twist with valid stamps for --seconds,
  4. PASS when the first output's positions match the measured pose.

Usage inside self_l4t:
    source /my_ws/l4t/.l4t_env.sh
    python3 scripts/check_servo_zero_command.py \
        --joint-topic /control/joint_states --seconds 2

Exit code 0 = PASS, 1 = FAIL (stale/zero command), 2 = refusal.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

UNPAUSE_SRV = "/servo_node/unpause_servo"
PAUSE_SRV = "/servo_node/pause_servo"
START_SRV = "/servo_node/start_servo"
TWIST_TOPIC = "/servo_node/delta_twist_cmds"
OUTPUT_TOPIC = "/arm_controller/joint_trajectory"
# Servo's robot_link_command_frame, which lgw_full.launch.py pins to base_link.
FRAME = "base_link"


class Probe(Node):
    def __init__(self, joint_topic: str) -> None:
        super().__init__("servo_zero_command_check")
        self.measured = None
        self.first_cmd = None
        self.cmd_count = 0
        self.create_subscription(
            JointState, joint_topic, self._on_joints, 50)
        self.create_subscription(
            JointTrajectory, OUTPUT_TOPIC, self._on_traj, 50)
        self.twist_pub = self.create_publisher(TwistStamped, TWIST_TOPIC, 10)
        self.pause = self.create_client(Trigger, PAUSE_SRV)
        self.unpause = self.create_client(Trigger, UNPAUSE_SRV)
        self.start = self.create_client(Trigger, START_SRV)

    def _on_joints(self, msg: JointState) -> None:
        by_name = dict(zip(msg.name, msg.position))
        if all(j in by_name for j in JOINTS):
            self.measured = [by_name[j] for j in JOINTS]

    def _on_traj(self, msg: JointTrajectory) -> None:
        self.cmd_count += 1
        if self.first_cmd is None and msg.points:
            by_name = dict(zip(msg.joint_names, msg.points[0].positions))
            if all(j in by_name for j in JOINTS):
                self.first_cmd = [by_name[j] for j in JOINTS]

    def call(self, client, name: str, timeout: float = 15.0) -> bool:
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

    def spin_for(self, seconds: float) -> None:
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)


def wait_for_measured(node: Probe, timeout: float = 5.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline and node.measured is None:
        rclpy.spin_once(node, timeout_sec=0.05)
    return node.measured is not None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--joint-topic", default="/control/joint_states",
                    help="Servo's state source: /control (mock) or /feedback (real).")
    ap.add_argument("--seconds", type=float, default=2.0,
                    help="How long to keep a zero twist in Servo's inbox.")
    ap.add_argument("--tolerance", type=float, default=0.05,
                    help="Max per-joint |cmd - measured| (rad) for PASS.")
    args = ap.parse_args()

    rclpy.init()
    node = Probe(args.joint_topic)

    if not wait_for_measured(node):
        print(f"FAIL: no joint states on {args.joint_topic}")
        return 2

    measured = list(node.measured)
    if max(abs(q) for q in measured) < 0.1:
        print("REFUSED: the arm is at the all-zero (wrist-singular) pose; "
              "move it to the posture first. Refusing to test there.")
        print("  measured:", [round(q, 3) for q in measured])
        return 2

    print("measured joints:", [round(q, 3) for q in measured])

    if not node.call(node.pause, "pause_servo"):
        return 1
    node.spin_for(0.2)
    # This is the fix's step: re-seed last_sent_command_ from the live state.
    if not node.call(node.start, "start_servo"):
        return 1
    node.spin_for(0.2)

    node.first_cmd = None
    node.cmd_count = 0
    if not node.call(node.unpause, "unpause_servo"):
        return 1

    deadline = time.time() + args.seconds
    while time.time() < deadline:
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = FRAME
        node.twist_pub.publish(msg)
        node.spin_for(0.05)

    if node.first_cmd is None:
        print(f"FAIL: Servo produced no output on {OUTPUT_TOPIC} "
              f"({node.cmd_count} messages)")
        return 1

    deltas = [abs(a - b) for a, b in zip(node.first_cmd, measured)]
    print("first Servo command:", [round(q, 3) for q in node.first_cmd])
    print("|cmd - measured|    :", [round(d, 3) for d in deltas])
    print(f"outputs seen        : {node.cmd_count}")

    if max(deltas) <= args.tolerance:
        print("PASS: the zero twist held the current pose.")
        return 0
    print("FAIL: the zero twist commanded a STALE pose — Servo's "
          "last_sent_command_ was not re-seeded after the move.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
