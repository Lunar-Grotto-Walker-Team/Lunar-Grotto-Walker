#!/usr/bin/env python3
"""Measure command -> real-arm latency with a small step.

The question this answers: how long after a command lands on
/control/joint_states does the arm actually start moving? That number decides
whether the real feedback can be used as Servo's state source.

Method (the same one that measured 283 ms on 2026-10-30, in move_j mode):
  1. read the current feedback for the chosen joint;
  2. publish current + delta on /control/joint_states -- a step, no ramp;
  3. wait for the feedback to move past a detection threshold;
  4. report t_feedback - t_publish, then step back and settle.

Both topics are subscribed in one node, so the two timestamps come from the
same clock and the same executor.

Usage (inside self_l4t, driver running):
    python3 scripts/measure_follow_latency.py --trials 3 --delta 0.03

Safety: keep --delta small (0.03 rad ~ 1.7 deg). In fast_mode the driver turns
the step into an un-smoothed move_js, so this IS the motion the arm makes.
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


class StepProbe(Node):
    def __init__(self, joint, delta, detect_rad, publish_hz):
        super().__init__("follow_latency_probe")
        self.joint = joint
        self.delta = delta
        self.detect_rad = detect_rad
        self.publish_period = 1.0 / publish_hz
        self.latest = None
        self.last_msg = None
        self.pub = self.create_publisher(JointState, "/control/joint_states", 10)
        self.create_subscription(
            JointState, "/feedback/joint_states", self._on_feedback, 50)

    def _on_feedback(self, msg):
        try:
            self.latest = (msg.position[msg.name.index(self.joint)],
                           self.get_clock().now().nanoseconds)
            self.last_msg = msg
        except ValueError:
            pass

    def current_all(self):
        """All six joint angles from the most recent feedback message."""
        msg = self.last_msg
        return [msg.position[msg.name.index(n)] for n in JOINTS]

    def wait_feedback(self, timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest is not None:
                return self.latest
        raise RuntimeError("no /feedback/joint_states")

    def send(self, q):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINTS
        msg.position = list(q)
        self.pub.publish(msg)

    def hold(self, q, seconds):
        deadline = time.time() + seconds
        while time.time() < deadline:
            self.send(q)
            rclpy.spin_once(self, timeout_sec=self.publish_period)

    def step_and_time(self, base_q, target_q):
        """Publish the step, then time how long the feedback takes to react."""
        deadline = time.time() + 0.4
        while time.time() < deadline:
            self.send(base_q)
            rclpy.spin_once(self, timeout_sec=self.publish_period)
        q0, _ = self.wait_feedback()
        t_pub = self.get_clock().now().nanoseconds
        idx = JOINTS.index(self.joint)

        reacted_at = None
        deadline = time.time() + 3.0
        while time.time() < deadline:
            self.send(target_q)
            rclpy.spin_once(self, timeout_sec=self.publish_period)
            if self.latest is not None and abs(self.latest[0] - q0) > self.detect_rad:
                reacted_at = self.latest[1]
                break

        # Keep the target a moment so the arm finishes the small move, then
        # come back and settle before the next trial.
        self.hold(target_q, 0.6)
        self.hold(base_q, 0.8)
        return (None if reacted_at is None
                else (reacted_at - t_pub) / 1e6), idx, q0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--joint", default="joint1")
    ap.add_argument("--delta", type=float, default=0.03)
    ap.add_argument("--trials", type=int, default=3)
    ap.add_argument("--detect-rad", type=float, default=0.005)
    ap.add_argument("--publish-hz", type=float, default=100.0)
    args = ap.parse_args()

    rclpy.init()
    probe = StepProbe(args.joint, args.delta, args.detect_rad, args.publish_hz)
    q, _ = probe.wait_feedback()
    print(f"[latency] start {args.joint}={q:+.4f} delta={args.delta:+.3f} "
          f"detect={args.detect_rad}")

    results = []
    for trial in range(args.trials):
        # Build the command from the arm's CURRENT pose: a step away from the
        # measured position is the only command that cannot also be a jump.
        probe.wait_feedback()
        cur = probe.current_all()
        base_q = list(cur)
        target_q = list(cur)
        idx = JOINTS.index(args.joint)
        sign = 1 if trial % 2 == 0 else -1
        target_q[idx] = cur[idx] + args.delta * sign

        ms, _, q0 = probe.step_and_time(base_q, target_q)
        results.append(ms)
        if ms is None:
            print(f"[latency] trial {trial}: no reaction within 3 s "
                  f"(q0={q0:+.4f})")
        else:
            print(f"[latency] trial {trial}: {ms:7.1f} ms  "
                  f"(q0={q0:+.4f} -> {target_q[idx]:+.4f})")

    good = [r for r in results if r is not None]
    if good:
        good.sort()
        print(f"[latency] n={len(good)} min={good[0]:.1f} "
              f"median={good[len(good) // 2]:.1f} max={good[-1]:.1f} ms")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
