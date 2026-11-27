#!/usr/bin/env python3
"""Record Servo's JointTrajectory output and the arm's actual joints, side by side.

Used to explain an unexplained arm motion: with `servo_linear_scale:=0`, Servo
should command the current pose and nothing else. This logs, per Servo message,
the commanded positions next to the latest measured joint positions so any
divergence is visible directly.

Usage (inside self_l4t), before calling takeover:
    python3 scripts/record_servo_traj.py --seconds 25 --out /tmp/traj.csv
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


class Recorder(Node):
    def __init__(self):
        super().__init__("servo_traj_recorder")
        self.latest_measured = None
        self.rows = []
        self.create_subscription(
            JointState, "/feedback/joint_states", self._on_js, 10)
        self.create_subscription(
            JointTrajectory, "/arm_controller/joint_trajectory", self._on_traj, 10)

    def _on_js(self, msg: JointState):
        try:
            self.latest_measured = [msg.position[msg.name.index(j)] for j in JOINTS]
        except ValueError:
            pass

    def _on_traj(self, msg: JointTrajectory):
        if not msg.points:
            return
        p = list(msg.points[0].positions)[:6]
        v = list(msg.points[0].velocities)[:6] if msg.points[0].velocities else []
        self.rows.append((time.time(), p, v, self.latest_measured))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=25.0)
    ap.add_argument("--out", default="/tmp/servo_traj.csv")
    args = ap.parse_args()

    rclpy.init()
    node = Recorder()
    deadline = time.time() + args.seconds
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

    print(f"captured {len(node.rows)} trajectory messages over {args.seconds:.0f}s")
    with open(args.out, "w") as f:
        f.write("t,idx,joint,cmd_pos,meas_pos,delta,cmd_vel\n")
        for t, p, v, m in node.rows:
            for i, j in enumerate(JOINTS):
                meas = m[i] if m and i < len(m) else float("nan")
                d = p[i] - meas
                vel = v[i] if i < len(v) else float("nan")
                f.write(f"{t:.3f},{i},{j},{p[i]:.6f},{meas:.6f},{d:.6f},{vel:.6f}\n")
    print(f"wrote {args.out}")

    if node.rows:
        first = node.rows[0]
        last = node.rows[-1]
        print("\nfirst cmd_pos :", [round(x, 4) for x in first[1]])
        print("last  cmd_pos :", [round(x, 4) for x in last[1]])
        print("max |vel|     :", round(max(
            abs(x) for _, _, v, _ in node.rows for x in v) if any(
            v for _, _, v, _ in node.rows) else 0.0, 4))
    return 0


if __name__ == "__main__":
    sys.exit(main())
