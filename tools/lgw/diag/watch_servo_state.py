#!/usr/bin/env python3
"""Live view of /lgw/servo/state during a platform-motion (walking) test.

Prints one line per second with the tracking errors and the follower gap
(max |/control - /feedback| over the six arm joints, the same quantity the
servo node's following monitor uses), plus every state transition as it
happens. At the end prints a summary, so a run can be judged without
scrolling the node log.

    python3 scripts/watch_servo_state.py --takeover --seconds 120

--takeover retries /lgw/servo/takeover until TRACKING (the node starts Servo
asynchronously ~7 s after launch), then keeps watching.

Exit code: 0 if the session reached TRACKING (when --takeover was given),
1 otherwise.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from lgw_bt_planner.msg import ServoState
from lgw_bt_planner.srv import Takeover

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


def by_joint(msg):
    if msg is None:
        return None
    table = dict(zip(msg.name, msg.position))
    if not all(j in table for j in JOINTS):
        return None
    return [table[j] for j in JOINTS]


class Watcher(Node):
    def __init__(self) -> None:
        super().__init__("servo_state_watch")
        self.state_msg = None
        self.control = None
        self.feedback = None
        self.transitions = []      # (t, from, to, reason)
        self.last_state = None
        self.last_reason = None
        self.max_pos_err = 0.0
        self.max_ang_err = 0.0
        self.max_gap = 0.0
        self.hold_episodes = 0
        self.samples = 0
        self.t0 = time.time()
        self.create_subscription(
            ServoState, "/lgw/servo/state", self._on_state, 10)
        self.create_subscription(
            JointState, "/control/joint_states", self._on_control, 50)
        self.create_subscription(
            JointState, "/feedback/joint_states", self._on_feedback, 50)
        self.takeover = self.create_client(Takeover, "/lgw/servo/takeover")

    # ---- callbacks ---------------------------------------------------
    def _on_state(self, msg: ServoState) -> None:
        now = time.time() - self.t0
        if msg.state != self.last_state:
            print(f"[{now:7.1f}s] STATE {self.last_state or '-':<10} -> "
                  f"{msg.state:<10} reason={msg.hold_reason}")
            if msg.state == "HOLD":
                self.hold_episodes += 1
            self.transitions.append(
                (now, self.last_state, msg.state, msg.hold_reason))
            self.last_state = msg.state
        elif msg.hold_reason != self.last_reason:
            print(f"[{now:7.1f}s]       reason {self.last_reason} -> "
                  f"{msg.hold_reason}")
        self.last_reason = msg.hold_reason
        self.state_msg = msg

    def _on_control(self, msg: JointState) -> None:
        q = by_joint(msg)
        if q is not None:
            self.control = q

    def _on_feedback(self, msg: JointState) -> None:
        q = by_joint(msg)
        if q is not None:
            self.feedback = q

    # ---- helpers -----------------------------------------------------
    def gap(self) -> float:
        if self.control is None or self.feedback is None:
            return float("nan")
        return max(abs(a - b) for a, b in zip(self.control, self.feedback))

    def spin_for(self, seconds: float) -> None:
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def line(self) -> str:
        m = self.state_msg
        if m is None:
            return "waiting for /lgw/servo/state"
        self.samples += 1
        self.max_pos_err = max(self.max_pos_err, m.position_error_m)
        self.max_ang_err = max(self.max_ang_err, m.angular_error_rad)
        gap = self.gap()
        if gap == gap:            # not NaN
            self.max_gap = max(self.max_gap, gap)
        flags = []
        if m.saturated:
            flags.append("SAT")
        if m.too_close:
            flags.append("CLOSE")
        if m.recovery_exhausted:
            flags.append("EXH")
        return (f"{m.state:<10} src={m.geometry_source:<5} "
                f"pos={m.position_error_m:5.3f}m ang={m.angular_error_rad:5.3f}rad "
                f"dist={m.target_distance_m:5.2f}m gap={gap:5.3f}rad "
                f"{' '.join(flags)}")

    def takeover_session(self, timeout: float = 45.0) -> bool:
        req = Takeover.Request()
        req.track_id = 0
        req.class_filter = ""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if not self.takeover.wait_for_service(timeout_sec=2.0):
                self.spin_for(1.0)
                continue
            future = self.takeover.call_async(req)
            start = time.time()
            while not future.done() and time.time() - start < 10.0:
                rclpy.spin_once(self, timeout_sec=0.05)
            if future.done():
                response = future.result()
                print(f"  takeover: success={response.success} "
                      f"state={response.state} {response.message}")
                if response.success and response.state == "TRACKING":
                    return True
                if response.state == "FAULT":
                    return False
            self.spin_for(1.0)
        print("  takeover: never accepted")
        return False


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="Monitor duration (0 = until Ctrl-C).")
    ap.add_argument("--takeover", action="store_true",
                    help="Call /lgw/servo/takeover first and wait for TRACKING.")
    args = ap.parse_args()

    rclpy.init()
    node = Watcher()
    ok = True
    try:
        if args.takeover:
            print("takeover (this may pre-position the arm with MoveGroup)...")
            ok = node.takeover_session()
            node.spin_for(2.0)
        deadline = (time.time() + args.seconds) if args.seconds > 0 else None
        next_line = time.time() + 1.0
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.time() >= next_line:
                print(f"[{time.time() - node.t0:7.1f}s] {node.line()}")
                next_line += 1.0
            if deadline is not None and time.time() >= deadline:
                break
    except KeyboardInterrupt:
        pass
    finally:
        print("\n--- summary ---")
        for t, src, dst, reason in node.transitions:
            print(f"  {t:7.1f}s  {str(src):<10} -> {dst:<10} reason={reason}")
        print(f"  hold episodes: {node.hold_episodes}")
        print(f"  max pos err {node.max_pos_err:.3f} m, "
              f"max ang err {node.max_ang_err:.3f} rad, "
              f"max gap {node.max_gap:.3f} rad")
        node.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
