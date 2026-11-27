#!/usr/bin/env python3
"""Guided real-arm verification for HOLD recovery (spec 2026-11-03 §15.1).

Driven by scripts/bench_hold_recovery_real.sh, which does the pre-flight
checks and the human safety prompts. This script never starts/stop the
hardware driver and never publishes static TFs; it only talks to the real
stack (follow:=true) and the fake track publisher.

Two phases, invoked as separate processes so the shell can put a prompt
between the two motions (takeover preposition, then the recovery move):

  --phase prepare   retry /lgw/servo/takeover until TRACKING, settle 3 s,
                    print the joint pose.
  --phase trigger   kill the fake target at the scripted moment, record
                    /lgw/servo/state and /feedback/joint_states, print the
                    transition list and PASS/FAIL.

Scenarios:
  baseline    hold_recovery_enable:=false; kill target; arm must NOT move.
  target_lost hold_recovery_enable:=true + allow_target_lost; target steps
              to B, then the stream dies -> RECOVERING must move the arm.
  release     same as target_lost, but release as soon as RECOVERING is seen.
  saturated   target held at an unreachable point; the node may refuse.
"""
import argparse
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from action_msgs.msg import GoalStatusArray
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from lgw_bt_planner.msg import ServoState
from lgw_bt_planner.srv import Takeover
from yoloe_interfaces.msg import UnifiedTrackArray

JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


class Probe(Node):
    def __init__(self, joint_topic: str) -> None:
        super().__init__("real_recovery_probe")
        self.q = None
        self.track_x = None
        self.track_y = None
        self.move_active = False
        self.recovering_ns = 0
        self.states = []   # (t, state, reason, attempts, exhausted)
        self.t0 = time.time()
        self.create_subscription(ServoState, "/lgw/servo/state", self._on_state, 10)
        self.create_subscription(JointState, joint_topic, self._on_q, 50)
        self.create_subscription(
            UnifiedTrackArray, "/perception/tracks/unified", self._on_tracks, 10)
        self.create_subscription(
            GoalStatusArray, "/move_action/_action/status", self._on_move, 10)
        self.takeover = self.create_client(Takeover, "/lgw/servo/takeover")
        self.release = self.create_client(Trigger, "/lgw/servo/release")

    def _on_state(self, msg: ServoState) -> None:
        key = (msg.state, msg.hold_reason, msg.recovery_attempts,
               msg.recovery_exhausted)
        if self.states and self.states[-1][1:] == key:
            return
        self.states.append((time.time() - self.t0,) + key)
        if msg.state == "RECOVERING" and self.recovering_ns == 0:
            self.recovering_ns = self.get_clock().now().nanoseconds

    def _on_q(self, msg: JointState) -> None:
        by_name = dict(zip(msg.name, msg.position))
        if all(j in by_name for j in JOINTS):
            self.q = [by_name[j] for j in JOINTS]

    def _on_move(self, msg: GoalStatusArray) -> None:
        # 1 = ACCEPTED, 2 = EXECUTING. Only count a goal created after this
        # recovery started; stale statuses from the takeover must not fire.
        self.move_active = any(
            s.status in (1, 2) and
            Time.from_msg(s.goal_info.stamp).nanoseconds >= self.recovering_ns
            for s in msg.status_list)

    def _on_tracks(self, msg: UnifiedTrackArray) -> None:
        if msg.tracks:
            self.track_x = msg.tracks[0].position.x
            self.track_y = msg.tracks[0].position.y

    def spin_for(self, seconds: float) -> None:
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_state(self, state: str, timeout: float) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.states and self.states[-1][1] == state:
                return True
        return False

    def call_response(self, client, request, timeout: float = 15.0):
        if not client.wait_for_service(timeout_sec=3.0):
            return None
        future = client.call_async(request)
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if future.done():
                return future.result()
        return None

    def takeover_session(self) -> bool:
        req = Takeover.Request()
        req.track_id = 0
        req.class_filter = ""
        deadline = time.time() + 40.0
        while time.time() < deadline:
            response = self.call_response(self.takeover, req)
            if response is not None and response.success and response.state == "TRACKING":
                return True
            if response is not None and response.state == "FAULT":
                print(f"FAIL: takeover -> {response.message}")
                return False
            self.spin_for(1.0)
        print("FAIL: takeover never accepted")
        return False

    def do_release(self) -> None:
        self.call_response(self.release, Trigger.Request(), timeout=8.0)


def fmt_q(q) -> str:
    return "[" + " ".join(f"{v:+.3f}" for v in q) + "]"


def phase_prepare(node: Probe) -> int:
    if not node.takeover_session():
        return 1
    node.spin_for(3.0)          # let the preposition + real-arm follow settle
    if node.q is not None:
        print("Q_BEFORE", fmt_q(node.q))
    print("PREPARE_OK")
    return 0


def max_delta(a, b) -> float:
    return max(abs(x - y) for x, y in zip(a, b))


def phase_trigger(node: Probe, args) -> int:
    scenario = args.scenario
    # Let the real arm finish converging to the frozen JTC before sampling
    # the baseline pose; otherwise normal follow-up motion looks like a move.
    node.spin_for(0.5)
    q_before = list(node.q) if node.q is not None else None

    target_b = None
    if scenario == "baseline":
        if args.target_pid > 0:
            try:
                os.kill(args.target_pid, signal.SIGTERM)
                print(f"target {args.target_pid} stopped -> stream dies in ~0.5s")
            except ProcessLookupError:
                print("target already gone")

    elif scenario in ("target_lost", "release"):
        # Publish B first, then kill A: after A is gone B is the only
        # publisher, so the last snapshot before the stream dies is B while
        # the arm is still near A's family (the real arm lags ~1 s).
        target_b = subprocess.Popen(
            [sys.executable, "tools/lgw/bench/fake_track_publisher.py",
             "--x", str(args.target_b_x), "--y", str(args.target_b_y),
             "--z", str(args.target_b_z), "--frame", "arm_base"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        # Wait until B is really publishing: rclpy startup is ~0.5-1 s, and
        # killing it earlier leaves the last-known geometry at A.
        deadline = time.time() + 6.0
        saw_b = False
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
            if node.track_y is not None and abs(node.track_y - args.target_b_y) < 0.05:
                saw_b = True
                break
        if not saw_b:
            print("WARN: target B was never observed; the recovery may see A")
        if args.target_pid > 0:
            try:
                os.kill(args.target_pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        node.spin_for(0.2)    # B is now the only publisher
        target_b.terminate()
        try:
            target_b.wait(timeout=2)
        except subprocess.TimeoutExpired:
            target_b.kill()
        print("target switched to B and stopped -> stream dies in ~0.5s")

    if scenario == "release":
        # Interrupt while the recovery's MoveGroup goal is actually in flight,
        # so the cancel path (not just the pre-settle path) is exercised.
        deadline = time.time() + 8.0
        saw_recovering = False
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if any(s[1] == "RECOVERING" for s in node.states):
                saw_recovering = True
            if saw_recovering and node.move_active:
                print("MoveGroup goal active -> calling release now")
                node.do_release()
                break
        else:
            if saw_recovering:
                print("WARN: no active MoveGroup goal seen; releasing anyway")
                node.do_release()

    node.spin_for(args.monitor_seconds)

    if node.q is not None:
        print("Q_AFTER", fmt_q(node.q))

    states = [s[1] for s in node.states]
    print("states:", " -> ".join(states))
    last = node.states[-1] if node.states else None
    ok = True

    if scenario == "baseline":
        if last is None or last[1] != "HOLD":
            print("FAIL: expected HOLD with recovery disabled")
            ok = False
        if last is not None and last[4]:
            print("FAIL: recovery_exhausted set although the feature is off")
            ok = False
        if "RECOVERING" in states:
            print("FAIL: RECOVERING appeared although recovery is disabled")
            ok = False
        # A real recovery move is >0.15 rad; allow the small residual settle
        # of the real arm toward the frozen JTC.
        if q_before is None or node.q is None or max_delta(q_before, node.q) > 0.10:
            print("FAIL: the arm moved in the baseline scenario")
            ok = False

    elif scenario in ("target_lost", "release"):
        if "RECOVERING" not in states:
            print("FAIL: RECOVERING never observed")
            ok = False
        if scenario == "target_lost":
            if last is None or last[1] != "HOLD" or not last[4]:
                print("FAIL: expected HOLD(exhausted) after the recovery")
                ok = False
            if q_before is None or node.q is None or max_delta(q_before, node.q) <= 0.05:
                print("FAIL: the recovery did not move the arm (>0.05 rad expected)")
                ok = False
        else:
            if last is None or last[1] not in ("IDLE", "RELEASING"):
                print("FAIL: release did not end the session (expected IDLE/RELEASING)")
                ok = False
            # No TRACKING may be entered after the release call.
            release_index = next(
                (i for i, s in enumerate(node.states) if s[1] in ("IDLE", "RELEASING")),
                len(node.states))
            if any(s[1] == "TRACKING" for s in node.states[release_index:]):
                print("FAIL: entered TRACKING after release")
                ok = False

    else:  # saturated
        if last is None or last[1] != "HOLD":
            print("INCONCLUSIVE: no HOLD within the window; move the target "
                  "farther out or raise --monitor-seconds")
            node.do_release()
            print("TRIGGER INCONCLUSIVE")
            return 2
        if last[2] not in ("saturated", "servo_halt"):
            print(f"FAIL: HOLD reason {last[2]}, expected saturated/servo_halt")
            ok = False

    # Always leave Servo paused.
    node.do_release()
    print("TRIGGER", "PASS" if ok else "FAIL")
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", required=True,
                    choices=["baseline", "target_lost", "release", "saturated"])
    ap.add_argument("--phase", required=True, choices=["prepare", "trigger"])
    ap.add_argument("--joint-topic", default="/feedback/joint_states")
    ap.add_argument("--target-pid", type=int, default=0)
    ap.add_argument("--target-b-x", type=float, default=0.65)
    ap.add_argument("--target-b-y", type=float, default=0.30,
                    help="lateral offset; moving only x barely changes the family")
    ap.add_argument("--target-b-z", type=float, default=-0.30)
    ap.add_argument("--monitor-seconds", type=float, default=8.0)
    args = ap.parse_args()

    rclpy.init()
    node = Probe(args.joint_topic)
    try:
        if args.phase == "prepare":
            return phase_prepare(node)
        return phase_trigger(node, args)
    finally:
        pass


if __name__ == "__main__":
    sys.exit(main())
