#!/usr/bin/env python3
"""Contract smoke test for the new target-registry publisher.

Runs the fake publisher in a child process, subscribes to
/perception/tracks/unified and checks the published contract:
  - default mode: one active track with position_arm_base / geometry_valid
  - --empty: an empty track list (scan trigger)

This does not start MoveIt/Servo; it pins the tracker-side contract that the
servo consumes.
"""
import argparse
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from yoloe_interfaces.msg import UnifiedTrackArray

TOPIC = "/perception/tracks/unified"


class Probe(Node):
    def __init__(self):
        super().__init__("target_registry_e2e_probe")
        self.msgs = []
        self.create_subscription(UnifiedTrackArray, TOPIC, self._cb, 10)

    def _cb(self, msg):
        self.msgs.append(msg)


def wait_for(probe, timeout_s=8.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and not probe.msgs:
        rclpy.spin_once(probe, timeout_sec=0.1)
    return bool(probe.msgs)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--empty", action="store_true")
    args = ap.parse_args()

    publisher = Path(__file__).with_name("fake_track_publisher.py")
    child = subprocess.Popen(
        [sys.executable, str(publisher), "--rate", "20", "--frame", "arm_base"]
        + (["--empty"] if args.empty else []))
    try:
        rclpy.init()
        probe = Probe()
        if not wait_for(probe):
            print("FAIL: no UnifiedTrackArray within 8 s", file=sys.stderr)
            return 1
        msg = probe.msgs[-1]
        if args.empty:
            if len(msg.tracks) != 0:
                print(f"FAIL: expected empty list, got {len(msg.tracks)}")
                return 1
            print("OK empty list")
            return 0
        if len(msg.tracks) != 1:
            print(f"FAIL: expected 1 track, got {len(msg.tracks)}")
            return 1
        t = msg.tracks[0]
        if t.task_state != "active":
            print(f"FAIL: task_state={t.task_state!r}")
            return 1
        if not t.geometry_valid:
            print("FAIL: geometry_valid is false")
            return 1
        if t.geometry_frame != "arm1/base_link":
            print(f"FAIL: geometry_frame={t.geometry_frame!r}")
            return 1
        if t.position_arm_base.z == 0.0 and t.position_arm_base.x == 0.0:
            print("FAIL: position_arm_base was not populated")
            return 1
        print(
            f"OK track_id={t.track_id} task_state={t.task_state} "
            f"source={t.source} arm_base=({t.position_arm_base.x:.3f},"
            f"{t.position_arm_base.y:.3f},{t.position_arm_base.z:.3f})")
        return 0
    finally:
        child.terminate()
        try:
            child.wait(timeout=3)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait(timeout=3)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
