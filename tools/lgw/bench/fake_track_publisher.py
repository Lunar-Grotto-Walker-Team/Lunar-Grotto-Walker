#!/usr/bin/env python3
"""Publish a static or moving UnifiedTrack for bench/integration tests.

The servo consumes one published arm-base point (`position_arm_base`).  Two
modes:

  arm_base (default)  the point is already in arm1/base_link.
  odom                the point is world-fixed in odom; this node converts it
                      to arm1/base_link every tick using the live TF, so the
                      servo sees a current arm-base point while the platform
                      moves.

Use --empty to publish an empty track list (scan trigger) and
--geometry-valid false to publish an undrivable target (HOLD path).
"""
import argparse
import math
import sys
import time
from typing import Optional, Sequence

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from yoloe_interfaces.msg import UnifiedTrack, UnifiedTrackArray

TRACK_ID = 1
CLASS_NAME = "cup"
GEOMETRY_FRAME = "arm1/base_link"


def _quat_matrix(q) -> np.ndarray:
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class FakeTrackPublisher(Node):
    def __init__(self, x: float, y: float, z: float, rate_hz: float,
                 frame: str = "arm_base", step_after: float = 0.0,
                 step_x: Optional[float] = None, sweep_amp: float = 0.0,
                 sweep_hz: float = 0.2,
                 from_base: Optional[Sequence[float]] = None,
                 base_frame: str = "base_link",
                 geometry_valid: bool = True, empty: bool = False):
        super().__init__("fake_track_publisher")
        self._pub = self.create_publisher(
            UnifiedTrackArray, "/perception/tracks/unified", 10)
        if from_base is not None:
            x, y, z = self._base_to_odom(from_base, base_frame)
            frame = "odom"
            self.get_logger().info(
                f"from {base_frame} {tuple(round(v, 3) for v in from_base)} "
                f"-> odom ({x:.3f}, {y:.3f}, {z:.3f})")
        self._p = (x, y, z)
        self._frame = frame
        self._geometry_valid = bool(geometry_valid)
        self._empty = bool(empty)
        self._last_arm_base = None
        self._tf_buffer = None
        self._tf_listener = None
        if frame == "odom":
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
        self._timer = self.create_timer(1.0 / rate_hz, self._tick)
        self._seq = 0
        self._step_after = step_after
        self._step_x = step_x
        self._t_start = self.get_clock().now()
        self._stepped = False
        self._sweep_amp = sweep_amp
        self._sweep_hz = sweep_hz
        self._x0 = x
        self.get_logger().info(
            f"publishing track_id={TRACK_ID} at {frame} {self._p} @ {rate_hz} Hz"
            f" empty={self._empty} geometry_valid={self._geometry_valid}")

    def _base_to_odom(self, p_base: Sequence[float],
                      base_frame: str, timeout: float = 15.0) -> tuple:
        buffer = Buffer()
        listener = TransformListener(buffer, self)  # noqa: F841
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if buffer.can_transform("odom", base_frame, rclpy.time.Time()):
                break
            rclpy.spin_once(self, timeout_sec=0.05)
        else:
            raise RuntimeError(
                f"no odom -> {base_frame} TF after {timeout:.0f} s; start the "
                "tracker or scripts/odom_tf_bridge.py first")

        t = buffer.lookup_transform("odom", base_frame, rclpy.time.Time())
        v = t.transform.translation
        R = _quat_matrix(t.transform.rotation)
        p = R @ np.asarray(p_base, dtype=float) + np.array([v.x, v.y, v.z])
        return (float(p[0]), float(p[1]), float(p[2]))

    def _odom_to_arm_base(self):
        if self._tf_buffer is None:
            return None
        latest = rclpy.time.Time()
        target = GEOMETRY_FRAME
        if not self._tf_buffer.can_transform("odom", target, latest):
            target = "base_link"
            if not self._tf_buffer.can_transform("odom", target, latest):
                return None
        try:
            t = self._tf_buffer.lookup_transform("odom", target, latest)
        except Exception:
            return None
        v = t.transform.translation
        R = _quat_matrix(t.transform.rotation)
        p = R.T @ (np.asarray(self._p, dtype=float) -
                   np.array([v.x, v.y, v.z]))
        return p

    def _tick(self) -> None:
        if self._sweep_amp > 0.0:
            elapsed = (self.get_clock().now() - self._t_start).nanoseconds * 1e-9
            x = self._x0 + self._sweep_amp * math.sin(
                2.0 * math.pi * self._sweep_hz * elapsed)
            self._p = (x, self._p[1], self._p[2])
        if (self._step_x is not None and not self._stepped and
                self._step_after > 0.0):
            elapsed = (self.get_clock().now() - self._t_start).nanoseconds * 1e-9
            if elapsed >= self._step_after:
                self._p = (self._step_x, self._p[1], self._p[2])
                self._stepped = True
                self.get_logger().info(
                    f"stepped target to {self._p} after {elapsed:.1f} s")

        now = self.get_clock().now().to_msg()
        arr = UnifiedTrackArray()
        arr.header.stamp = now
        arr.header.frame_id = "odom"
        arr.wall_stamp = now
        arr.sequence_id = self._seq
        self._seq += 1
        # 2026-11-17: servo follows selected_track_id; an unset id would make
        # every takeover scan-only.
        arr.selected_track_id = 0 if self._empty else TRACK_ID
        if self._empty:
            arr.tracks = []
            self._pub.publish(arr)
            return

        if self._frame == "odom":
            p_arm = self._odom_to_arm_base()
            geometry_valid = self._geometry_valid and p_arm is not None
            if p_arm is None:
                p_arm = (self._last_arm_base if self._last_arm_base is not None
                         else self._p)
            else:
                self._last_arm_base = p_arm
            source = "odom"
        else:
            p_arm = self._p
            geometry_valid = self._geometry_valid
            source = "arm"

        track = UnifiedTrack()
        track.stamp = now
        track.track_id = TRACK_ID
        track.task_state = "active"
        track.class_name = CLASS_NAME
        track.confidence = 0.9
        track.observation_count = 10
        track.confirmed_hits = 10
        track.position = Point(x=self._p[0], y=self._p[1], z=self._p[2])
        track.position_arm_base = Point(
            x=float(p_arm[0]), y=float(p_arm[1]), z=float(p_arm[2]))
        track.geometry_frame = GEOMETRY_FRAME
        track.geometry_valid = bool(geometry_valid)
        track.source = source
        track.last_observation_stamp = now
        track.time_since_last_observation_ms = 0
        arr.tracks = [track]
        self._pub.publish(arr)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--x", type=float, default=0.5)
    ap.add_argument("--y", type=float, default=0.0)
    ap.add_argument("--z", type=float, default=0.3)
    ap.add_argument("--rate", type=float, default=20.0)
    ap.add_argument(
        "--frame", choices=["arm_base", "odom"], default="arm_base")
    ap.add_argument("--step-after", type=float, default=0.0)
    ap.add_argument("--step-x", type=float, default=None)
    ap.add_argument("--sweep-amp", type=float, default=0.0)
    ap.add_argument("--sweep-hz", type=float, default=0.2)
    ap.add_argument("--from-base", nargs=3, type=float, default=None,
                    metavar=("X", "Y", "Z"))
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--geometry-valid", choices=["true", "false"],
                    default="true")
    ap.add_argument("--empty", action="store_true",
                    help="publish an empty track list (scan trigger).")
    args = ap.parse_args()

    rclpy.init()
    node = FakeTrackPublisher(
        args.x, args.y, args.z, args.rate, args.frame,
        args.step_after, args.step_x, args.sweep_amp, args.sweep_hz,
        from_base=args.from_base, base_frame=args.base_frame,
        geometry_valid=(args.geometry_valid == "true"), empty=args.empty)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
