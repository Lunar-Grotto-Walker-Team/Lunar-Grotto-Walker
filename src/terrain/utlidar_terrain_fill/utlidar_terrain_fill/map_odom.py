"""Internal odom->map estimator for the fill node when Terminal C is absent.

Terminal C's go2_driver/map_odom_correction computes

    T_map_odom = T_map_body * T_base_body^-1 * T_odom_base^-1

from /aft_pgo_odom, the driver's /odom and the body_to_base extrinsic.  This
module reproduces the same math from messages that are available without C:

  /aft_pgo_odom       -> T_map_body (camera_init == map)
  /utlidar/robot_pose -> T_odom_ulidar (the Go2 lidar pose in odom)
  /lf/sportmodestate  -> body_height for the driver's base_footprint z

It is used only inside the fill node; it never publishes TF, so it cannot
conflict with C's map_odom_correction when both are running.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


def quat_to_R(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = x * x + y * y + z * z + w * w
    if n <= 0.0:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def rpy_to_R(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ZYX: Rz(yaw) Ry(pitch) Rx(roll), matching map_odom_math.hpp."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def make_T(R: np.ndarray, t) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=float)
    return T


def inv_T(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ t
    return out


def pose_to_T(position, orientation) -> np.ndarray:
    return make_T(
        quat_to_R(orientation.x, orientation.y, orientation.z,
                  orientation.w),
        (position.x, position.y, position.z))


@dataclass(frozen=True)
class MapOdomConfig:
    body_to_base_x: float = 0.187793
    body_to_base_y: float = 0.02329
    body_to_base_z: float = 0.391836
    body_to_base_roll: float = 0.0
    body_to_base_pitch: float = 0.2269
    body_to_base_yaw: float = 0.0
    # driver.cpp: base_footprint.x = robot_pose.x - 0.28945
    utlidar_x_offset: float = 0.28945
    # driver.cpp: body_height_ = sportmodestate.body_height + 0.057 - 0.046825
    body_height_bias: float = 0.057 - 0.046825
    timeout_s: float = 2.0


class MapOdomFallback:
    """Caches the three inputs and composes T_map_odom on demand."""

    def __init__(self, cfg: MapOdomConfig):
        self.cfg = cfg
        self._T_map_body: Optional[np.ndarray] = None
        self._T_odom_ulidar: Optional[np.ndarray] = None
        self._body_height: Optional[float] = None
        self._pgo_recv: Optional[float] = None
        self._pose_recv: Optional[float] = None
        self._height_recv: Optional[float] = None

    def set_pgo(self, T_map_body: np.ndarray, now: float) -> None:
        self._T_map_body = np.array(T_map_body, dtype=float, copy=True)
        self._pgo_recv = float(now)

    def set_robot_pose(self, T_odom_ulidar: np.ndarray, now: float) -> None:
        self._T_odom_ulidar = np.array(T_odom_ulidar, dtype=float, copy=True)
        self._pose_recv = float(now)

    def set_body_height(self, height: float, now: float) -> None:
        self._body_height = float(height)
        self._height_recv = float(now)

    def _fresh(self, recv: Optional[float], now: float) -> bool:
        if recv is None:
            return False
        return (float(now) - recv) <= self.cfg.timeout_s

    def map_odom(self, now: float) -> Optional[np.ndarray]:
        if (self._T_map_body is None or self._T_odom_ulidar is None
                or self._body_height is None):
            return None
        if not (self._fresh(self._pgo_recv, now)
                and self._fresh(self._pose_recv, now)
                and self._fresh(self._height_recv, now)):
            return None

        T_odom_base = np.array(self._T_odom_ulidar, copy=True)
        # The 0.28945 m uLidar->base offset is along the robot/base x axis.
        # Rotate it by the robot orientation before subtracting; subtracting
        # it in odom x is only valid at yaw=0 and produces a yaw-dependent
        # horizontal error of 2*0.28945*sin(|yaw|/2).
        offset = self._T_odom_ulidar[:3, :3] @ np.array(
            [self.cfg.utlidar_x_offset, 0.0, 0.0])
        T_odom_base[0, 3] -= offset[0]
        T_odom_base[1, 3] -= offset[1]
        T_odom_base[2, 3] -= self._body_height + self.cfg.body_height_bias

        T_base_body = make_T(
            rpy_to_R(self.cfg.body_to_base_roll,
                     self.cfg.body_to_base_pitch,
                     self.cfg.body_to_base_yaw),
            (self.cfg.body_to_base_x,
             self.cfg.body_to_base_y,
             self.cfg.body_to_base_z),
        )
        return (self._T_map_body @ inv_T(T_base_body)
                @ inv_T(T_odom_base))
