"""Pure-numpy rigid-transform helpers for the cloud relay.

No ROS imports here: this module is unit-tested with plain ``python3``.
"""

import numpy as np


def rpy_to_matrix(roll, pitch, yaw):
    """R = Rz(yaw) @ Ry(pitch) @ Rx(roll), returned as a 4x4."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    rx = np.array([[1.0, 0.0, 0.0],
                   [0.0, cr, -sr],
                   [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp],
                   [0.0, 1.0, 0.0],
                   [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0],
                   [sy, cy, 0.0],
                   [0.0, 0.0, 1.0]])

    matrix = np.eye(4)
    matrix[:3, :3] = rz @ ry @ rx
    return matrix


def body_to_base_matrix(x, y, z, roll, pitch, yaw):
    """Static extrinsic T_base_footprint_body (body -> base_footprint)."""
    matrix = rpy_to_matrix(roll, pitch, yaw)
    matrix[:3, 3] = (x, y, z)
    return matrix


def transform_points(points, matrix):
    """Apply a 4x4 rigid transform to an (N, 3) array.

    The result keeps the dtype of ``points`` (float64 in -> float64 out).
    """
    points = np.asarray(points)
    dtype = points.dtype if np.issubdtype(
        points.dtype, np.floating) else np.float64

    xyz = points.astype(np.float64, copy=False).reshape(-1, 3)
    rotation = matrix[:3, :3]
    translation = matrix[:3, 3]
    out = xyz @ rotation.T + translation
    return out.astype(dtype, copy=False)


def compose(T_target_source, T_source_point):
    """Composition of rigid transforms: A @ B (B applied first)."""
    return np.asarray(T_target_source) @ np.asarray(T_source_point)


def quaternion_to_matrix(x, y, z, w):
    """Quaternion (x, y, z, w) -> 3x3 rotation matrix."""
    norm_sq = x * x + y * y + z * z + w * w
    if norm_sq < 1e-12:
        return np.eye(3)
    s = 2.0 / norm_sq
    return np.array([
        [1.0 - s * (y * y + z * z), s * (x * y - w * z), s * (x * z + w * y)],
        [s * (x * y + w * z), 1.0 - s * (x * x + z * z), s * (y * z - w * x)],
        [s * (x * z - w * y), s * (y * z + w * x), 1.0 - s * (x * x + y * y)],
    ])


def transform_to_matrix(translation, rotation):
    """A tf transform (translation xyz, quaternion xyzw) -> 4x4 matrix."""
    matrix = np.eye(4)
    matrix[:3, :3] = quaternion_to_matrix(*rotation)
    matrix[:3, 3] = translation
    return matrix


def cloud_to_base_points(points, T_cloud_body, T_base_body):
    """Express cloud-frame points in the robot base frame.

    ``T_cloud_body`` is the body pose in the cloud frame (what
    ``lookup_transform(cloud_frame, body_frame)`` returns); the points are
    moved into the body frame with its inverse and then through the static
    ``body -> base_footprint`` extrinsic.

    Publishing in the base frame keeps the local costmap's obstacle marks
    free of the drifting ``map -> odom`` correction: the costmap only
    applies the high-rate, continuous ``odom -> base`` transform.
    """
    body = transform_points(points, np.linalg.inv(np.asarray(T_cloud_body)))
    return transform_points(body, np.asarray(T_base_body))
