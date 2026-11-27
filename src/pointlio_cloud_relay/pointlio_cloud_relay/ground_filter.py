"""Split a gravity-aligned cloud into ground and obstacle points.

The local costmap's ObstacleLayer filters points by *absolute* height in its
global frame (``odom``).  On sloped ground that is wrong: a 10 deg ramp puts
the floor 0.1 m above the robot after only 0.57 m, so the layer marks the
slope itself as lethal and DWB refuses to follow a valid global path.

This node classifies each point by its height **above the local ground**
instead:

1. rasterize the cloud into a small local grid centred on the robot
   (default 12 m, 0.5 m cells) in the incoming frame, which must be
   gravity-aligned (``camera_init`` from point_lio);
2. estimate the ground height per cell as a low percentile of z, ignoring
   cells with too few points (a cell that only sees a wall must not take the
   wall as its own ground);
3. fill empty cells from their neighbours (covers the Mid360S ground blind
   zone under the robot);
4. split points into ``ground`` (h <= min_height) and ``obstacles``
   (min_height < h <= max_height); points above max_height (lintels,
   ceilings) are dropped so overhead structure cannot block a 2D costmap.

Both clouds are republished in the input frame with a fresh stamp.  The
local costmap consumes them as two observation sources: ground with
``marking: false, clearing: true`` and obstacles with ``marking: true``.
Ground points therefore keep clearing stale marks without ever becoming
obstacles.
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import Buffer, TransformException, TransformListener


def choose_output_stamp(mode: str, stamp_exact: bool, age_sec: float,
                        tolerance_sec: float, now_stamp, input_stamp):
    """Pick the stamp for the published clouds.

    ``input_stamp`` is the lidar stamp of the sweep.  Keeping it lets the
    downstream costmap look up ``odom->base`` *at the sweep time*, which
    removes the last part of the rotation smear.  That only works when the
    input stamp is in the same clock domain as the local TF -- true on the
    robot (Livox software timestamps are Jetson time) but not in bag replay,
    where the stamps are the recording date.  ``age_sec`` is
    ``now - input_stamp``; ``mode`` is ``auto`` / ``always`` / ``never``.

    ``stamp_exact`` is False when the pose came from the latest-transform
    fallback; such a frame must be re-stamped so the stamp and the pose it
    was rotated with describe the same instant.
    """
    if mode == 'never':
        return now_stamp
    if mode == 'always':
        return input_stamp if stamp_exact else now_stamp
    if stamp_exact and -tolerance_sec <= age_sec <= tolerance_sec:
        return input_stamp
    return now_stamp

from pointlio_cloud_relay.cloud_to_base import build_cloud_msg
from pointlio_cloud_relay.transform import (
    body_to_base_matrix, cloud_to_base_points, transform_to_matrix,
)


def estimate_ground_grid(xyz, origin, cell_size, n, percentile, min_points):
    """Per-cell low percentile of z as the ground height.

    Args:
        xyz: (N, 3) points in a gravity-aligned frame.
        origin: (2,) xy of the grid's lower-left corner.
        cell_size: cell edge in metres.
        n: number of cells per side.
        percentile: 0-100, which z quantile to call "ground".
        min_points: cells with fewer points are left NaN (untrusted).

    Returns:
        (n, n) float64 array, NaN where no trusted ground estimate exists.
    """
    ground = np.full((n, n), np.nan, dtype=np.float64)
    if xyz.shape[0] == 0:
        return ground

    rc = np.floor((xyz[:, :2] - origin) / cell_size).astype(np.int64)
    inside = ((rc[:, 0] >= 0) & (rc[:, 0] < n) &
              (rc[:, 1] >= 0) & (rc[:, 1] < n))
    if not np.any(inside):
        return ground

    z = xyz[inside, 2]
    cell = rc[inside, 0] * n + rc[inside, 1]

    # lexsort's LAST key is the primary one: group by cell first, then sort
    # z inside each cell so the percentile is well defined.
    order = np.lexsort((z, cell))
    cell_s = cell[order]
    z_s = z[order]

    starts = np.flatnonzero(np.r_[True, cell_s[1:] != cell_s[:-1]])
    counts = np.diff(np.r_[starts, cell_s.size])
    valid = counts >= min_points
    if not np.any(valid):
        return ground

    frac = float(np.clip(percentile / 100.0, 0.0, 1.0))
    idx = starts + np.floor(frac * (counts - 1)).astype(np.int64)
    rows = cell_s[starts] // n
    cols = cell_s[starts] % n
    ground[rows[valid], cols[valid]] = z_s[idx[valid]]
    return ground


def fill_missing(ground, iterations):
    """Fill NaN cells with the minimum of their valid 8-neighbours.

    Repeated dilation grows the observed ground into the robot's blind zone
    and into sparse cells.  ``np.fmin`` ignores NaN, so only real estimates
    propagate.  Cells that remain NaN after ``iterations`` are genuinely
    unobserved and their points are dropped by the caller.
    """
    n_rows, n_cols = ground.shape
    for _ in range(int(iterations)):
        pad = np.pad(ground, 1, constant_values=np.nan)
        neigh = np.full_like(ground, np.inf)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                neigh = np.fmin(
                    neigh,
                    pad[1 + dy:n_rows + 1 + dy, 1 + dx:n_cols + 1 + dx])
        fill = np.isnan(ground) & np.isfinite(neigh)
        if not np.any(fill):
            break
        ground[fill] = neigh[fill]
    return ground


def drop_origin_points(xyz, min_range=1e-3):
    """Drop exact-origin padding from a driver's fixed-capacity array.

    The Unitree L2 publishes a fixed array whose unused slots are exact
    ``(0, 0, 0)``: 10000 of 11239 points in every frame we measured.  A real
    return never sits exactly on the acquisition-frame origin, so those are
    padding.  Left in, they collapse into the odom (or camera_init) origin
    cell -- which is where the robot *starts* -- and hijack that cell's ground
    estimate, besides inflating the published ground cloud.
    """
    if xyz.shape[0] == 0:
        return xyz
    return xyz[np.linalg.norm(xyz, axis=1) > min_range]


def split_ground_obstacles(xyz, robot_xy, grid_size, cell_size,
                           percentile, min_points, min_height,
                           fill_iterations, max_height=1.2):
    """Split points into (ground, obstacles) by height above local ground.

    Points above ``max_height`` (door lintels, ceilings, overhangs) are
    dropped rather than classified: a 2 m lintel must not block a 0.4 m
    robot in a 2D costmap. Points outside the grid, or in cells with no
    ground estimate, are dropped as well.
    """
    n = max(1, int(round(grid_size / cell_size)))
    half = 0.5 * n * cell_size
    origin = np.asarray(robot_xy, dtype=np.float64)[:2] - half

    ground = estimate_ground_grid(
        xyz, origin, cell_size, n, percentile, min_points)
    ground = fill_missing(ground, fill_iterations)

    empty = np.zeros((0, 3), dtype=np.float64)
    if xyz.shape[0] == 0:
        return empty, empty

    rc = np.floor((xyz[:, :2] - origin) / cell_size).astype(np.int64)
    inside = ((rc[:, 0] >= 0) & (rc[:, 0] < n) &
              (rc[:, 1] >= 0) & (rc[:, 1] < n))
    sel = np.flatnonzero(inside)
    if sel.size == 0:
        return empty, empty

    ground_z = ground[rc[sel, 0], rc[sel, 1]]
    known = np.isfinite(ground_z)
    sel = sel[known]
    ground_z = ground_z[known]

    height = xyz[sel, 2] - ground_z
    is_ground = height <= min_height
    is_obstacle = (height > min_height) & (height <= max_height)
    return xyz[sel[is_ground]], xyz[sel[is_obstacle]]


class GroundFilter(Node):
    """Publish ground / obstacle point clouds for the local costmap."""

    def __init__(self):
        super().__init__('ground_filter')

        self.declare_parameter('input_topic', '/cloud_registered')
        self.declare_parameter('output_obstacles_topic', '/cloud_obstacles')
        self.declare_parameter('output_ground_topic', '/cloud_ground')
        self.declare_parameter('body_frame', 'aft_mapped')
        self.declare_parameter('grid_size', 12.0)
        self.declare_parameter('cell_size', 0.5)
        self.declare_parameter('ground_percentile', 20.0)
        self.declare_parameter('min_points_per_cell', 2)
        self.declare_parameter('min_height', 0.15)
        self.declare_parameter('max_height', 1.2)
        self.declare_parameter('fill_iterations', 4)
        # Output frame.  The input cloud is in camera_init (SLAM world); the
        # local costmap lives in odom and would have to apply the drifting
        # map->odom correction to every scan, which smears previously marked
        # cells into a fan while the robot rotates.  Publishing in
        # base_footprint instead means the costmap only applies the
        # continuous, high-rate odom->base transform.  Set to "" for the old
        # behaviour (output in the input frame).
        self.declare_parameter('output_frame', 'base_footprint')
        # 2026-11-11: 输出消息盖什么时间戳。
        #   auto   : 输入 stamp 与本地时钟同域(±stamp_match_tolerance_sec)
        #            且位姿是 stamp 精确查到的 -> 保留雷达 stamp，下游 costmap
        #            可以按"扫到那一刻"查 odom->base，去掉处理延迟误差；
        #            否则(离线回放 stamp 是录制日期/走了 latest 回退)重盖 now()。
        #   always : 只要位姿是精确查到的就保留（调试用）。
        #   never  : 始终重盖 now()（旧行为）。
        self.declare_parameter('preserve_input_stamp', 'auto')
        self.declare_parameter('stamp_match_tolerance_sec', 1.0)
        self.declare_parameter('body_to_base_x', 0.187793)
        self.declare_parameter('body_to_base_y', 0.02329)
        self.declare_parameter('body_to_base_z', 0.391836)
        self.declare_parameter('body_to_base_roll', 0.0)
        self.declare_parameter('body_to_base_pitch', 0.2269)
        self.declare_parameter('body_to_base_yaw', 0.0)

        input_topic = self.get_parameter('input_topic').value
        obstacles_topic = self.get_parameter('output_obstacles_topic').value
        ground_topic = self.get_parameter('output_ground_topic').value
        self._body_frame = self.get_parameter('body_frame').value
        self._grid_size = float(self.get_parameter('grid_size').value)
        self._cell_size = float(self.get_parameter('cell_size').value)
        self._percentile = float(self.get_parameter('ground_percentile').value)
        self._min_points = int(self.get_parameter('min_points_per_cell').value)
        self._min_height = float(self.get_parameter('min_height').value)
        self._max_height = float(self.get_parameter('max_height').value)
        self._fill_iterations = int(self.get_parameter('fill_iterations').value)
        self._output_frame = str(self.get_parameter('output_frame').value)
        self._stamp_mode = str(
            self.get_parameter('preserve_input_stamp').value).lower()
        if self._stamp_mode not in ('auto', 'always', 'never'):
            raise ValueError(
                'preserve_input_stamp must be auto, always or never')
        self._stamp_tolerance = float(
            self.get_parameter('stamp_match_tolerance_sec').value)
        self._T_base_body = body_to_base_matrix(
            float(self.get_parameter('body_to_base_x').value),
            float(self.get_parameter('body_to_base_y').value),
            float(self.get_parameter('body_to_base_z').value),
            float(self.get_parameter('body_to_base_roll').value),
            float(self.get_parameter('body_to_base_pitch').value),
            float(self.get_parameter('body_to_base_yaw').value))

        if self._cell_size <= 0.0 or self._grid_size <= 0.0:
            raise ValueError('grid_size and cell_size must be positive')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        #: diagnostics: stamp-exact misses and output-stamp decisions.
        self._stamp_fallbacks = 0
        self._stamp_preserved = 0
        self._stamp_restamped = 0

        self._pub_obstacles = self.create_publisher(
            PointCloud2, obstacles_topic, qos_profile_sensor_data)
        self._pub_ground = self.create_publisher(
            PointCloud2, ground_topic, qos_profile_sensor_data)
        self._sub = self.create_subscription(
            PointCloud2, input_topic, self._on_cloud, qos_profile_sensor_data)

        self.get_logger().info(
            f'ground_filter: {input_topic} -> {obstacles_topic} + '
            f'{ground_topic}, grid {self._grid_size} m @ {self._cell_size} m, '
            f'ground p{self._percentile:.0f}, min_height {self._min_height} m')

    def _on_cloud(self, msg):
        points = pc2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if points.size == 0:
            return
        xyz = np.stack(
            (points['x'], points['y'], points['z']), axis=-1
        ).astype(np.float64)
        xyz = drop_origin_points(xyz)
        if xyz.shape[0] == 0:
            return

        # Robot pose in the cloud frame, i.e. T^cloud_body.
        #
        # Stamp-exact first, latest as the fallback.  The cloud and this TF
        # both come from point_lio and are stamped with the same
        # get_ros_time(lidar_end_time), so the lookup is on one clock and
        # gives the pose of *this* sweep; the latest transform can be one
        # sweep newer, which at 90 deg/s throws far points ~1.6 m outside the
        # walls (local costmap fans outward).  A stamp-exact lookup can still
        # miss when the sweep's TF has not been inserted yet, so fall back
        # instead of dropping the frame (the old behaviour dropped every
        # frame under load).
        #
        # Non-blocking for the same reason as cloud_to_base: a blocking
        # lookup in this callback starves the TF listener that would
        # satisfy it.
        stamp_exact = True
        try:
            transform = self._tf_buffer.lookup_transform(
                msg.header.frame_id, self._body_frame,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.0))
        except TransformException as exc:
            stamp_exact = False
            try:
                transform = self._tf_buffer.lookup_transform(
                    msg.header.frame_id, self._body_frame, Time(),
                    timeout=Duration(seconds=0.0))
            except TransformException as exc2:
                self.get_logger().warn(
                    f'ground_filter: TF {self._body_frame} -> '
                    f'{msg.header.frame_id} unavailable: {exc} / {exc2}',
                    throttle_duration_sec=5.0)
                return
            self._stamp_fallbacks += 1
            self.get_logger().warn(
                f'ground_filter: stamp-exact TF lookup failed ({exc}); '
                f'using the latest transform instead '
                f'({self._stamp_fallbacks} fallbacks since start)',
                throttle_duration_sec=10.0)

        robot_xy = (transform.transform.translation.x,
                    transform.transform.translation.y)

        ground_pts, obstacle_pts = split_ground_obstacles(
            xyz, robot_xy, self._grid_size, self._cell_size,
            self._percentile, self._min_points, self._min_height,
            self._fill_iterations, self._max_height)

        now = self.get_clock().now()
        age = (now - Time.from_msg(msg.header.stamp)).nanoseconds * 1e-9
        stamp = choose_output_stamp(
            self._stamp_mode, stamp_exact, age, self._stamp_tolerance,
            now.to_msg(), msg.header.stamp)
        if stamp is msg.header.stamp:
            self._stamp_preserved += 1
        else:
            self._stamp_restamped += 1
        output_frame = msg.header.frame_id
        if self._output_frame:
            T_cloud_body = transform_to_matrix(
                (transform.transform.translation.x,
                 transform.transform.translation.y,
                 transform.transform.translation.z),
                (transform.transform.rotation.x, transform.transform.rotation.y,
                 transform.transform.rotation.z, transform.transform.rotation.w))
            ground_pts = cloud_to_base_points(
                ground_pts, T_cloud_body, self._T_base_body)
            obstacle_pts = cloud_to_base_points(
                obstacle_pts, T_cloud_body, self._T_base_body)
            output_frame = self._output_frame
        if ground_pts.shape[0] > 0:
            self._pub_ground.publish(build_cloud_msg(
                ground_pts, output_frame, stamp))
        if obstacle_pts.shape[0] > 0:
            self._pub_obstacles.publish(build_cloud_msg(
                obstacle_pts, output_frame, stamp))

        self.get_logger().info(
            f'ground_filter: {xyz.shape[0]} points -> '
            f'{ground_pts.shape[0]} ground, {obstacle_pts.shape[0]} obstacles; '
            f'stamp {self._stamp_mode}: preserved={self._stamp_preserved} '
            f'restamped={self._stamp_restamped} '
            f'fallbacks={self._stamp_fallbacks}',
            throttle_duration_sec=10.0)


def main(args=None):
    rclpy.init(args=args)
    node = GroundFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
