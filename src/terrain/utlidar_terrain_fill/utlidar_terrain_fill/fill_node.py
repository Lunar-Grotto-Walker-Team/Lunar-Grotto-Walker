"""ROS shell for the uLidar near-field ground fill.

Subscribes /utlidar/cloud_deskewed (odom) and /utlidar/robot_pose, extracts
ground points with :mod:`fill_core`, keeps a 5 s **odom** cache, and
publishes /utlidar_terrain_fill in map at 1 Hz.  map<-odom is looked up at the
latest time; a large jump (PGO correction) relocates the cached patch by
re-applying the new transform at publish time instead of clearing it.
"""
from __future__ import annotations

import threading

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from tf2_ros import Buffer, TransformException, TransformListener
from unitree_go.msg import SportModeState

from .fill_core import FillPipeline, GroundConfig
from .map_odom import MapOdomConfig, MapOdomFallback, pose_to_T


def _quat_to_R(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = x * x + y * y + z * z + w * w
    if n <= 0.0:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def _transform_to_matrix(tf) -> np.ndarray:
    t = tf.transform.translation
    q = tf.transform.rotation
    T = np.eye(4)
    T[:3, :3] = _quat_to_R(q.x, q.y, q.z, q.w)
    T[:3, 3] = (t.x, t.y, t.z)
    return T


class UTLidarTerrainFill(Node):
    def __init__(self, **kwargs):
        super().__init__('utlidar_terrain_fill', **kwargs)

        self.declare_parameter('input_topic', '/utlidar/cloud_deskewed')
        self.declare_parameter('pose_topic', '/utlidar/robot_pose')
        self.declare_parameter('output_topic', '/utlidar_terrain_fill')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('source_frame', 'odom')
        self.declare_parameter('window_s', 5.0)
        self.declare_parameter('voxel_leaf', 0.10)
        self.declare_parameter('ground_cell', 0.25)
        self.declare_parameter('ground_grid_size', 12.0)
        self.declare_parameter('ground_percentile', 20.0)
        self.declare_parameter('ground_min_points', 3)
        self.declare_parameter('ground_spread_max', 0.20)
        self.declare_parameter('ground_fill_iterations', 4)
        self.declare_parameter('ground_h_min', -0.10)
        self.declare_parameter('ground_h_max', 0.15)
        self.declare_parameter('max_range_m', 3.0)
        self.declare_parameter('z_offset_m', 0.0)
        # 2026-11-19: publish the ground *surface* (one point per
        # dense_spacing cell) instead of the sparse L2 returns.  See
        # dense_ground_surface() for why.  dense_spacing <= 0 -> voxel_leaf.
        self.declare_parameter('dense_output', True)
        self.declare_parameter('dense_spacing', 0.0)
        self.declare_parameter('pose_jump_trans_m', 0.5)
        self.declare_parameter('pose_jump_rot_rad', 0.3)
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('tf_timeout_s', 0.1)
        # Internal map->odom fallback, used only when Terminal C's TF is
        # absent.  The values mirror pointlio_frames.yaml and driver.cpp.
        self.declare_parameter('pgo_odom_topic', '/aft_pgo_odom')
        self.declare_parameter('body_height_topic', '/lf/sportmodestate')
        self.declare_parameter('body_to_base_x', 0.187793)
        self.declare_parameter('body_to_base_y', 0.02329)
        self.declare_parameter('body_to_base_z', 0.391836)
        self.declare_parameter('body_to_base_roll', 0.0)
        self.declare_parameter('body_to_base_pitch', 0.2269)
        self.declare_parameter('body_to_base_yaw', 0.0)
        self.declare_parameter('utlidar_x_offset', 0.28945)
        self.declare_parameter('body_height_bias', 0.057 - 0.046825)
        self.declare_parameter('fallback_timeout_s', 2.0)

        self._input_topic = str(self.get_parameter('input_topic').value)
        self._pose_topic = str(self.get_parameter('pose_topic').value)
        self._output_topic = str(self.get_parameter('output_topic').value)
        self._target_frame = str(self.get_parameter('target_frame').value)
        self._source_frame = str(self.get_parameter('source_frame').value)
        self._tf_timeout = float(self.get_parameter('tf_timeout_s').value)
        self._pgo_odom_topic = str(
            self.get_parameter('pgo_odom_topic').value)
        self._body_height_topic = str(
            self.get_parameter('body_height_topic').value)

        cfg = GroundConfig(
            grid_size=float(self.get_parameter('ground_grid_size').value),
            cell=float(self.get_parameter('ground_cell').value),
            percentile=float(self.get_parameter('ground_percentile').value),
            min_points=int(self.get_parameter('ground_min_points').value),
            spread_max=float(self.get_parameter('ground_spread_max').value),
            fill_iterations=int(
                self.get_parameter('ground_fill_iterations').value),
            h_min=float(self.get_parameter('ground_h_min').value),
            h_max=float(self.get_parameter('ground_h_max').value),
            max_range=float(self.get_parameter('max_range_m').value),
            voxel_leaf=float(self.get_parameter('voxel_leaf').value),
            window_s=float(self.get_parameter('window_s').value),
            z_offset=float(self.get_parameter('z_offset_m').value),
            jump_trans=float(self.get_parameter('pose_jump_trans_m').value),
            jump_rot=float(self.get_parameter('pose_jump_rot_rad').value),
            dense_output=bool(self.get_parameter('dense_output').value),
            dense_spacing=float(self.get_parameter('dense_spacing').value),
        )
        self._pipeline = FillPipeline(cfg)
        self._fallback = MapOdomFallback(MapOdomConfig(
            body_to_base_x=float(self.get_parameter('body_to_base_x').value),
            body_to_base_y=float(self.get_parameter('body_to_base_y').value),
            body_to_base_z=float(self.get_parameter('body_to_base_z').value),
            body_to_base_roll=float(
                self.get_parameter('body_to_base_roll').value),
            body_to_base_pitch=float(
                self.get_parameter('body_to_base_pitch').value),
            body_to_base_yaw=float(
                self.get_parameter('body_to_base_yaw').value),
            utlidar_x_offset=float(
                self.get_parameter('utlidar_x_offset').value),
            body_height_bias=float(
                self.get_parameter('body_height_bias').value),
            timeout_s=float(
                self.get_parameter('fallback_timeout_s').value),
        ))

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._fallback_lock = threading.Lock()
        # The cloud callback is heavy (ground extraction at 15 Hz); the 1 Hz
        # publish timer must not be starved by a continuously ready
        # subscription queue in a single-threaded executor.
        self._cb_group = ReentrantCallbackGroup()

        self._latest_pose_xy: tuple[float, float] | None = None
        self._sub = self.create_subscription(
            PointCloud2, self._input_topic, self._on_cloud,
            qos_profile_sensor_data, callback_group=self._cb_group)
        self._pose_sub = self.create_subscription(
            PoseStamped, self._pose_topic, self._on_pose, 10)
        self._pgo_sub = self.create_subscription(
            Odometry, self._pgo_odom_topic, self._on_pgo, 10)
        self._body_height_sub = self.create_subscription(
            SportModeState, self._body_height_topic,
            self._on_body_height, 10)
        self._pub = self.create_publisher(
            PointCloud2, self._output_topic, qos_profile_sensor_data)

        rate = max(0.1, float(self.get_parameter('publish_rate_hz').value))
        self._timer = self.create_timer(
            1.0 / rate, self._publish_fill, callback_group=self._cb_group)

        self.get_logger().info(
            f'utlidar_terrain_fill up: {self._input_topic} -> '
            f'{self._output_topic} ({self._source_frame} -> '
            f'{self._target_frame}), window={cfg.window_s}s, '
            f'ground cell={cfg.cell}m, h=[{cfg.h_min},{cfg.h_max}], '
            f'per-scan map transform')

    def _on_pose(self, msg: PoseStamped) -> None:
        self._latest_pose_xy = (
            float(msg.pose.position.x), float(msg.pose.position.y))
        now_s = self.get_clock().now().nanoseconds * 1e-9
        with self._fallback_lock:
            self._fallback.set_robot_pose(
                pose_to_T(msg.pose.position, msg.pose.orientation), now_s)

    def _on_pgo(self, msg: Odometry) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        with self._fallback_lock:
            self._fallback.set_pgo(
                pose_to_T(msg.pose.pose.position, msg.pose.pose.orientation),
                now_s)

    def _on_body_height(self, msg: SportModeState) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        with self._fallback_lock:
            self._fallback.set_body_height(float(msg.body_height), now_s)

    def _get_T(self, now_s: float, warn: bool) -> np.ndarray | None:
        """map<-odom at the current time.

        Prefer C's TF.  Use a zero timeout: this is called at cloud rate, so
        waiting 0.1 s for an absent TF would eat the whole sensor period.
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                self._target_frame, self._source_frame, Time(),
                timeout=Duration(seconds=0.0))
            return _transform_to_matrix(tf)
        except TransformException as exc:
            with self._fallback_lock:
                T = self._fallback.map_odom(now_s)
            if T is None and warn:
                self.get_logger().warn(
                    f'TF {self._target_frame} <- {self._source_frame} '
                    f'unavailable ({exc}) and the internal map->odom '
                    f'fallback lacks fresh /aft_pgo_odom + robot_pose + '
                    f'sportmodestate; dropping scans',
                    throttle_duration_sec=5.0)
            return T

    def _on_cloud(self, msg: PointCloud2) -> None:
        frame = msg.header.frame_id.lstrip('/')
        if frame != self._source_frame:
            self.get_logger().warn(
                f'cloud frame {frame!r} != source_frame '
                f'{self._source_frame!r}; dropping',
                throttle_duration_sec=5.0)
            return
        if self._latest_pose_xy is None:
            self.get_logger().warn(
                'no robot pose yet; dropping cloud',
                throttle_duration_sec=5.0)
            return
        points = pc2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if points.size == 0:
            return
        xyz = np.stack(
            (points['x'], points['y'], points['z']), axis=-1
        ).astype(np.float64)
        now_s = self.get_clock().now().nanoseconds * 1e-9
        T = self._get_T(now_s, warn=False)
        if T is None:
            self.get_logger().warn(
                'no map->odom at scan time; dropping cloud',
                throttle_duration_sec=5.0)
            return
        # Record the current map<-odom.  The buffer stores odom points, so a
        # jump is handled by re-applying this transform at publish time --
        # nothing is cleared (update_tf always returns False now).
        self._pipeline.update_tf(T, now_s)
        added = self._pipeline.add_scan(
            xyz, self._latest_pose_xy, now_s, T)
        self.get_logger().info(
            f'scan: {xyz.shape[0]} points -> {added} odom points',
            throttle_duration_sec=5.0)

    def _publish_fill(self) -> None:
        now_s = self.get_clock().now().nanoseconds * 1e-9
        pts = self._pipeline.build_fill(now_s)
        if pts.shape[0] == 0:
            return
        header = PointCloud2().header
        header.frame_id = self._target_frame
        header.stamp = self.get_clock().now().to_msg()
        out = pc2.create_cloud_xyz32(header, pts.astype(np.float32))
        self._pub.publish(out)
        self.get_logger().info(
            f'published {pts.shape[0]} fill points in {self._target_frame}',
            throttle_duration_sec=5.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UTLidarTerrainFill()
    # Two threads: the 15 Hz cloud/ground path and the 1 Hz timer.  The
    # pipeline protects its short buffer/T operations; the heavy ground
    # extraction stays outside the lock.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
