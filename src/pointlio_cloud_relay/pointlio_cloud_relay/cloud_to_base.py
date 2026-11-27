"""Relay point_lio clouds from the LIO body frame to base_footprint.

``point_lio`` publishes ``/cloud_registered`` in its own body/odometry frame
(``camera_init``/``aft_mapped``).  Nav2 costmaps need the cloud expressed in
``base_footprint``.  This node looks up the *dynamic* TF between those frames
for every cloud and then applies the *static* body->base_footprint extrinsic.

The transform chain per cloud is::

    T^base_footprint_camera = T^base_footprint_body @ T^body_camera
    base_footprint           = T^base_footprint_body @ (dynamic TF) @ camera
"""

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from pointlio_cloud_relay.transform import (
    body_to_base_matrix, compose, transform_points,
)


def quaternion_to_matrix(x, y, z, w):
    """Convert a quaternion to a 3x3 rotation matrix (no scipy needed)."""
    norm_sq = x * x + y * y + z * z + w * w
    if norm_sq < 1e-12:
        return np.eye(3)
    s = 2.0 / norm_sq
    return np.array([
        [1.0 - s * (y * y + z * z), s * (x * y - w * z), s * (x * z + w * y)],
        [s * (x * y + w * z), 1.0 - s * (x * x + z * z), s * (y * z - w * x)],
        [s * (x * z - w * y), s * (y * z + w * x), 1.0 - s * (x * x + y * y)],
    ])


def transform_stamped_to_matrix(transform_stamped):
    """Convert a tf2 ``TransformStamped`` to a homogeneous 4x4 matrix."""
    translation = transform_stamped.transform.translation
    rotation = transform_stamped.transform.rotation

    matrix = np.eye(4)
    matrix[:3, :3] = quaternion_to_matrix(
        rotation.x, rotation.y, rotation.z, rotation.w)
    matrix[:3, 3] = (translation.x, translation.y, translation.z)
    return matrix


def build_cloud_msg(points, frame_id, stamp):
    """Build a ``PointCloud2`` with x/y/z fields from an (N, 3) array.

    ``stamp`` may be ``None`` (unit tests); then ``header.stamp`` is left at
    its zero default.
    """
    header = Header()
    header.frame_id = frame_id
    if stamp is not None:
        header.stamp = stamp

    xyz = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    return pc2.create_cloud_xyz32(header, xyz.tolist())


class CloudToBase(Node):
    """Transform the incoming cloud into ``base_frame`` and republish it."""

    def __init__(self):
        super().__init__('cloud_to_base')

        self.declare_parameter('input_topic', '/cloud_registered')
        self.declare_parameter('output_topic', '/cloud_base')
        self.declare_parameter('body_frame', 'aft_mapped')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('body_to_base_x', 0.187793)
        self.declare_parameter('body_to_base_y', 0.02329)
        self.declare_parameter('body_to_base_z', 0.391836)
        self.declare_parameter('body_to_base_roll', 0.0)
        self.declare_parameter('body_to_base_pitch', 0.2269)
        self.declare_parameter('body_to_base_yaw', 0.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self._body_frame = self.get_parameter('body_frame').value
        self._base_frame = self.get_parameter('base_frame').value

        # Static extrinsic T^base_footprint_body.
        self._T_base_body = body_to_base_matrix(
            self.get_parameter('body_to_base_x').value,
            self.get_parameter('body_to_base_y').value,
            self.get_parameter('body_to_base_z').value,
            self.get_parameter('body_to_base_roll').value,
            self.get_parameter('body_to_base_pitch').value,
            self.get_parameter('body_to_base_yaw').value,
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Sensor-data preset (BEST_EFFORT), equivalent to SensorDataQoS().
        self._publisher = self.create_publisher(
            PointCloud2, output_topic, qos_profile_sensor_data)
        self._subscription = self.create_subscription(
            PointCloud2, input_topic, self._on_cloud, qos_profile_sensor_data)

        self.get_logger().info(
            f'cloud_to_base: {input_topic} -> {output_topic} '
            f'({self._body_frame} -> {self._base_frame})')

    def _on_cloud(self, msg):
        points = pc2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if points.size == 0:
            self.get_logger().debug('cloud_to_base: dropping empty cloud')
            return

        xyz = np.stack(
            (points['x'], points['y'], points['z']), axis=-1
        ).astype(np.float64)

        try:
            # T^body_camera: msg frame expressed in the LIO body frame.
            #
            # Look up the LATEST transform, not the cloud stamp.  Under load
            # point_lio publishes the TF for a sweep a few tens of ms after
            # the cloud; a stamp-exact lookup then extrapolates "into the
            # future" and drops every frame (observed: /cloud_base 0 Hz while
            # the full A+B+C stack ran with RViz).  One TF period (~0.1 s) of
            # pose error is far below the costmap resolution.
            #
            # Deliberately non-blocking: this callback runs in a
            # mutually-exclusive group, so a blocking lookup would keep the
            # TF listener from processing the very /tf message that
            # satisfies it.  A missing transform is dropped and retried on
            # the next cloud.
            transform = self._tf_buffer.lookup_transform(
                self._body_frame, msg.header.frame_id, Time(),
                timeout=Duration(seconds=0.0))
        except TransformException as exc:
            self.get_logger().warn(
                f'cloud_to_base: TF {msg.header.frame_id} -> '
                f'{self._body_frame} unavailable: {exc}',
                throttle_duration_sec=5.0)
            return

        T_base_camera = compose(
            self._T_base_body, transform_stamped_to_matrix(transform))
        points_base = transform_points(xyz, T_base_camera)

        out = build_cloud_msg(
            points_base, self._base_frame, self.get_clock().now().to_msg())
        self._publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CloudToBase()
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
