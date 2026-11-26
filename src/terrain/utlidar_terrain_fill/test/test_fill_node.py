import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, PointField

from utlidar_terrain_fill.fill_node import UTLidarTerrainFill


def _cloud(points):
    m = PointCloud2()
    m.header.frame_id = "odom"
    m.height = 1
    m.width = len(points)
    m.is_dense = True
    m.fields = [
        PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
        for i, n in enumerate(("x", "y", "z"))
    ]
    m.point_step = 12
    m.row_step = 12 * len(points)
    m.data = b"".join(
        np.asarray(p, dtype=np.float32).tobytes() for p in points)
    return m


class _FakeBuffer:
    def lookup_transform(self, target, source, time, timeout=None):
        from geometry_msgs.msg import TransformStamped
        t = TransformStamped()
        t.header.frame_id = target
        t.child_frame_id = source
        t.transform.rotation.w = 1.0
        return t


class _Capture:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def test_node_publishes_fill_cloud_in_map():
    rclpy.init()
    node = UTLidarTerrainFill(parameter_overrides=[])
    try:
        node._tf_buffer = _FakeBuffer()
        node._pub = _Capture()
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        node._on_pose(pose)
        cloud = _cloud([(x, y, 0.0) for x in np.linspace(-0.5, 0.5, 20)
                        for y in np.linspace(-0.5, 0.5, 20)])
        node._on_cloud(cloud)
        node._publish_fill()
        assert len(node._pub.messages) == 1
        out = node._pub.messages[0]
        assert out.header.frame_id == "map"
        assert out.width > 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


from nav_msgs.msg import Odometry
from tf2_ros import TransformException
from unitree_go.msg import SportModeState


class _RaisingBuffer:
    def lookup_transform(self, target, source, time, timeout=None):
        raise TransformException("map->odom not available")


def test_node_uses_internal_map_odom_fallback():
    rclpy.init()
    node = UTLidarTerrainFill(parameter_overrides=[])
    try:
        node._tf_buffer = _RaisingBuffer()
        node._pub = _Capture()

        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.pose.position.x = 0.28945
        pose.pose.position.z = 0.310175
        pose.pose.orientation.w = 1.0
        node._on_pose(pose)

        pgo = Odometry()
        pgo.header.frame_id = "camera_init"
        pgo.pose.pose.orientation.w = 1.0
        node._on_pgo(pgo)

        state = SportModeState()
        state.body_height = 0.3
        node._on_body_height(state)

        cloud = _cloud([(x, y, 0.0) for x in np.linspace(-0.4, 0.4, 15)
                        for y in np.linspace(-0.4, 0.4, 15)])
        node._on_cloud(cloud)
        node._publish_fill()
        assert len(node._pub.messages) == 1
        out = node._pub.messages[0]
        assert out.header.frame_id == "map"
        assert out.width > 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
