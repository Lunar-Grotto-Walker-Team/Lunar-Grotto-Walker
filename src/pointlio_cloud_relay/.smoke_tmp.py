import numpy as np
import rclpy
from rclpy.time import Time
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header

from pointlio_cloud_relay.cloud_to_base import CloudToBase, build_cloud_msg

rclpy.init()
node = CloudToBase()

# Publish capture instead of a real publisher.
published = []
node._publisher = type('FakePub', (), {'publish': lambda self, m: published.append(m)})()

stamp = node.get_clock().now().to_msg()

# T^aft_mapped_livox: identity rotation, +1 m in x.
tf = TransformStamped()
tf.header.frame_id = 'aft_mapped'
tf.child_frame_id = 'livox_frame'
tf.header.stamp = stamp
tf.transform.rotation.w = 1.0
tf.transform.translation.x = 1.0
node._tf_buffer.set_transform(tf, 'smoke')

pts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=np.float32)
cloud = build_cloud_msg(pts, 'livox_frame', stamp)
node._on_cloud(cloud)

assert len(published) == 1, len(published)
out = published[0]
assert out.header.frame_id == 'base_footprint'
got = np.array([[p[0], p[1], p[2]] for p in pc2.read_points(out)], dtype=np.float64)
print('in :', pts.tolist())
print('out:', got.tolist())

# Expected: T^base_body @ T^body_cam; cam +x=1 -> body 2,0,0 -> base:
T = node._T_base_body
exp = pts.astype(np.float64) @ T[:3, :3].T + T[:3, 3] + T[:3, :3] @ np.array([1.0, 0.0, 0.0])
assert np.allclose(got, exp, atol=1e-5), (got, exp)
print('smoke OK; T^base_body translation =', T[:3, 3].tolist())

node.destroy_node()
rclpy.shutdown()
