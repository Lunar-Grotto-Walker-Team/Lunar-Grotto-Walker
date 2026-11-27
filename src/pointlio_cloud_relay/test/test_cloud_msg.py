import numpy as np
from sensor_msgs_py import point_cloud2 as pc2

from pointlio_cloud_relay.cloud_to_base import build_cloud_msg


def test_build_cloud_msg_keeps_points_and_frame():
    pts = np.array([[1.0, 2.0, 3.0], [-1.0, 0.0, 0.5]])
    msg = build_cloud_msg(pts, "base_footprint", None)
    assert msg.header.frame_id == "base_footprint"
    assert msg.height == 1 and msg.width == 2
    got = np.array([[p[0], p[1], p[2]] for p in pc2.read_points(msg)])
    assert np.allclose(got, pts)


def test_build_cloud_msg_empty_is_allowed():
    msg = build_cloud_msg(np.zeros((0, 3)), "base_footprint", None)
    assert msg.width == 0 and msg.height == 1
