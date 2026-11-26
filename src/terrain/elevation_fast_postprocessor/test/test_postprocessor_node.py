import numpy as np
import rclpy
from grid_map_msgs.msg import GridMap
from elevation_fast_postprocessor.postprocessor_node import PostprocessorNode


def _msg_with(elev):
    msg = GridMap()
    msg.info.resolution = 0.1
    msg.info.length_x = elev.shape[0] * 0.1
    msg.info.length_y = elev.shape[1] * 0.1
    msg.header.frame_id = "map"
    msg.layers = ["elevation"]
    msg.basic_layers = ["elevation"]
    from std_msgs.msg import Float32MultiArray
    arr = Float32MultiArray()
    arr.data = elev.flatten(order="F").astype(np.float32).tolist()
    msg.data = [arr]
    return msg


def test_process_adds_derived_layers_and_keeps_validity():
    rclpy.init()
    node = PostprocessorNode()
    try:
        elev = np.zeros((11, 11))
        elev[5:, 5:] = np.nan
        out = node._process(_msg_with(elev))
        assert out is not None
        assert set(out.layers) == {
            "elevation", "elevation_inpainted", "slope", "roughness"}
        assert np.isnan(np.asarray(out.data[0].data, dtype=np.float32)).sum() > 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_process_returns_none_for_all_nan():
    rclpy.init()
    node = PostprocessorNode()
    try:
        assert node._process(_msg_with(np.full((5, 5), np.nan))) is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_node_uses_transient_local_qos():
    rclpy.init()
    node = PostprocessorNode()
    try:
        assert node._sub.qos_profile.durability.name == "TRANSIENT_LOCAL"
        assert node._pub.qos_profile.durability.name == "TRANSIENT_LOCAL"
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_process_returns_none_for_a_grid_without_elevation():
    rclpy.init()
    node = PostprocessorNode()
    try:
        msg = _msg_with(np.zeros((5, 5)))
        msg.layers = ["slope"]
        assert node._process(msg) is None
    finally:
        node.destroy_node()
        rclpy.shutdown()
