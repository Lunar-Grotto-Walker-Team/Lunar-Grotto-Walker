"""Node-level checks that need a live rclpy context."""
import rclpy

from elevation_traversability_bridge.bridge_node import TerrainTraversabilityBridge


def test_input_subscription_is_reliable():
    """The bridge must not subscribe BEST_EFFORT.

    /elevation_map_raw_post is ~8 MB per frame. With BEST_EFFORT the DDS
    layer silently drops some samples under load: measured 2026-11-14, the
    publisher ran at 2.0 Hz while this node saw 1.55 Hz, with 55 ms of
    processing and zero lag -- the loss was in delivery, not in the node.
    """
    rclpy.init()
    node = TerrainTraversabilityBridge()
    try:
        assert node._sub.qos_profile.reliability.name == "RELIABLE"
        assert node._sub.qos_profile.depth >= 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
