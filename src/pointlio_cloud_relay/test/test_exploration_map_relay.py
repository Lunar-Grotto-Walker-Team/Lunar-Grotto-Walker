import numpy as np
from nav_msgs.msg import OccupancyGrid
from pointlio_cloud_relay.exploration_map_relay import (
    ExplorationMapRelay, convert_and_pool)


def _raw(values):
    return np.asarray(values, dtype=np.uint8)


def test_byte_mapping_handles_int8_reinterpretation():
    # 128/254 arrive as int8 -128/-2 but must be read as uint8.
    # 2026-11-19: SOFT(128) 必须映射为 50(可通行高代价), 不能再当 OCCUPIED。
    raw = _raw([0, 85, 128, 254])
    out, w, h = convert_and_pool(raw, width=4, height=1, pool=1)
    assert out.tolist() == [0, -1, 50, 100]


def test_pooling_soft_wins_over_free_but_not_obstacle():
    raw = _raw([[0, 128], [85, 0]])       # 2x2 block: free + soft + unknown
    out, _, _ = convert_and_pool(raw, width=2, height=2, pool=2)
    assert out.tolist() == [50]


def test_pooling_obstacle_wins_over_soft():
    raw = _raw([[128, 254], [0, 0]])
    out, _, _ = convert_and_pool(raw, width=2, height=2, pool=2)
    assert out.tolist() == [100]


def test_pooling_priority_free_before_unknown():
    raw = _raw([[0, 85], [85, 85]])       # 2x2 block: one free, three unknown
    out, w, h = convert_and_pool(raw, width=2, height=2, pool=2)
    assert (w, h) == (1, 1)
    assert out.tolist() == [0]


def test_pooling_priority_obstacle_wins():
    raw = _raw([[0, 254], [0, 0]])
    out, w, h = convert_and_pool(raw, width=2, height=2, pool=2)
    assert out.tolist() == [100]


def test_pooling_all_unknown_stays_unknown():
    raw = _raw([85] * 4)
    out, w, h = convert_and_pool(raw, width=2, height=2, pool=2)
    assert out.tolist() == [-1]


def test_pooling_crops_partial_block_at_far_edge():
    # 3 cols x 2 rows, pool 2 -> the leftover column is cropped: 1x1
    raw = _raw([0] * 6)
    out, w, h = convert_and_pool(raw, width=3, height=2, pool=2)
    assert (w, h) == (1, 1)
    assert out.tolist() == [0]


def test_process_sets_resolution_and_origin():
    import rclpy
    rclpy.init()
    node = ExplorationMapRelay()
    try:
        msg = OccupancyGrid()
        msg.info.resolution = 0.1
        msg.info.width = 4
        msg.info.height = 4
        msg.info.origin.position.x = -3.0
        msg.info.origin.position.y = -2.0
        msg.data = [0] * 16
        out = node._process(msg)
        assert out.info.resolution == 0.2
        assert out.info.width == 2 and out.info.height == 2
        assert out.info.origin.position.x == -3.0
        assert out.info.origin.position.y == -2.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_process_ignores_empty_grid():
    import rclpy
    rclpy.init()
    node = ExplorationMapRelay()
    try:
        msg = OccupancyGrid()
        msg.info.width = 0
        msg.info.height = 0
        assert node._process(msg) is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_relay_uses_transient_local_qos():
    import rclpy
    rclpy.init()
    node = ExplorationMapRelay()
    try:
        assert node._sub.qos_profile.durability.name == "TRANSIENT_LOCAL"
        assert node._pub.qos_profile.durability.name == "TRANSIENT_LOCAL"
        assert node._sub.qos_profile.reliability.name == "RELIABLE"
    finally:
        node.destroy_node()
        rclpy.shutdown()
