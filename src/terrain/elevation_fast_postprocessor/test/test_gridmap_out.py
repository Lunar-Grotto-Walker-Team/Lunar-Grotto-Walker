import numpy as np
from grid_map_msgs.msg import GridMap
from elevation_fast_postprocessor.gridmap_out import build_multilayer_gridmap


class Spec:  # duck-typed stand-in, avoids a cross-package import
    resolution = 0.1
    length_x, length_y = 0.6, 1.0
    origin_x, origin_y = 0.5, -0.3
    n_rows, n_cols = 6, 10


def test_layers_and_data_are_column_major():
    a = np.arange(60, dtype=np.float64).reshape(6, 10)
    b = a * 2
    stamp_ns = 123 * 10**9
    msg = build_multilayer_gridmap({"elevation": a, "slope": b}, Spec(),
                                   "map", stamp_ns)
    assert isinstance(msg, GridMap)
    assert msg.layers == ["elevation", "slope"]
    assert msg.basic_layers == ["elevation"]
    assert len(msg.data) == 2
    assert msg.data[0].data[3 * 6 + 4] == float(a[4, 3])
    assert msg.info.resolution == 0.1
    assert msg.info.length_x == 0.6
    assert msg.info.pose.position.x == 0.5
    assert msg.header.frame_id == "map"
    assert msg.header.stamp.sec == 123
