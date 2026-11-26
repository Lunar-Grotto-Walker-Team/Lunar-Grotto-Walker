"""Build a multi-layer grid_map_msgs/GridMap message.

Storage rules are the same ones ``pgo_gridmap_projector/gridmap_msg.py``
documents: layer data is column-major, the flat index of cell (row, col) is
``col * n_rows + row``, and ``GridMap.data`` carries ONE
``Float32MultiArray`` per layer in ``msg.layers`` order.
"""
from __future__ import annotations

from typing import Any

import numpy as np
from builtin_interfaces.msg import Time
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

_COLUMN_LABEL = "column_index"
_ROW_LABEL = "row_index"
_BASIC_LAYER = "elevation"


def build_multilayer_gridmap(layers: dict[str, np.ndarray], spec: Any,
                             frame_id: str, stamp: Any) -> GridMap:
    """Build a GridMap carrying every array in ``layers``.

    ``spec`` is duck-typed: it only needs ``resolution``, ``length_x``,
    ``length_y``, ``origin_x``, ``origin_y``, ``n_rows`` and ``n_cols``, so
    this module does not depend on the projector package.
    """
    names = list(layers.keys())
    n_rows = int(spec.n_rows)
    n_cols = int(spec.n_cols)

    msg = GridMap()
    msg.header.frame_id = frame_id
    if isinstance(stamp, Time):
        msg.header.stamp = stamp
    else:
        msg.header.stamp = Time(sec=int(stamp) // 10**9,
                                nanosec=int(stamp) % 10**9)

    msg.info.resolution = float(spec.resolution)
    msg.info.length_x = float(spec.length_x)
    msg.info.length_y = float(spec.length_y)
    msg.info.pose.position.x = float(spec.origin_x)
    msg.info.pose.position.y = float(spec.origin_y)
    msg.info.pose.position.z = 0.0
    msg.info.pose.orientation.x = 0.0
    msg.info.pose.orientation.y = 0.0
    msg.info.pose.orientation.z = 0.0
    msg.info.pose.orientation.w = 1.0

    msg.layers = names
    msg.basic_layers = [_BASIC_LAYER] if _BASIC_LAYER in layers else names[:1]

    data = []
    for name in names:
        arr = np.asarray(layers[name], dtype=np.float64)
        if arr.shape != (n_rows, n_cols):
            raise ValueError(
                f"layer {name!r} has shape {arr.shape}, expected "
                f"({n_rows}, {n_cols})")
        flat = arr.flatten(order="F").astype(np.float32)

        array = Float32MultiArray()
        dim0 = MultiArrayDimension()
        dim0.label = _COLUMN_LABEL
        dim0.size = n_cols
        dim0.stride = n_rows * n_cols
        dim1 = MultiArrayDimension()
        dim1.label = _ROW_LABEL
        dim1.size = n_rows
        dim1.stride = n_rows
        array.layout.dim = [dim0, dim1]
        array.data = flat.tolist()
        data.append(array)

    msg.data = data
    msg.outer_start_index = 0
    msg.inner_start_index = 0
    return msg
