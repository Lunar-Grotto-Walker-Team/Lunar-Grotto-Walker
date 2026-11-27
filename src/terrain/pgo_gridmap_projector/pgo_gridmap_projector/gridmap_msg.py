"""Construction of grid_map_msgs/GridMap messages from a 2-D elevation array.

The layout follows grid_map's own conventions exactly, so that
`grid_map::GridMapRosConverter::fromMessage` and the `grid_map_filters`
plugins read it without complaint:

  * one layer named "elevation", listed in both `layers` and
    `basic_layers` (an empty `basic_layers` would make every cell invalid,
    because `GridMap::isValid(index, layer)` reads the named layer);
  * `data` is a single `std_msgs/Float32MultiArray` in **column-major**
    order, i.e. the flat index of cell (row, col) is `col * n_rows + row`
    (this is what `matrixEigenCopyToMultiArrayMessage` produces for a
    column-major Eigen matrix);
  * `layout.dim[0]` describes the column axis and `layout.dim[1]` the row
    axis, with the label strings "column_index" / "row_index" taken from
    `grid_map::storageIndexNames`.
"""
from __future__ import annotations

import array as array_mod

from typing import Any

import numpy as np
from builtin_interfaces.msg import Time
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

from .rasterizer import GridSpec


GRIDMAP_COLUMN_MAJOR = True

_LAYER_NAME = "elevation"
_COLUMN_LABEL = "column_index"
_ROW_LABEL = "row_index"


def build_gridmap_msg(elevation: np.ndarray, spec: GridSpec, frame_id: str,
                      stamp: Any, extra_layers: dict | None = None) -> GridMap:
    """Build a GridMap from a (n_rows, n_cols) elevation array.

    ``extra_layers`` appends named arrays (e.g. ``{"clearance": ...}``) after
    the elevation layer; they share the geometry and the column-major packing.

    Args:
        elevation: 2-D array, `arr[row, col] = z`; NaN marks empty cells.
        spec: the grid geometry the array was produced with.
        frame_id: frame to stamp on the message header.
        stamp: message stamp (a `std_msgs/Time` or an int nanosecond value).

    Returns:
        A `grid_map_msgs.msg.GridMap` with a single "elevation" layer.
    """
    elevation = np.asarray(elevation, dtype=np.float64)
    # Validate the dimensionality FIRST: unpacking `elevation.shape` into
    # two names raises an opaque "not enough values to unpack" for a 1-D or
    # 3-D input, which tells the caller nothing about what was expected.
    if elevation.ndim != 2:
        raise ValueError(
            f"elevation must be a 2-D (n_rows, n_cols) array, got a "
            f"{elevation.ndim}-D array with shape {elevation.shape}"
        )
    n_rows, n_cols = elevation.shape
    if (n_rows, n_cols) != (spec.n_rows, spec.n_cols):
        raise ValueError(
            f"elevation shape {elevation.shape} does not match grid spec "
            f"({spec.n_rows}, {spec.n_cols})"
        )

    msg = GridMap()
    msg.header.frame_id = frame_id
    if isinstance(stamp, Time):
        msg.header.stamp = stamp
    else:
        msg.header.stamp = Time(sec=int(stamp) // 10**9,
                                nanosec=int(stamp) % 10**9)

    msg.info.resolution = spec.resolution
    msg.info.length_x = spec.length_x
    msg.info.length_y = spec.length_y
    msg.info.pose.position.x = spec.origin_x
    msg.info.pose.position.y = spec.origin_y
    msg.info.pose.position.z = 0.0
    msg.info.pose.orientation.x = 0.0
    msg.info.pose.orientation.y = 0.0
    msg.info.pose.orientation.z = 0.0
    msg.info.pose.orientation.w = 1.0

    names = [_LAYER_NAME]
    msg.basic_layers = [_LAYER_NAME]

    # grid_map stores its matrices column-major, so the flat buffer is
    # ordered by column first: the flat index of cell (row, col) is
    # col * n_rows + row. numpy's Fortran-order flatten of the (n_rows,
    # n_cols) array produces exactly that ordering.
    #
    # Do NOT write this as `elevation.T.reshape(-1, order="F")` — the
    # transpose changes the meaning and lands the value at the wrong index
    # (373*500 + 246 becomes 246*500 + 373). Verified: with the sentinel
    # at cell (246, 373), `flatten(order="F")` puts it at 186746 and
    # `elevation.T.reshape(-1, order="F")` puts it at 123373.
    #
    # `GRIDMAP_COLUMN_MAJOR` documents (and drives) the storage order; it
    # is asserted to be True by the test suite, so it cannot drift away
    # from grid_map's actual layout without turning the suite red.
    flat = (elevation.flatten(order="F") if GRIDMAP_COLUMN_MAJOR
            else elevation.flatten(order="C"))

    # In ROS 2 Python, msg.data is a plain list, NOT a sequence with .add()
    # (that is the ROS 1 / C++ style). Build the Float32MultiArray
    # explicitly and assign the list.
    data = Float32MultiArray()
    dim0 = MultiArrayDimension()
    dim0.label = _COLUMN_LABEL
    dim0.size = n_cols
    dim0.stride = n_rows * n_cols
    dim1 = MultiArrayDimension()
    dim1.label = _ROW_LABEL
    dim1.size = n_rows
    dim1.stride = n_rows
    data.layout.dim = [dim0, dim1]
    # array.array('f'), not .tolist(): at 490k cells the list route costs
    # ~150 ms per frame in Python objects before rclpy converts it again.
    data.data = array_mod.array("f", flat.astype(np.float32).tobytes())
    payload = [data]
    for name, arr in (extra_layers or {}).items():
        extra = Float32MultiArray()
        extra.layout.dim = [dim0, dim1]
        flat_extra = (np.asarray(arr, dtype=np.float64).flatten(order="F")
                      if GRIDMAP_COLUMN_MAJOR
                      else np.asarray(arr, dtype=np.float64).flatten(order="C"))
        extra.data = array_mod.array(
            "f", flat_extra.astype(np.float32).tobytes())
        payload.append(extra)
        names.append(name)
    msg.data = payload

    msg.layers = names
    msg.outer_start_index = 0
    msg.inner_start_index = 0
    return msg
