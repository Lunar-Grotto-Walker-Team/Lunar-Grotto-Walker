"""GridMap message construction tests.

The layout constants asserted here were measured from the real grid_map
library (see test/gridmap_reference.cpp). This test locks them in so a
future refactor cannot silently shift or transpose the map.
"""
import numpy as np
import pytest
from builtin_interfaces.msg import Time

from pgo_gridmap_projector.gridmap_msg import (
    GRIDMAP_COLUMN_MAJOR,
    build_gridmap_msg,
)
from pgo_gridmap_projector.rasterizer import GridSpec


SPEC = GridSpec(resolution=0.10, length_x=50.0, length_y=50.0)

# A deliberately non-square grid: n_rows != n_cols, so the flat stride
# (col * n_rows + row) and the transposed formula (col * n_cols + row)
# are distinguishable.
RECT_SPEC = GridSpec(resolution=0.10, length_x=5.0, length_y=3.0)


def _msg_with_sentinel():
    elevation = np.full((500, 500), np.nan, dtype=np.float64)
    elevation[246, 373] = 12345.0
    return build_gridmap_msg(elevation, SPEC, "map", 0)


def test_info_fields():
    msg = _msg_with_sentinel()
    assert msg.info.resolution == pytest.approx(0.10)
    assert msg.info.length_x == pytest.approx(50.0)
    assert msg.info.length_y == pytest.approx(50.0)
    assert msg.info.pose.position.x == pytest.approx(0.0)
    assert msg.info.pose.position.y == pytest.approx(0.0)
    assert msg.info.pose.position.z == pytest.approx(0.0)
    assert msg.info.pose.orientation.w == pytest.approx(1.0)


def test_layers_and_basic_layers():
    msg = _msg_with_sentinel()
    assert list(msg.layers) == ["elevation"]
    assert list(msg.basic_layers) == ["elevation"]


def test_start_index_is_zero():
    msg = _msg_with_sentinel()
    assert msg.outer_start_index == 0
    assert msg.inner_start_index == 0


def test_header_frame_id_and_int_nanosecond_stamp():
    """An int stamp is split into sec/nanosec and the frame_id is kept."""
    elevation = np.full((500, 500), np.nan, dtype=np.float64)
    msg = build_gridmap_msg(elevation, SPEC, "odom", 1_500_000_000)
    assert msg.header.frame_id == "odom"
    assert msg.header.stamp.sec == 1
    assert msg.header.stamp.nanosec == 500_000_000


def test_header_stamp_accepts_a_time_instance():
    """A builtin_interfaces/Time is passed through unchanged."""
    elevation = np.full((500, 500), np.nan, dtype=np.float64)
    stamp = Time(sec=7, nanosec=123)
    msg = build_gridmap_msg(elevation, SPEC, "map", stamp)
    assert msg.header.frame_id == "map"
    assert msg.header.stamp.sec == 7
    assert msg.header.stamp.nanosec == 123


def test_gridmap_column_major_constant():
    """The documented storage order must match what grid_map uses."""
    assert GRIDMAP_COLUMN_MAJOR is True


def test_data_layout_is_column_major():
    """Measured: cell (246, 373) sits at flat index 373*500 + 246 = 186746."""
    msg = _msg_with_sentinel()
    data = msg.data[0]
    assert len(data.data) == 500 * 500
    assert data.data[186746] == pytest.approx(12345.0)
    # And nowhere else.
    nonzero = [i for i, v in enumerate(data.data) if v == pytest.approx(12345.0)]
    assert nonzero == [186746]


def test_layout_dim_metadata():
    """Measured from matrixEigenCopyToMultiArrayMessage (column-major path)."""
    msg = _msg_with_sentinel()
    dims = msg.data[0].layout.dim
    assert len(dims) == 2
    assert dims[0].label == "column_index"
    assert dims[0].size == 500
    assert dims[0].stride == 500 * 500
    assert dims[1].label == "row_index"
    assert dims[1].size == 500
    assert dims[1].stride == 500


def test_nan_is_preserved_for_empty_cells():
    msg = _msg_with_sentinel()
    data = msg.data[0]
    assert np.isnan(data.data[0])
    assert data.data[186746] == pytest.approx(12345.0)


def test_non_square_grid_uses_n_rows_as_the_stride():
    """On a rectangular grid the stride is n_rows, not n_cols.

    grid_map's column-major buffer puts cell (row, col) at
    ``col * n_rows + row``. With n_rows == 50 and n_cols == 30 the two
    candidate strides give different answers (50*col + row vs 30*col +
    row), so this test pins the message builder's stride down.
    """
    n_rows, n_cols = RECT_SPEC.n_rows, RECT_SPEC.n_cols
    assert (n_rows, n_cols) == (50, 30)

    elevation = np.full((n_rows, n_cols), np.nan, dtype=np.float64)
    elevation[7, 11] = 42.0
    msg = build_gridmap_msg(elevation, RECT_SPEC, "map", 0)

    data = msg.data[0]
    assert len(data.data) == n_rows * n_cols

    expected = 11 * n_rows + 7  # column-major: col * n_rows + row
    assert data.data[expected] == pytest.approx(42.0)
    nonzero = [i for i, v in enumerate(data.data) if not np.isnan(v)]
    assert nonzero == [expected]

    # The transposed stride would have put the sentinel at 11*30 + 7.
    assert data.data[11 * n_cols + 7] != pytest.approx(42.0)

    # The dim metadata must describe the same rectangular geometry.
    dims = data.layout.dim
    assert dims[0].label == "column_index"
    assert dims[0].size == n_cols
    assert dims[0].stride == n_rows * n_cols
    assert dims[1].label == "row_index"
    assert dims[1].size == n_rows
    assert dims[1].stride == n_rows


def test_non_2d_elevation_raises_a_clear_error():
    """A 1-D or 3-D array must not raise an opaque unpack error."""
    for bad in (np.zeros(10), np.zeros((2, 3, 4))):
        with pytest.raises(ValueError) as excinfo:
            build_gridmap_msg(bad, SPEC, "map", 0)
        message = str(excinfo.value)
        assert "2-D" in message
        assert "elevation" in message


def test_shape_mismatch_raises():
    """A grid whose shape disagrees with the spec is rejected."""
    with pytest.raises(ValueError) as excinfo:
        build_gridmap_msg(np.zeros((100, 100)), SPEC, "map", 0)
    assert "does not match grid spec" in str(excinfo.value)
