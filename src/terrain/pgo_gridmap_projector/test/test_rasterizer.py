"""Unit tests for the pure-numpy rasterizer.

No ROS imports: the rasterizer is deliberately separable from the node.
"""
import numpy as np
import pytest

from pgo_gridmap_projector.rasterizer import (
    AGGREGATIONS,
    GridSpec,
    RasterError,
    _aggregate,
    native_available,
    rasterize,
)


SPEC = GridSpec(resolution=0.10, length_x=50.0, length_y=50.0)

# Three points that all land in cell (246, 373):
# x in (0.3, 0.4] -> row 246 ; y in (-12.4, -12.3] -> col 373
THREE_IN_ONE_CELL = [
    (0.35, -12.35, 0.1),
    (0.37, -12.37, 0.3),
    (0.39, -12.39, 0.2),
]


def _xyz(points):
    """points: list of (x, y, z) -> (N, 3) float64 array."""
    return np.asarray(points, dtype=np.float64)


def test_rasterize_returns_per_cell_counts_when_requested():
    spec = GridSpec(resolution=0.1, length_x=1.0, length_y=1.0,
                    origin_x=0.0, origin_y=0.0)
    xyz = _xyz([(0.05, 0.05, 0.0), (0.06, 0.06, 0.0), (0.5, 0.5, 0.0)])
    elev, counts = rasterize(xyz, spec, return_counts=True)
    assert elev.shape == counts.shape == (10, 10)
    assert counts.sum() == 3
    # row = floor((0.5 - x)/0.1): x=0.05 -> 4; x=0.5 -> 0
    assert counts[4, 4] == 2
    assert counts[0, 0] == 1


def test_grid_spec_dimensions():
    assert SPEC.n_rows == 500
    assert SPEC.n_cols == 500
    assert SPEC.half_x == pytest.approx(25.0)
    assert SPEC.half_y == pytest.approx(25.0)


def test_single_point_lands_in_measured_cell():
    # Measured against grid_map::GridMap::getIndex (see plan "Global
    # Constraints"): (0.37, -12.40) -> row 246, col 373.
    out = rasterize(_xyz([(0.37, -12.40, 0.2)]), SPEC)
    assert out.shape == (500, 500)
    assert out[246, 373] == pytest.approx(0.2)
    assert np.count_nonzero(~np.isnan(out)) == 1


def test_empty_grid_is_all_nan():
    out = rasterize(_xyz([]), SPEC)
    assert out.shape == (500, 500)
    assert np.all(np.isnan(out))


def test_median_is_default():
    # Three points in one cell: median is the middle one (0.2).
    out = rasterize(_xyz(THREE_IN_ONE_CELL), SPEC)
    assert out[246, 373] == pytest.approx(0.2)


def test_even_count_median_averages_middle_two():
    out = rasterize(_xyz(THREE_IN_ONE_CELL[:2]), SPEC)
    assert out[246, 373] == pytest.approx(0.2)


@pytest.mark.parametrize("agg,expected", [
    ("median", 0.2), ("mean", 0.2), ("max", 0.3), ("min", 0.1),
])
def test_aggregation_modes(agg, expected):
    out = rasterize(_xyz(THREE_IN_ONE_CELL), SPEC, aggregation=agg)
    assert out[246, 373] == pytest.approx(expected)


@pytest.mark.skipif(not native_available(),
                    reason="pgo_raster_cpp library not built")
def test_native_median_matches_numpy():
    """The C++ kernel must reproduce the numpy median cell for cell."""
    rng = np.random.default_rng(7)
    n = 4000
    pts = np.column_stack([
        rng.uniform(-1.2, 1.2, n),          # some points fall outside
        rng.uniform(-1.2, 1.2, n),
        rng.normal(0.0, 0.4, n),
    ])
    pts[::97, 2] = np.nan                    # non-finite points
    spec = GridSpec(resolution=0.10, length_x=2.0, length_y=2.0,
                    origin_x=0.03, origin_y=0.017)
    z_min, z_max = -0.7, 0.6

    got, cnt = rasterize(pts, spec, aggregation="median",
                         z_min=z_min, z_max=z_max, return_counts=True)

    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    keep = np.isfinite(pts).all(axis=1) & (z >= z_min) & (z <= z_max)
    row = np.floor((spec.origin_x + spec.half_x - x[keep])
                   / spec.resolution).astype(np.int64)
    col = np.floor((spec.origin_y + spec.half_y - y[keep])
                   / spec.resolution).astype(np.int64)
    inside = ((row >= 0) & (row < spec.n_rows)
              & (col >= 0) & (col < spec.n_cols))
    flat = (col * spec.n_rows + row)[inside]
    ref = _aggregate(flat, z[keep][inside],
                     spec.n_rows * spec.n_cols, "median")
    ref = ref.reshape((spec.n_rows, spec.n_cols), order="F")

    np.testing.assert_allclose(got, ref, equal_nan=True, rtol=0, atol=0)
    assert int(cnt.sum()) == int(inside.sum())


@pytest.mark.skipif(not native_available(),
                    reason="pgo_raster_cpp library not built")
def test_native_median_matches_numpy():
    """The C++ kernel must reproduce the numpy median cell for cell."""
    rng = np.random.default_rng(7)
    n = 4000
    pts = np.column_stack([
        rng.uniform(-1.2, 1.2, n),          # some points fall outside
        rng.uniform(-1.2, 1.2, n),
        rng.normal(0.0, 0.4, n),
    ])
    pts[::97, 2] = np.nan                    # non-finite points
    spec = GridSpec(resolution=0.10, length_x=2.0, length_y=2.0,
                    origin_x=0.03, origin_y=0.017)
    z_min, z_max = -0.7, 0.6

    got, cnt = rasterize(pts, spec, aggregation="median",
                         z_min=z_min, z_max=z_max, return_counts=True)

    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    keep = np.isfinite(pts).all(axis=1) & (z >= z_min) & (z <= z_max)
    row = np.floor((spec.origin_x + spec.half_x - x[keep])
                   / spec.resolution).astype(np.int64)
    col = np.floor((spec.origin_y + spec.half_y - y[keep])
                   / spec.resolution).astype(np.int64)
    inside = ((row >= 0) & (row < spec.n_rows)
              & (col >= 0) & (col < spec.n_cols))
    flat = (col * spec.n_rows + row)[inside]
    ref = _aggregate(flat, z[keep][inside],
                     spec.n_rows * spec.n_cols, "median")
    ref = ref.reshape((spec.n_rows, spec.n_cols), order="F")

    np.testing.assert_allclose(got, ref, equal_nan=True, rtol=0, atol=0)
    assert int(cnt.sum()) == int(inside.sum())


def test_points_in_different_cells_stay_separate():
    out = rasterize(_xyz([(0.35, -12.35, 0.4), (5.0, 5.0, 0.2)]), SPEC)
    assert out[246, 373] == pytest.approx(0.4)
    assert out[200, 200] == pytest.approx(0.2)
    assert np.count_nonzero(~np.isnan(out)) == 2


def test_z_limits_drop_points():
    out = rasterize(_xyz([(0.37, -12.40, 0.5)]), SPEC, z_max=0.49)
    assert np.all(np.isnan(out))
    out = rasterize(_xyz([(0.37, -12.40, -0.5)]), SPEC, z_min=-0.49)
    assert np.all(np.isnan(out))


def test_points_outside_window_are_dropped():
    out = rasterize(_xyz([(100.0, 0.0, 0.1), (-100.0, 0.0, 0.1)]), SPEC)
    assert np.all(np.isnan(out))


def test_unknown_aggregation_raises():
    with pytest.raises(RasterError):
        rasterize(_xyz([(0.0, 0.0, 0.0)]), SPEC, aggregation="p95")


def test_supported_aggregations():
    assert "median" in AGGREGATIONS
    assert "max" in AGGREGATIONS


def test_non_square_grid_pins_flat_index_stride():
    # Non-square grid: n_rows (500) != n_cols (300), so the flat cell index
    # stride is observable. grid_map's flat index is col * n_rows + row
    # (column-major), NOT col * n_cols + row.
    spec = GridSpec(resolution=0.10, length_x=50.0, length_y=30.0)
    assert spec.n_rows == 500
    assert spec.n_cols == 300

    # origin (0, 0) -> row = floor((25.0 - 0.37) / 0.1) = 246
    #                col = floor((15.0 - (-5.15)) / 0.1) = 201
    out = rasterize(_xyz([(0.37, -5.15, 0.2)]), spec)
    assert out.shape == (500, 300)
    assert out[246, 201] == pytest.approx(0.2)
    assert np.count_nonzero(~np.isnan(out)) == 1


def test_origin_shifts_the_grid_window():
    # grid_map's getIndexFromPosition uses the grid's CENTRE (mapPosition),
    # so a grid centred on (origin_x, origin_y) must shift its cells.
    spec = GridSpec(resolution=0.10, length_x=50.0, length_y=50.0,
                     origin_x=10.0, origin_y=5.0)
    # row = floor((10.0 + 25.0 - 10.0) / 0.1) = 250
    # col = floor(( 5.0 + 25.0 -  5.0) / 0.1) = 250
    out = rasterize(_xyz([(10.0, 5.0, 0.2)]), spec)
    assert out.shape == (500, 500)
    assert out[250, 250] == pytest.approx(0.2)
    assert np.count_nonzero(~np.isnan(out)) == 1


def test_non_finite_point_is_dropped_and_does_not_contaminate():
    # A NaN x used to cast to a bogus integer index (0 on aarch64) and
    # contaminate a real cell instead of being rejected.
    out = rasterize(_xyz([(np.nan, 0.0, 0.2)]), SPEC)
    assert out.shape == (500, 500)
    assert np.all(np.isnan(out))
    assert np.count_nonzero(~np.isnan(out)) == 0


def test_malformed_empty_input_raises():
    # Shape validation must run before the "no points" early return.
    with pytest.raises(RasterError):
        rasterize(np.zeros((0, 4)), SPEC)
    # A well-formed empty cloud is still valid and yields a blank grid.
    out = rasterize(np.zeros((0, 3)), SPEC)
    assert out.shape == (500, 500)
    assert np.all(np.isnan(out))
