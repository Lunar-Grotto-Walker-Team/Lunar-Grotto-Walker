import numpy as np
import pytest
from pgo_gridmap_projector.extent import ExtentConfig, update_extent


def _pts(xs, ys, zs=None):
    zs = zs if zs is not None else [0.0] * len(xs)
    return np.array([[x, y, z] for x, y, z in zip(xs, ys, zs)], dtype=np.float64)


def test_empty_points_return_min_square():
    spec = update_extent(np.zeros((0, 3)), ExtentConfig(resolution=0.1), None)
    assert (spec.length_x, spec.length_y) == (10.0, 10.0)
    assert (spec.origin_x, spec.origin_y) == (0.0, 0.0)


def test_points_inside_min_length_keep_min():
    spec = update_extent(_pts([-1.0, 1.0], [-1.0, 1.0]),
                         ExtentConfig(resolution=0.1), None)
    assert spec.length_x == 10.0


def test_margin_and_quantum_round_outward():
    # span 12.4 + 2*margin = 16.4 -> ceil to quantum 1.0 -> 17.0
    spec = update_extent(_pts([0.0, 12.4], [0.0, 0.0]),
                         ExtentConfig(resolution=0.1), None)
    assert spec.length_x == 17.0
    assert spec.length_y == 10.0


def test_growth_is_monotonic_by_default():
    cfg = ExtentConfig(resolution=0.1)
    prev = update_extent(_pts([0.0, 12.4], [0.0, 0.0]), cfg, None)
    same = update_extent(_pts([0.0, 1.0], [0.0, 0.0]), cfg, prev)
    assert same == prev


def test_lower_left_is_aligned_to_resolution():
    spec = update_extent(_pts([-3.37, 4.91], [-2.03, 1.77]),
                         ExtentConfig(resolution=0.1), None)
    ll_x = spec.origin_x - spec.length_x / 2
    ll_y = spec.origin_y - spec.length_y / 2
    assert abs(ll_x / 0.1 - round(ll_x / 0.1)) < 1e-6
    assert abs(ll_y / 0.1 - round(ll_y / 0.1)) < 1e-6


def test_saturation_caps_to_max_and_keeps_alignment():
    spec = update_extent(_pts([-90.0, 90.0], [0.0, 0.0]),
                         ExtentConfig(resolution=0.1, max_length=120.0), None)
    assert spec.length_x == 120.0
    assert abs(spec.origin_x) < 1e-6
    ll_x = spec.origin_x - spec.length_x / 2
    assert abs(ll_x / 0.1 - round(ll_x / 0.1)) < 1e-6


def test_non_finite_and_out_of_gate_points_are_ignored():
    pts = np.array([[np.nan, 0.0, 0.0], [1e9, 1e9, 0.0], [1.0, 2.0, 5.0]])
    spec = update_extent(pts, ExtentConfig(resolution=0.1), None,
                         z_min=-0.5, z_max=0.5)
    assert spec.length_x == 10.0


def test_single_far_outlier_does_not_move_the_grid():
    """One stray point must not drag the bbox (and the grid) to 1e9."""
    rng = np.random.default_rng(0)
    pts = np.column_stack([rng.uniform(-5, 5, 201), rng.uniform(-5, 5, 201),
                           np.zeros(201)])
    pts = np.vstack([pts, [1e9, 1e9, 0.0]])
    spec = update_extent(pts, ExtentConfig(resolution=0.1), None)
    assert abs(spec.origin_x) < 10.0
    assert abs(spec.origin_y) < 10.0


def test_same_world_point_keeps_its_cell_across_growth():
    """Growing the grid must not shift the cell lattice (Review Focus #2).

    The cell INDEX shifts when the grid grows (the origin moves), so the
    property to pin is the physical extent of the cell holding a given
    world point: its boundaries must be identical before and after growth.
    """
    import math

    pts = _pts([-3.0, 3.0], [-2.0, 2.0])
    first = update_extent(pts, ExtentConfig(resolution=0.1), None)
    grown = update_extent(np.vstack([pts, _pts([20.0], [0.0])]),
                          ExtentConfig(resolution=0.1), first)
    assert (grown.length_x, grown.length_y) != (first.length_x, first.length_y)

    def cell_bounds(spec, x):
        res = spec.resolution
        upper = spec.origin_x + spec.length_x / 2
        idx = math.floor((upper - x) / res)
        return upper - (idx + 1) * res, upper - idx * res

    lo1, hi1 = cell_bounds(first, 0.37)
    lo2, hi2 = cell_bounds(grown, 0.37)
    assert abs(lo1 - lo2) < 1e-6
    assert abs(hi1 - hi2) < 1e-6


# ---------------------------------------------------------------------------
# window_spec: fixed robot-centred window
# ---------------------------------------------------------------------------

def test_window_spec_centres_on_pose_and_keeps_the_length():
    from pgo_gridmap_projector.extent import window_spec
    spec = window_spec((3.4, -2.6), 70.0, 0.10)
    assert spec.length_x == spec.length_y == 70.0
    assert spec.n_rows == spec.n_cols == 700
    # quantum defaults to 1 m, so the centre snaps to whole metres
    assert spec.origin_x == 3.0
    assert spec.origin_y == -3.0


def test_window_spec_quantum_zero_keeps_the_exact_pose():
    from pgo_gridmap_projector.extent import window_spec
    spec = window_spec((3.4, -2.6), 70.0, 0.10, quantum=0.0)
    assert spec.origin_x == 3.4
    assert spec.origin_y == -2.6


def test_window_spec_needs_a_pose():
    from pgo_gridmap_projector.extent import window_spec
    assert window_spec(None, 70.0, 0.10) is None


def test_window_spec_is_independent_of_the_map_bbox():
    # Two poses 50 m apart must give the same size, different origin: that is
    # the whole point of replacing the bbox-driven extent.
    from pgo_gridmap_projector.extent import window_spec
    a = window_spec((0.0, 0.0), 70.0, 0.10)
    b = window_spec((50.0, 0.0), 70.0, 0.10)
    assert (a.n_rows, a.n_cols) == (b.n_rows, b.n_cols) == (700, 700)
    assert (a.origin_x, b.origin_x) == (0.0, 50.0)
