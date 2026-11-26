"""Tests for the per-cell ground reference (pure numpy, no rclpy)."""
import numpy as np
import pytest

from pgo_gridmap_projector.ground import (
    GroundConfig, ReferenceResult, _cell_indices, build_reference,
    cave_profile, cell_stats, coarse_spec, coverage_ok, despike_sparse,
    fill_sparse_from_reference, output_band_counts, ring_check, seed_cells,
    seed_z_from_pose, select_ground)
from pgo_gridmap_projector.rasterizer import GridSpec


def _spec(length=10.0, res=0.1):
    return GridSpec(resolution=res, length_x=length, length_y=length,
                    origin_x=0.0, origin_y=0.0)


def _plane(xs, ys, z=0.0):
    return np.array([[x, y, z] for x in xs for y in ys], dtype=float)


def test_coarse_spec_shares_lattice_and_is_five_times_coarser():
    s = coarse_spec(_spec())
    assert s.resolution == 0.5
    assert (s.origin_x, s.origin_y) == (0.0, 0.0)
    assert (s.length_x, s.length_y) == (10.0, 10.0)
    assert s.n_rows == 20 and s.n_cols == 20


def test_coarse_spec_rejects_non_integer_cell_count():
    with pytest.raises(ValueError):
        coarse_spec(_spec(length=10.3))   # 20.6 coarse cells


def test_coarse_spec_rejects_a_resolution_that_does_not_divide_0_5():
    with pytest.raises(ValueError):
        coarse_spec(_spec(res=0.2))       # 0.5 / 0.2 = 2.5, no nesting


def test_cell_stats_flat_floor_p20_p80():
    xyz = _plane(np.linspace(-0.2, 0.2, 9), np.linspace(-0.2, 0.2, 9), 0.0)
    st = cell_stats(xyz, coarse_spec(_spec()))
    occupied = st.counts > 0
    assert occupied.any()
    assert np.allclose(st.p20[occupied], 0.0)
    assert np.allclose(st.p80[occupied], 0.0)


def test_cell_stats_wall_has_large_spread():
    xyz = np.array([[0.05, 0.05, z] for z in np.linspace(0.0, 1.0, 20)])
    st = cell_stats(xyz, coarse_spec(_spec()))
    assert st.counts.max() == 20
    assert np.nanmax(st.p80 - st.p20) > 0.5   # well above upright_spread_max


def test_cell_stats_empty_cell_is_nan_and_points_outside_are_dropped():
    xyz = np.array([[99.0, 99.0, 0.0]])
    st = cell_stats(xyz, coarse_spec(_spec()))
    assert np.all(np.isnan(st.p20))
    assert st.counts.sum() == 0


def test_cave_profile_overrides_only_the_six_values():
    c = GroundConfig()
    cave = cave_profile(c)
    assert (cave.seed_radius, cave.ring_check_tol, cave.ring_outer,
            cave.ref_band_above, cave.out_band_above,
            cave.max_gap_cells) == (0.6, 0.25, 1.2, 0.35, 1.00, 3)
    assert cave.ref_resolution == c.ref_resolution
    assert cave.out_band_below == c.out_band_below


SPEC = coarse_spec(_spec())


def _flat_cloud(radius=3.0, z=0.0):
    xs = np.arange(-radius, radius + 1e-9, 0.05)
    return _plane(xs, xs, z)


def test_seed_z_is_pose_minus_body_height():
    assert seed_z_from_pose((1.0, 2.0, 1.392), GroundConfig()) == pytest.approx(1.0)


def test_ring_accepts_matching_ground():
    st = cell_stats(_flat_cloud(), SPEC)
    r = ring_check(st, SPEC, (0.0, 0.0), 0.0, GroundConfig())
    assert r.ok and r.cells >= 8
    assert r.z == pytest.approx(0.0, abs=1e-9)


def test_ring_trusts_ground_30cm_below_the_seed():
    # A 0.3 m map/pose lag is normal (vis-cache staleness); it must not skip.
    st = cell_stats(_flat_cloud(z=-0.30), SPEC)
    r = ring_check(st, SPEC, (0.0, 0.0), 0.0, GroundConfig())
    assert r.ok
    assert r.z == pytest.approx(-0.30, abs=1e-6)


def _ring_cell_points(count, z, skip=0, n_per_cell=5):
    """Points at the centres of ``count`` coarse cells in the 1.2-3.0 m ring."""
    cells = []
    for row in range(SPEC.n_rows):
        for col in range(SPEC.n_cols):
            x = SPEC.origin_x + SPEC.half_x - (row + 0.5) * SPEC.resolution
            y = SPEC.origin_y + SPEC.half_y - (col + 0.5) * SPEC.resolution
            d = np.hypot(x, y)
            if 1.2 <= d <= 3.0:
                cells.append((x, y))
    chosen = cells[skip:skip + count]
    return np.array([[x, y, z] for (x, y) in chosen
                     for _ in range(n_per_cell)], dtype=float)


def test_ring_picks_the_ground_cluster_not_a_dominant_ceiling():
    # Floor on 10 ring cells, a flat ceiling on 40: the plain median is the
    # ceiling (the corridor failure), the cluster nearest the seed is the floor.
    floor = _ring_cell_points(10, 0.0, skip=0)
    ceiling = _ring_cell_points(40, 1.1, skip=10)
    st = cell_stats(np.vstack([floor, ceiling]), SPEC)
    r = ring_check(st, SPEC, (0.0, 0.0), 0.0, GroundConfig())
    assert r.ok
    assert r.z == pytest.approx(0.0, abs=1e-9)


def test_ring_rejects_a_cluster_far_from_the_seed():
    ring = _ring_cell_points(10, 1.5)
    st = cell_stats(ring, SPEC)
    r = ring_check(st, SPEC, (0.0, 0.0), 0.0, GroundConfig())
    assert not r.ok


def test_ring_does_not_pick_a_ceiling_above_the_feet():
    # 1.5 m cave, pose seed 0.8 m above the map floor: the ceiling (1.5) is
    # nearer (0.7) than the floor (0.8) and would be accepted by a pure
    # distance check.  A floor cannot be 0.7 m ABOVE the feet.
    floor = _ring_cell_points(10, 0.0)
    ceiling = _ring_cell_points(40, 1.5, skip=10)
    st = cell_stats(np.vstack([floor, ceiling]), SPEC)
    r = ring_check(st, SPEC, (0.0, 0.0), 0.8, GroundConfig())
    assert not r.ok


def test_ring_accepts_a_floor_slightly_above_the_pose_seed():
    # A small positive cache lag (map floor 0.2 above the pose estimate) is
    # normal and must not skip.
    st = cell_stats(_flat_cloud(z=0.0), SPEC)
    r = ring_check(st, SPEC, (0.0, 0.0), -0.2, GroundConfig())
    assert r.ok and r.z == pytest.approx(0.0, abs=1e-9)


def test_reference_seed_follows_the_map_ring():
    # The map floor is 0.3 m below the pose-derived seed; the flood must start
    # on the map's floor, not on the pose model of it.
    res = build_reference(_flat_cloud(z=-0.30), POSE, SPEC, GroundConfig())
    assert res.ring_ok
    assert res.seed_z == pytest.approx(-0.30, abs=1e-6)


def test_seed_accepts_ground_below_the_pose_by_the_cache_lag():
    # Floor + ceiling in the same seed cells (a cave column): the ring is
    # spread-filtered out, so the seed must still lock onto the floor 0.5 m
    # below the pose-derived height.  A symmetric +/-0.30 window would miss it
    # and leave the frame without any seed.
    floor = _flat_cloud(radius=1.2, z=-0.5)
    ceiling = _flat_cloud(radius=1.2, z=1.0)
    st = cell_stats(np.vstack([floor, ceiling]), SPEC)
    ref, acc, pending, _ = seed_cells(st, SPEC, (0.0, 0.0), 0.0, GroundConfig())
    assert acc[9, 9] and not pending[9, 9]
    assert ref[9, 9] == pytest.approx(-0.5, abs=1e-6)


def test_ring_ignores_wall_cells():
    # A ring made only of vertical wall points must not be used as ground.
    xyz = np.array([[x, 2.0, z] for x in np.arange(-2, 2, 0.05)
                    for z in np.linspace(0.0, 1.0, 10)])
    st = cell_stats(xyz, SPEC)
    r = ring_check(st, SPEC, (0.0, 0.0), 0.0, GroundConfig())
    assert r.z is None or r.cells == 0


def test_seed_marks_blind_zone_with_pose_prior():
    st = cell_stats(np.zeros((0, 3)), SPEC)
    ref, acc, pending, _ = seed_cells(st, SPEC, (0.0, 0.0), 0.0, GroundConfig())
    assert acc.any() and not pending.any()
    assert np.allclose(ref[acc], 0.0)


def test_seed_cell_with_floor_and_ceiling_is_accepted_using_floor():
    xyz = np.vstack([
        np.array([[0.05, 0.05, 0.0], [0.06, 0.06, 0.01], [0.07, 0.07, 0.02]]),
        np.array([[0.05, 0.05, 1.1], [0.06, 0.06, 1.1]]),
    ])
    st = cell_stats(xyz, SPEC)
    ref, acc, pending, _ = seed_cells(st, SPEC, (0.0, 0.0), 0.0, GroundConfig())
    assert acc[9, 9] and not pending[9, 9]
    assert ref[9, 9] == pytest.approx(0.01, abs=1e-9)


def test_seed_cell_with_only_ceiling_is_pending():
    xyz = np.array([[0.05, 0.05, 1.1], [0.06, 0.06, 1.1], [0.07, 0.07, 1.1]])
    st = cell_stats(xyz, SPEC)
    _, acc, pending, _ = seed_cells(st, SPEC, (0.0, 0.0), 0.0, GroundConfig())
    assert pending[9, 9] and not acc[9, 9]


POSE = (0.0, 0.0, 0.392)   # seed_z = 0


def test_flat_floor_is_fully_connected():
    xyz = _flat_cloud()
    res = build_reference(xyz, POSE, SPEC, GroundConfig())
    assert res.ring_ok
    # Cells with fewer than min_points_ref are legitimately NaN (single
    # stray returns at the cloud boundary must not become ground).
    dense = cell_stats(xyz, SPEC).counts >= GroundConfig().min_points_ref
    assert res.accepted[dense].all()


def test_ceiling_only_column_stays_nan():
    floor = _flat_cloud(radius=3.0, z=0.0)
    ceiling = np.array([[x, y, 1.1] for x in (4.0, 4.1, 4.2)
                        for y in (4.0, 4.1, 4.2)])
    xyz = np.vstack([floor, ceiling])
    res = build_reference(xyz, POSE, SPEC, GroundConfig())
    st = cell_stats(ceiling, SPEC)
    assert np.isnan(res.ref[st.counts > 0]).all()


def test_moderate_slope_connects():
    xs = np.arange(-2, 2, 0.05)
    xyz = _plane(xs, xs)
    xyz[:, 2] = 0.4 * xyz[:, 0]          # 0.2 m rise per 0.5 m cell, ground 0 at origin
    res = build_reference(xyz, POSE, SPEC, GroundConfig())
    dense = cell_stats(xyz, SPEC).counts >= GroundConfig().min_points_ref
    assert res.accepted[dense].all()


def test_sixty_centimetre_step_blocks_upper_level():
    # Stop the lower level at 1.95: arange's 2.0 drifts to 2.0000000000000036
    # and lands in the coarse cell (2.0, 2.5], contaminating the upper cell.
    lower = np.array([[x, y, 0.0] for x in np.arange(-2.0, 1.95, 0.05)
                      for y in np.arange(-2.0, 2.0, 0.05)])
    upper = np.array([[x, y, 0.6] for x in np.arange(2.5, 3.5, 0.05)
                      for y in np.arange(-3.0, 3.0, 0.05)])
    xyz = np.vstack([lower, upper])
    res = build_reference(xyz, POSE, SPEC, GroundConfig())
    st = cell_stats(upper, SPEC)
    assert np.isnan(res.ref[st.counts > 0]).all()


def test_one_point_five_metre_blind_gap_is_bridged():
    ring = np.array([[x, y, 0.0] for x in np.arange(3.0, 4.0, 0.05)
                     for y in np.arange(-3.0, 3.0, 0.05)])
    res = build_reference(ring, POSE, SPEC, GroundConfig())
    st = cell_stats(ring, SPEC)
    assert res.accepted[st.counts > 0].any()


def _result_with_floor(z=0.0):
    st = cell_stats(_flat_cloud(z=z), SPEC)
    ref = np.where(st.counts > 0, z, np.nan)
    return ReferenceResult(ref, st.counts > 0, np.zeros_like(st.counts, np.int16),
                           z, z, True, {})


def test_select_ground_keeps_points_in_band():
    xyz = np.array([[0.05, 0.05, 0.0], [0.05, 0.05, 0.44], [0.05, 0.05, 0.46]])
    mask = select_ground(xyz, _result_with_floor(), SPEC, GroundConfig())
    assert mask.tolist() == [True, True, False]


def test_low_rock_is_visible_at_045_but_not_at_020():
    xyz = np.array([[0.05, 0.05, 0.22]] * 3)
    base = _result_with_floor()
    assert select_ground(xyz, base, SPEC, GroundConfig()).all()
    tight = GroundConfig(out_band_above=0.20)
    assert not select_ground(xyz, base, SPEC, tight).any()


def test_35_degree_slope_has_no_nan_stripe():
    # Reference taken from each cell's own p20 (the output-band question is
    # isolated from the flood's connectivity capacity).
    xs = np.arange(-2, 2, 0.02)
    xyz = _plane(xs, xs)
    xyz[:, 2] = np.tan(np.radians(35.0)) * xyz[:, 0]
    st = cell_stats(xyz, SPEC)
    ref = np.where(st.counts > 0, st.p20, np.nan)
    res = ReferenceResult(ref, st.counts > 0, np.zeros_like(st.counts, np.int16),
                          0.0, 0.0, True, {})
    assert select_ground(xyz, res, SPEC, GroundConfig()).all()


def test_fine_cells_nest_in_exactly_one_coarse_cell():
    fine = _spec()
    coarse = coarse_spec(fine)
    # ref encodes the coarse row: adjacent rows differ by 0.05, so a
    # one-row mis-mapping would fall outside the +/-0.01 output band.
    ref = (np.arange(coarse.n_rows)[:, None] * 0.05
           + np.zeros((1, coarse.n_cols)))
    res = ReferenceResult(ref, np.ones_like(ref, dtype=bool),
                          np.zeros(ref.shape, dtype=np.int16), 0.0, 0.0,
                          True, {})
    xs = 5.0 - (np.arange(fine.n_rows) + 0.5) * 0.1
    ys = 5.0 - (np.arange(fine.n_cols) + 0.5) * 0.1
    xg, yg = np.meshgrid(xs, ys, indexing="ij")
    zg = (np.arange(fine.n_rows) // 5)[:, None] * 0.05 + np.zeros_like(xg)
    xyz = np.stack([xg, yg, zg], axis=-1).reshape(-1, 3)
    tight = GroundConfig(out_band_above=0.01, out_band_below=0.01)
    assert select_ground(xyz, res, coarse, tight).all()


def test_output_band_counts_high_and_low_points():
    xyz = np.array([[0.05, 0.05, 0.5], [0.05, 0.05, 0.1], [0.05, 0.05, -0.5]])
    above, below = output_band_counts(
        xyz, _result_with_floor(), SPEC, GroundConfig())
    assert (above, below) == (1, 1)


def test_despike_replaces_a_sparse_spike():
    elev = np.zeros((5, 5))
    elev[2, 2] = 0.5
    counts = np.full((5, 5), 10)
    counts[2, 2] = 1
    out = despike_sparse(elev, counts, min_support=3)
    assert out[2, 2] == pytest.approx(0.0)


def test_despike_keeps_a_supported_wall_cell():
    # A 1-cell-wide wall base has many points; median-filtering it away would
    # erase the wall from the terrain layer.
    elev = np.zeros((5, 5))
    elev[2, 2] = 0.25
    counts = np.full((5, 5), 10)
    counts[2, 2] = 20
    out = despike_sparse(elev, counts, min_support=3)
    assert out[2, 2] == pytest.approx(0.25)


def test_despike_keeps_unobserved_cells_nan():
    elev = np.zeros((5, 5))
    elev[2, 2] = np.nan
    counts = np.full((5, 5), 10)
    counts[2, 2] = 0
    out = despike_sparse(elev, counts, min_support=3)
    assert np.isnan(out[2, 2])


def test_despike_sparse_flat_cell_is_unchanged():
    elev = np.zeros((5, 5))
    counts = np.full((5, 5), 10)
    counts[2, 2] = 1
    out = despike_sparse(elev, counts, min_support=3)
    assert out[2, 2] == pytest.approx(0.0)
    assert np.isfinite(out).all()


def test_reference_fill_uses_the_cell_median_not_the_p20():
    # Cell (9, 13) is outside the 1.5 m seed disc and contains only a 0..0.4 m
    # spread (a rough patch); the neighbours are a flat floor.  The flood's
    # p20 gate reference is low, while the in-band median is ~0.15.
    xs = np.arange(-2.0, 2.0 + 1e-9, 0.05)
    floor_xy = np.array([[x, y] for x in xs for y in xs], dtype=float)
    row, col, _ = _cell_indices(floor_xy, SPEC)
    floor_xy = floor_xy[~((row == 9) & (col == 13))]   # the cell stays empty
    floor = np.column_stack([floor_xy, np.zeros(floor_xy.shape[0])])
    rough = np.array([[0.25, -1.75, z] for z in np.linspace(0.0, 0.4, 10)])
    res = build_reference(np.vstack([floor, rough]), POSE, SPEC, GroundConfig())
    assert res.ring_ok
    assert res.accepted[9, 13]
    assert res.fill_ref is not None
    assert res.fill_ref[9, 13] > res.ref[9, 13] + 0.05
    assert res.fill_ref[9, 13] == pytest.approx(0.15, abs=0.05)


def test_fill_sparse_from_reference_uses_the_coarse_ground():
    elev = np.full((10, 10), 9.0)
    elev[0, 0] = elev[0, 5] = elev[5, 0] = elev[5, 5] = 0.8   # spikes
    elev[2, 2] = 7.0                                          # supported
    elev[9, 9] = np.nan                                       # unobserved
    counts = np.full((10, 10), 10)
    for rc in ((0, 0), (0, 5), (5, 0), (5, 5)):
        counts[rc] = 1
    counts[9, 9] = 0
    ref = np.array([[0.0, 10.0], [20.0, 30.0]])
    out = fill_sparse_from_reference(
        elev, counts, ref, ratio=5, min_support=3)
    assert out[0, 0] == 0.0
    assert out[0, 5] == 10.0
    assert out[5, 0] == 20.0
    assert out[5, 5] == 30.0
    assert out[2, 2] == 7.0
    assert np.isnan(out[9, 9])


def test_fill_sparse_from_reference_rejects_shape_mismatch():
    with pytest.raises(ValueError):
        fill_sparse_from_reference(
            np.zeros((10, 10)), np.zeros((10, 10), dtype=int),
            np.zeros((2, 2)), ratio=4, min_support=3)


def test_coverage_guard_cases():
    c = GroundConfig()
    assert coverage_ok(10, None, c)
    assert coverage_ok(10, 100, c)            # previous below the absolute floor
    assert not coverage_ok(500, 1000, c)      # 50% < 60%
    assert coverage_ok(700, 1000, c)
    assert not coverage_ok(100, 1000, c)      # below the absolute floor


def test_three_metre_hole_stops_propagation():
    # The coarse cell containing x=4.0 spans (3.5, 4.0], so start beyond it:
    # otherwise the first return resets the gap budget one cell early.
    ring = np.array([[x, y, 0.0] for x in np.arange(4.5, 5.0, 0.05)
                     for y in np.arange(-3.0, 3.0, 0.05)])
    res = build_reference(ring, POSE, SPEC, GroundConfig())
    st = cell_stats(ring, SPEC)
    assert np.isnan(res.ref[st.counts > 0]).all()


def test_cell_stats_matches_the_naive_lexsort():
    """The native counting sort must reproduce the numpy grouping exactly."""
    from pgo_gridmap_projector.ground import cell_stats, coarse_spec
    from pgo_gridmap_projector.rasterizer import GridSpec

    rng = np.random.default_rng(11)
    n = 120_000
    xyz = np.stack([rng.uniform(-20, 20, n), rng.uniform(-20, 20, n),
                    rng.normal(0.0, 0.4, n)], axis=1)
    xyz[rng.random(n) < 0.02, 2] = np.nan          # some rejected points
    coarse = coarse_spec(GridSpec(resolution=0.1, length_x=40.0,
                                  length_y=40.0))

    got = cell_stats(xyz, coarse)

    # Reference implementation: exactly the pre-2026-11-14 numpy path.
    finite = np.isfinite(xyz).all(axis=1)
    xy, z = xyz[finite, :2], xyz[finite, 2]
    row = np.floor((coarse.origin_x + coarse.half_x - xy[:, 0])
                   / coarse.resolution).astype(np.int64)
    col = np.floor((coarse.origin_y + coarse.half_y - xy[:, 1])
                   / coarse.resolution).astype(np.int64)
    inside = ((row >= 0) & (row < coarse.n_rows)
              & (col >= 0) & (col < coarse.n_cols))
    flat = col[inside] * coarse.n_rows + row[inside]
    z = z[inside]
    order = np.lexsort((z, flat))
    sorted_flat = flat[order]
    sorted_z = z[order]
    starts = np.flatnonzero(np.r_[True, sorted_flat[1:] != sorted_flat[:-1]])
    counts = np.diff(np.r_[starts, sorted_flat.size])
    uniq = sorted_flat[starts]

    assert np.array_equal(got.sorted_z, sorted_z)
    counts_flat = got.counts.reshape(-1, order="F")
    assert counts_flat[uniq].tolist() == counts.tolist()
    idx20 = starts + np.floor(0.2 * (counts - 1)).astype(np.int64)
    p20_flat = got.p20.reshape(-1, order="F")
    assert np.allclose(p20_flat[uniq], sorted_z[idx20])


def test_native_flood_matches_the_python_flood():
    """The C++ BFS must reproduce the list/bisect reference cell for cell."""
    from pgo_gridmap_projector.ground import (
        _flood, _flood_python, _load_native_cell_sort, cell_stats,
        coarse_spec, seed_cells, seed_z_from_pose, ring_check, GroundConfig)

    if _load_native_cell_sort() is None:
        pytest.skip("libpgo_raster.so not built")

    rng = np.random.default_rng(5)
    n = 90_000
    xyz = np.stack([rng.uniform(-30, 30, n), rng.uniform(-30, 30, n),
                    0.08 * rng.uniform(-30, 30, n) + rng.normal(0, 0.04, n)],
                   axis=1)
    spec = coarse_spec(GridSpec(resolution=0.1, length_x=60.0, length_y=60.0))
    cfg = GroundConfig()
    stats = cell_stats(xyz, spec)
    pose = (0.0, 0.0, 0.4)
    ring = ring_check(stats, spec, pose[:2], seed_z_from_pose(pose, cfg), cfg)
    seed_z = seed_z_from_pose(pose, cfg) if ring.z is None else ring.z
    ref, accepted, _, _ = seed_cells(stats, spec, pose[:2], seed_z, cfg)
    fill = ref.copy()
    n_rows, n_cols = stats.counts.shape
    seeds = np.array([int(cc) * n_rows + int(rr)
                      for rr, cc in np.argwhere(accepted)], dtype=np.int64)

    py = _flood_python(stats, n_rows, n_cols, seeds, ref, fill, accepted, cfg)
    native = _flood(stats, n_rows, n_cols, seeds, ref, fill, accepted, cfg)
    for i in range(4):
        assert np.array_equal(np.nan_to_num(py[i]), np.nan_to_num(native[i])), \
            f"output {i} differs"
