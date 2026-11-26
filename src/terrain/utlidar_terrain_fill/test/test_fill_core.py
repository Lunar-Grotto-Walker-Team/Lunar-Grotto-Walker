import numpy as np

from utlidar_terrain_fill.fill_core import (
    GroundConfig, RollingBuffer, classify_ground, dense_ground_surface,
    estimate_ground_grid, fill_ground, voxel_downsample)


def _floor(n=8):
    xs = np.linspace(-0.9, 0.9, n)
    ys = np.linspace(-0.9, 0.9, n)
    return np.array([[x, y, 0.0] for x in xs for y in ys], dtype=float)


def test_flat_floor_cells_are_trusted():
    xyz = _floor()
    g = estimate_ground_grid(
        xyz, np.array([-2.0, -2.0]), 0.5, 8, 20.0, 3, 0.20)
    assert g.trusted.any()
    assert np.nanmax(np.abs(g.z[g.trusted])) < 1e-9


def test_wall_cell_is_untrusted():
    # A 0.25 m cell filled by vertical wall points from z=0 to z=1.
    xyz = np.array([[0.1, 0.1, z] for z in np.linspace(0.0, 1.0, 20)],
                   dtype=float)
    g = estimate_ground_grid(
        xyz, np.array([0.0, 0.0]), 0.25, 4, 20.0, 3, 0.20)
    assert not g.trusted[0, 0]
    assert np.isnan(g.z[0, 0])


def test_too_few_points_is_untrusted():
    xyz = np.array([[0.1, 0.1, 0.0], [0.11, 0.11, 0.0]], dtype=float)
    g = estimate_ground_grid(
        xyz, np.array([0.0, 0.0]), 0.25, 4, 20.0, 3, 0.20)
    assert not g.trusted[0, 0]


def test_30deg_slope_is_trusted_and_kept():
    x = np.linspace(-1.0, 1.0, 40)
    y = np.linspace(-0.5, 0.5, 20)
    tan30 = np.tan(np.deg2rad(30.0))
    xyz = np.array([[xx, yy, xx * tan30] for xx in x for yy in y], dtype=float)
    g = estimate_ground_grid(
        xyz, np.array([-2.0, -2.0]), 0.25, 16, 20.0, 3, 0.20)
    g = fill_ground(g, 4)
    kept = classify_ground(xyz, g, -0.10, 0.15, 3.0, (0.0, 0.0))
    assert len(kept) > 0.5 * len(xyz)


def test_wall_cell_points_are_dropped_after_neighbour_fill():
    floor = np.array([[x, y, 0.0] for x in np.linspace(0.0, 0.4, 10)
                      for y in np.linspace(0.0, 0.4, 10)])
    wall = np.array([[0.55, 0.2, z] for z in np.linspace(0.0, 1.0, 20)])
    xyz = np.vstack([floor, wall])
    g = estimate_ground_grid(
        xyz, np.array([0.0, 0.0]), 0.25, 4, 20.0, 3, 0.20)
    g = fill_ground(g, 4)
    kept = classify_ground(xyz, g, -0.10, 0.15, 3.0, (0.0, 0.0))
    assert not any(p[2] > 0.20 for p in kept)


def test_dense_surface_covers_more_than_the_raw_returns():
    # The real L2 gives ~1 return per 0.1 m cell.  Publishing the returns
    # leaves many 0.1 m output cells empty (they become UNKNOWN); the dense
    # surface puts one point in every 0.1 m cell of the ground estimate.
    xyz = np.array([[x, y, 0.0] for x in np.arange(-0.5, 0.51, 0.1)
                    for y in np.arange(-0.5, 0.51, 0.1)])
    g = estimate_ground_grid(
        xyz, np.array([-2.0, -2.0]), 0.25, 16, 20.0, 3, 0.20)
    g = fill_ground(g, 4)
    raw = voxel_downsample(xyz, 0.10)
    dense = dense_ground_surface(g, (0.0, 0.0), 3.0, 0.10)

    def cover(pts):
        return len(set(zip(np.floor(pts[:, 0] / 0.1).astype(int),
                           np.floor(pts[:, 1] / 0.1).astype(int))))

    # The dense surface must fill every 0.1 m cell inside the floor, not
    # just the ones a return happened to land in.
    assert cover(dense) > cover(raw)
    assert cover(dense) >= int(round(1.0 / 0.1) ** 2 * 0.9)
    assert abs(np.median(dense[:, 2])) < 1e-9


def test_dense_surface_does_not_floor_over_a_wall_cell():
    floor = np.array([[x, y, 0.0] for x in np.arange(0.0, 0.51, 0.1)
                      for y in np.arange(0.0, 0.51, 0.1)])
    wall = np.array([[0.55, 0.2, z] for z in np.linspace(0.0, 1.0, 20)])
    xyz = np.vstack([floor, wall])
    g = estimate_ground_grid(
        xyz, np.array([-2.0, -2.0]), 0.25, 16, 20.0, 3, 0.20)
    assert g.rejected[10, 8]  # x in [0.5, 0.75), y in [0.0, 0.25)
    g = fill_ground(g, 4)
    dense = dense_ground_surface(g, (0.0, 0.0), 3.0, 0.1)
    in_wall_cell = ((dense[:, 0] >= 0.5) & (dense[:, 0] < 0.75)
                    & (dense[:, 1] >= 0.0) & (dense[:, 1] < 0.25))
    assert not np.any(in_wall_cell)


def test_voxel_downsample_keeps_one_per_cell():
    xyz = np.array([[0.01, 0.01, 0.0], [0.02, 0.02, 0.0], [1.0, 1.0, 0.0]])
    out = voxel_downsample(xyz, 0.10)
    assert len(out) == 2


def test_rolling_buffer_prunes_and_clears():
    b = RollingBuffer(5.0)
    b.add(10.0, np.array([[0.0, 0.0, 0.0]]))
    b.add(16.0, np.array([[1.0, 0.0, 0.0]]))
    b.prune(16.0)
    assert len(b) == 1
    b.clear()
    assert len(b) == 0


def _scan():
    return np.array([[x, y, 0.0] for x in np.linspace(-0.5, 0.5, 20)
                     for y in np.linspace(-0.5, 0.5, 20)])


def test_pipeline_applies_z_offset_and_identity_tf():
    from utlidar_terrain_fill.fill_core import FillPipeline
    p = FillPipeline(GroundConfig(z_offset=0.05))
    p.update_tf(np.eye(4), 10.0)
    p.add_scan(_scan(), (0.0, 0.0), 10.0, np.eye(4))
    out = p.build_fill(10.0)
    assert len(out) > 0
    assert abs(np.median(out[:, 2]) - 0.05) < 1e-6


def test_scan_is_transformed_to_map_with_its_own_tf():
    from utlidar_terrain_fill.fill_core import FillPipeline
    # Same scan under identity and under a +1 m map shift: the whole dense
    # patch (which extends one fill iteration past the observed square) must
    # move by exactly the shift.
    base = FillPipeline(GroundConfig())
    base.update_tf(np.eye(4), 10.0)
    base.add_scan(_scan(), (0.0, 0.0), 10.0, np.eye(4))
    before = base.build_fill(10.0)
    assert len(before) > 0

    p = FillPipeline(GroundConfig())
    T = np.eye(4)
    T[0, 3] = 1.0
    p.update_tf(T, 10.0)
    p.add_scan(_scan(), (0.0, 0.0), 10.0, T)
    out = p.build_fill(10.0)
    assert len(out) > 0
    assert abs(np.median(out[:, 0]) - (np.median(before[:, 0]) + 1.0)) < 1e-6
    assert abs(np.median(out[:, 1]) - np.median(before[:, 1])) < 1e-6
    # And the patch stays bounded by the fill extent, not the 12 m grid.
    assert out[:, 0].min() > -0.7
    assert out[:, 0].max() < 3.0


def test_tf_jump_relocates_the_buffer_instead_of_clearing_it():
    """A map<-odom correction must not throw the fill away.

    The buffer holds odom points, so the jump moves the published patch.  The
    old design cleared it and, on this stack, lost ~1 s of every 5 s window.
    """
    from utlidar_terrain_fill.fill_core import FillPipeline
    p = FillPipeline(GroundConfig())
    p.update_tf(np.eye(4), 10.0)
    p.add_scan(_scan(), (0.0, 0.0), 10.0, np.eye(4))
    before = p.build_fill(10.0)
    assert len(before) > 0
    jump = np.eye(4)
    jump[0, 3] = 1.0
    assert p.update_tf(jump, 11.0) is False
    after = p.build_fill(11.0)
    assert len(after) == len(before)
    assert abs(np.median(after[:, 0]) - (np.median(before[:, 0]) + 1.0)) < 1e-6


def test_window_expiry_drops_old_scan():
    from utlidar_terrain_fill.fill_core import FillPipeline
    p = FillPipeline(GroundConfig(window_s=5.0))
    p.update_tf(np.eye(4), 10.0)
    p.add_scan(_scan(), (0.0, 0.0), 10.0, np.eye(4))
    assert len(p.build_fill(16.0)) == 0
