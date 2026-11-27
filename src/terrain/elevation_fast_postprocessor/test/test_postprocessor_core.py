import numpy as np
from elevation_fast_postprocessor.postprocessor_core import (
    box_mean, compute_layers, fill_invalid, surface_normals)


def test_fill_invalid_uses_nearest_valid_value():
    e = np.full((5, 5), np.nan)
    e[:, 0] = 1.0
    out = fill_invalid(e)
    assert np.isfinite(out).all()
    assert np.allclose(out[:, 4], 1.0)


def test_fill_invalid_all_nan_stays_nan():
    out = fill_invalid(np.full((3, 3), np.nan))
    assert np.isnan(out).all()


def test_box_mean_preserves_a_linear_ramp_interior():
    res = 0.1
    elev = np.tile(np.arange(11) * res, (11, 1)).astype(np.float64)  # 沿 col 线性
    smooth = box_mean(elev, radius_m=0.15, resolution=res)
    assert np.allclose(smooth[3:8, 3:8], elev[3:8, 3:8], atol=1e-9)


def test_normals_flat_map_points_up():
    nx, ny, nz = surface_normals(np.zeros((5, 5)), resolution=0.1)
    assert np.allclose(nz[2:-2, 2:-2], 1.0)
    assert np.allclose(nx[2:-2, 2:-2], 0.0, atol=1e-9)


def test_normals_ramp_along_rows_gives_45_degrees():
    res = 0.1
    elev = np.tile((np.arange(11) * res)[:, None], (1, 11))  # row ↔ x
    nx_, ny_, nz_ = surface_normals(elev, resolution=res)
    slope = np.arccos(np.clip(nz_, -1.0, 1.0))
    assert np.allclose(slope[4:7, 4:7], np.pi / 4, atol=1e-6)
    assert np.allclose(np.abs(nx_[4:7, 4:7]), np.sin(np.pi / 4), atol=1e-6)


def test_roughness_is_zero_on_a_plane_and_positive_at_a_step():
    elev = np.zeros((11, 11))
    elev[:, 6:] = 0.3
    layers = compute_layers(elev, resolution=0.1, mean_radius_m=0.15)
    assert np.allclose(layers["roughness"][2:8, 2:4], 0.0, atol=1e-9)
    assert layers["roughness"][5, 5] > 0.0


def test_compute_layers_nan_cells_stay_nan():
    elev = np.full((7, 7), np.nan)
    elev[2:5, 2:5] = 0.0
    layers = compute_layers(elev, resolution=0.1)
    assert np.isnan(layers["slope"][0, 0])
    assert np.isfinite(layers["slope"][3, 3])


def test_debug_layers_are_opt_in():
    elev = np.zeros((7, 7))
    assert "normal_vectors_x" not in compute_layers(elev, 0.1)
    assert "normal_vectors_x" in compute_layers(elev, 0.1, publish_debug=True)


def test_box_mean_uses_a_3x3_kernel_at_radius_0_15():
    """radius 0.15 m at 0.10 m/cell is the old chain's 3x3 neighbourhood.

    floor(0.15/0.10) = 1 -> kernel 3.  A 5x5 kernel would average 25 cells
    (max radius 0.28 m) and move the roughness the bridge thresholds use.
    """
    e = np.zeros((11, 11))
    e[5, 5] = 1.0
    smooth = box_mean(e, radius_m=0.15, resolution=0.1)
    assert smooth[5, 6] > 0.0        # one cell away: inside
    assert smooth[5, 7] == 0.0       # two cells away: outside a 3x3


def test_normals_point_toward_the_direction_of_descent():
    """row increases as x DECREASES (rasterizer convention).

    elev rises with row, so the ground slopes up toward -x and the outward
    normal must have a POSITIVE x component.
    """
    res = 0.1
    elev = np.tile((np.arange(11) * res)[:, None], (1, 11))
    nx_, ny_, _ = surface_normals(elev, resolution=res)
    assert nx_[5, 5] > 0.7
    assert abs(ny_[5, 5]) < 1e-9
