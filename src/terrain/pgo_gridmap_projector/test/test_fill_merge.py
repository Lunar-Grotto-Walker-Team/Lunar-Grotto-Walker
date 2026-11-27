import numpy as np

from pgo_gridmap_projector.fill_merge import merge_fill_points


def test_fill_in_livox_occupied_cell_is_dropped():
    livox = np.array([[0.0, 0.0, 0.0]])
    fill = np.array([[0.01, 0.01, 0.0]])
    merged, kept, dropped = merge_fill_points(
        livox, fill, 10.0, 10.0, 2.0, 0.10, -0.5, 0.5)
    assert kept == 0 and dropped == 1
    np.testing.assert_allclose(merged, livox)


def test_fill_in_empty_cell_is_kept():
    livox = np.array([[0.0, 0.0, 0.0]])
    fill = np.array([[1.0, 1.0, 0.0]])
    merged, kept, dropped = merge_fill_points(
        livox, fill, 10.0, 10.0, 2.0, 0.10, -0.5, 0.5)
    assert kept == 1 and dropped == 0
    assert len(merged) == 2


def test_stale_fill_is_ignored():
    livox = np.array([[0.0, 0.0, 0.0]])
    fill = np.array([[1.0, 1.0, 0.0]])
    merged, kept, dropped = merge_fill_points(
        livox, fill, 0.0, 10.0, 2.0, 0.10, -0.5, 0.5)
    assert kept == 0 and dropped == 0
    np.testing.assert_allclose(merged, livox)


def test_fill_outside_z_gate_is_dropped():
    livox = np.array([[0.0, 0.0, 0.0]])
    fill = np.array([[1.0, 1.0, 9.0]])
    merged, kept, dropped = merge_fill_points(
        livox, fill, 10.0, 10.0, 2.0, 0.10, -0.5, 0.5)
    assert kept == 0 and dropped == 1
    np.testing.assert_allclose(merged, livox)
