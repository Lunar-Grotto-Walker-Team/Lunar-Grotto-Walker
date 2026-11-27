"""Unit tests for the ground/obstacle split. Pure numpy, no ROS graph."""

import numpy as np

from pointlio_cloud_relay.ground_filter import (
    drop_origin_points, estimate_ground_grid, fill_missing,
    split_ground_obstacles,
)


GRID = 12.0
CELL = 0.2
# Defaults matching config/pointlio_frames.yaml.
KW = dict(grid_size=GRID, cell_size=CELL, percentile=20.0,
          min_points=3, min_height=0.15, fill_iterations=10,
          max_height=1.2)


def _floor(step=0.1, half=3.0, z=0.0):
    xs = np.arange(-half, half, step)
    ys = np.arange(-half, half, step)
    x, y = np.meshgrid(xs, ys)
    return np.stack((x.ravel(), y.ravel(), np.full(x.size, z)), axis=-1)


def test_exact_origin_padding_is_dropped():
    # The Unitree L2 pads its fixed-size array with exact (0, 0, 0) points
    # (10000 of 11239 per frame). They must not reach the splitter.
    floor = _floor()
    pad = np.zeros((10000, 3))
    kept = drop_origin_points(np.vstack((pad, floor)))
    # _floor() itself contains one point at (0, 0, 0), which is dropped too.
    assert kept.shape[0] == floor.shape[0] - 1


def test_origin_filter_keeps_real_near_field_points():
    # A point 1 cm from the acquisition origin is real (the L2 sees the
    # ground under the dog); only the exact zeros are padding.
    pts = np.array([[0.01, 0.0, 0.0], [0.0, -0.02, 0.0], [0.0, 0.0, 0.0]])
    kept = drop_origin_points(pts)
    assert kept.shape[0] == 2


def test_flat_floor_is_all_ground():
    xyz = _floor()
    ground, obstacles = split_ground_obstacles(xyz, (0.0, 0.0), **KW)
    assert obstacles.shape[0] == 0
    assert ground.shape[0] == xyz.shape[0]


def test_gentle_slope_is_all_ground():
    # 10 degree ramp along +y; the floor rises 1.76 m over 10 m.
    xyz = _floor(half=3.0)
    xyz[:, 2] = np.tan(np.radians(10.0)) * xyz[:, 1]
    ground, obstacles = split_ground_obstacles(xyz, (0.0, 0.0), **KW)
    assert obstacles.shape[0] == 0, (
        f'{obstacles.shape[0]} slope points were marked as obstacles')
    assert ground.shape[0] == xyz.shape[0]


def test_vertical_wall_above_ground_is_obstacle():
    floor = _floor()
    wall_y, wall_z = np.meshgrid(np.linspace(-1.0, 1.0, 41),
                                 np.arange(0.0, 1.5, 0.05))
    wall = np.stack((np.full(wall_z.size, 1.0),
                     wall_y.ravel(), wall_z.ravel()), axis=-1)
    xyz = np.vstack((floor, wall))

    ground, obstacles = split_ground_obstacles(xyz, (0.0, 0.0), **KW)
    assert obstacles.shape[0] > 0
    assert obstacles[:, 2].max() > 1.0
    # Nothing on the flat floor may be promoted to an obstacle.
    floor_only = obstacles[np.abs(obstacles[:, 2]) < 1e-6]
    assert floor_only.shape[0] == 0


def test_box_on_slope_is_obstacle():
    xyz = _floor(half=3.0)
    xyz[:, 2] = np.tan(np.radians(8.0)) * xyz[:, 1]
    # A box top 0.5 m above the ramp surface.
    box = _floor(half=0.5)
    box[:, 0] += 1.0
    box[:, 1] += 1.0
    box[:, 2] = np.tan(np.radians(8.0)) * box[:, 1] + 0.5
    xyz = np.vstack((xyz, box))

    ground, obstacles = split_ground_obstacles(xyz, (0.0, 0.0), **KW)
    assert obstacles.shape[0] == box.shape[0]
    assert ground.shape[0] == xyz.shape[0] - box.shape[0]


def test_blind_zone_is_filled_from_neighbours():
    # No returns within 1.5 m of the robot (Mid360S ground blind zone), the
    # rest of a 10 degree ramp is observed.
    xyz = _floor(half=3.5)
    xyz = xyz[np.hypot(xyz[:, 0], xyz[:, 1]) > 1.5]
    xyz[:, 2] = np.tan(np.radians(10.0)) * xyz[:, 1]

    ground, obstacles = split_ground_obstacles(xyz, (0.0, 0.0), **KW)
    assert obstacles.shape[0] == 0
    assert ground.shape[0] == xyz.shape[0]


def test_isolated_spike_in_a_dense_cell_stays_an_obstacle():
    floor = _floor(step=0.05, half=0.15)   # 36 points in one 0.2 m cell
    spike = np.array([[0.0, 0.0, 0.5]])
    xyz = np.vstack((floor, spike))
    ground, obstacles = split_ground_obstacles(xyz, (0.0, 0.0), **KW)
    assert obstacles.shape[0] == 1
    assert np.allclose(obstacles[0], spike[0])


def test_points_outside_the_grid_are_dropped():
    xyz = np.array([[100.0, 100.0, 0.0], [101.0, 100.0, 1.0]])
    ground, obstacles = split_ground_obstacles(xyz, (0.0, 0.0), **KW)
    assert ground.shape[0] == 0
    assert obstacles.shape[0] == 0


def test_empty_cloud():
    xyz = np.zeros((0, 3))
    ground, obstacles = split_ground_obstacles(xyz, (0.0, 0.0), **KW)
    assert ground.shape[0] == 0
    assert obstacles.shape[0] == 0


def test_ground_grid_percentile_is_robust_to_a_high_outlier():
    n = 60
    origin = np.array([-6.0, -6.0])
    # Ten ground points plus one 0.5 m spike in the same cell. Offset by
    # half a cell so the sample is not sitting on a cell boundary.
    pts = np.array([[0.05, 0.05, 0.0]] * 10 + [[0.05, 0.05, 0.5]])
    grid = estimate_ground_grid(
        pts, origin, CELL, n, percentile=20.0, min_points=3)
    row = int(np.floor((0.05 - origin[0]) / CELL))
    col = int(np.floor((0.05 - origin[1]) / CELL))
    assert grid[row, col] == 0.0


def test_fill_missing_spreads_the_nearest_valid_height():
    grid = np.full((5, 5), np.nan)
    grid[2, 2] = 1.0
    filled = fill_missing(grid, 2)
    # Two dilation steps reach the 5x5 corner cells.
    assert np.all(np.isfinite(filled))
    assert np.allclose(filled, 1.0)


def test_overhead_point_above_max_height_is_dropped():
    # A 2 m door lintel must not become an obstacle in a 2D costmap.
    floor = _floor()
    lintel = np.array([[0.05, 0.05, 2.0]])
    xyz = np.vstack((floor, lintel))
    ground, obstacles = split_ground_obstacles(xyz, (0.0, 0.0), **KW)
    assert obstacles.shape[0] == 0
    assert ground.shape[0] == floor.shape[0]


def test_low_obstacle_below_max_height_is_kept():
    floor = _floor()
    box = np.array([[0.05, 0.05, 0.5]])
    xyz = np.vstack((floor, box))
    ground, obstacles = split_ground_obstacles(xyz, (0.0, 0.0), **KW)
    assert obstacles.shape[0] == 1
    assert np.allclose(obstacles[0], box[0])
