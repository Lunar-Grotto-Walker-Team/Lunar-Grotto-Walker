import numpy as np

from utlidar_terrain_fill.map_odom import (
    MapOdomConfig,
    MapOdomFallback,
    inv_T,
    make_T,
    rpy_to_R,
)


def _cfg(**kw):
    base = dict(
        body_to_base_x=0.187793,
        body_to_base_y=0.02329,
        body_to_base_z=0.391836,
        body_to_base_roll=0.0,
        body_to_base_pitch=0.2269,
        body_to_base_yaw=0.0,
        utlidar_x_offset=0.28945,
        body_height_bias=0.057 - 0.046825,
        timeout_s=2.0,
    )
    base.update(kw)
    return MapOdomConfig(**base)


def _base_from_robot(T_odom_ulidar, cfg, body_height):
    T = np.array(T_odom_ulidar, copy=True)
    offset = T_odom_ulidar[:3, :3] @ np.array(
        [cfg.utlidar_x_offset, 0.0, 0.0])
    T[0, 3] -= offset[0]
    T[1, 3] -= offset[1]
    T[2, 3] -= body_height + cfg.body_height_bias
    return T


def test_formula_composes_to_map_body_over_base():
    cfg = _cfg()
    fb = MapOdomFallback(cfg)
    T_map_body = make_T(rpy_to_R(0.1, 0.2, 0.3), (1.0, 2.0, 3.0))
    T_robot = make_T(rpy_to_R(0.0, 0.1, 0.5), (0.4, 0.1, 0.6))
    body_height = 0.3
    fb.set_pgo(T_map_body, 10.0)
    fb.set_robot_pose(T_robot, 10.0)
    fb.set_body_height(body_height, 10.0)

    T = fb.map_odom(10.0)
    T_odom_base = _base_from_robot(T_robot, cfg, body_height)
    T_base_body = make_T(
        rpy_to_R(cfg.body_to_base_roll, cfg.body_to_base_pitch,
                 cfg.body_to_base_yaw),
        (cfg.body_to_base_x, cfg.body_to_base_y, cfg.body_to_base_z),
    )
    expected = T_map_body @ inv_T(T_base_body) @ inv_T(T_odom_base)
    np.testing.assert_allclose(T, expected, atol=1e-9)
    # The key composition: T_map_odom * T_odom_base == T_map_body * T_base_body^-1
    np.testing.assert_allclose(
        T @ T_odom_base, T_map_body @ inv_T(T_base_body), atol=1e-9)


def test_identity_pose_and_zero_extrinsics_give_identity():
    cfg = _cfg(
        body_to_base_x=0.0, body_to_base_y=0.0, body_to_base_z=0.0,
        body_to_base_roll=0.0, body_to_base_pitch=0.0, body_to_base_yaw=0.0,
        utlidar_x_offset=0.28945, body_height_bias=0.010175, timeout_s=2.0)
    fb = MapOdomFallback(cfg)
    fb.set_pgo(np.eye(4), 1.0)
    fb.set_body_height(0.3, 1.0)
    T_robot = make_T(np.eye(3), (0.28945, 0.0, 0.3 + 0.010175))
    fb.set_robot_pose(T_robot, 1.0)
    np.testing.assert_allclose(fb.map_odom(1.0), np.eye(4), atol=1e-9)


def test_yaw_rotates_the_ulidar_x_offset():
    cfg = _cfg()
    fb = MapOdomFallback(cfg)
    body_height = 0.3
    yaw = np.pi / 2.0
    R = rpy_to_R(0.0, 0.0, yaw)
    offset_base = np.array([cfg.utlidar_x_offset, 0.0, 0.0])
    p_ulidar = R @ offset_base + np.array(
        [0.0, 0.0, body_height + cfg.body_height_bias])
    T_robot = make_T(R, p_ulidar)
    fb.set_pgo(np.eye(4), 1.0)
    fb.set_robot_pose(T_robot, 1.0)
    fb.set_body_height(body_height, 1.0)
    T_map_odom = fb.map_odom(1.0)
    T_base_body = make_T(
        rpy_to_R(cfg.body_to_base_roll, cfg.body_to_base_pitch,
                 cfg.body_to_base_yaw),
        (cfg.body_to_base_x, cfg.body_to_base_y, cfg.body_to_base_z))
    T_map_base = np.eye(4) @ inv_T(T_base_body)
    T_base_ulidar = inv_T(T_map_base) @ T_map_odom @ T_robot
    # In base_footprint the uLidar is at (+0.28945, 0, body_height+bias),
    # independent of yaw.  The old unrotated subtraction would give
    # (0, +0.28945, ...) here.
    np.testing.assert_allclose(
        T_base_ulidar[:3, 3],
        [cfg.utlidar_x_offset, 0.0, body_height + cfg.body_height_bias],
        atol=1e-9)


def test_missing_or_stale_inputs_return_none():
    cfg = _cfg()
    fb = MapOdomFallback(cfg)
    assert fb.map_odom(0.0) is None
    fb.set_pgo(np.eye(4), 0.0)
    fb.set_robot_pose(np.eye(4), 0.0)
    assert fb.map_odom(0.0) is None  # body height missing
    fb.set_body_height(0.3, 0.0)
    assert fb.map_odom(1.0) is not None
    assert fb.map_odom(3.0) is None  # all older than 2 s
