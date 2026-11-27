import math
import numpy as np

from pointlio_cloud_relay.transform import (
    body_to_base_matrix, compose, rpy_to_matrix, transform_points,
)


def test_rpy_yaw_90_maps_x_to_y():
    T = rpy_to_matrix(0.0, 0.0, math.pi / 2)
    p = transform_points(np.array([[1.0, 0.0, 0.0]]), T)
    assert np.allclose(p, [[0.0, 1.0, 0.0]], atol=1e-9)


def test_body_to_base_matches_installation_geometry():
    # Physical chain: base_footprint -> base_link (0,0,0.357)
    #   -> livox_frame (0.187, 0, 0.0803, pitch +13 deg)
    #   -> body/IMU (lidar->IMU = -(extrinsic_T), R=I).
    # The product is T^base_body; body_to_base_matrix must return it directly.
    def T(x, y, z, roll, pitch, yaw):
        m = rpy_to_matrix(roll, pitch, yaw)
        m[:3, 3] = (x, y, z)
        return m

    T_base_body = (T(0, 0, 0.357, 0, 0, 0)
                   @ T(0.187, 0, 0.0803, 0, math.radians(13.0), 0)
                   @ T(0.011, 0.02329, -0.04412, 0, 0, 0))
    got = body_to_base_matrix(0.187793, 0.02329, 0.391836,
                              roll=0.0, pitch=math.radians(13.0), yaw=0.0)
    assert np.allclose(got, T_base_body, atol=1e-3), (
        "body->base_footprint extrinsic does not match the installation chain")


def test_compose_applies_right_to_left():
    A = rpy_to_matrix(0.0, 0.0, math.pi / 2)      # rotate +x -> +y
    B = np.eye(4)
    B[0, 3] = 1.0                                  # translate +x
    p = transform_points(np.array([[0.0, 0.0, 0.0]]), compose(A, B))
    assert np.allclose(p, [[0.0, 1.0, 0.0]], atol=1e-9)


def test_transform_to_matrix_matches_rpy_helper():
    from pointlio_cloud_relay.transform import transform_to_matrix
    T = transform_to_matrix((1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0))
    assert np.allclose(T, np.array([[1, 0, 0, 1], [0, 1, 0, 2],
                                    [0, 0, 1, 3], [0, 0, 0, 1]]))


def test_cloud_to_base_points_applies_inverse_and_extrinsic():
    from pointlio_cloud_relay.transform import (
        cloud_to_base_points, transform_to_matrix)
    # body origin sits at x=1 in the cloud frame; base is 0.5 m above body.
    T_cloud_body = transform_to_matrix((1.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    T_base_body = body_to_base_matrix(0.0, 0.0, 0.5, 0.0, 0.0, 0.0)
    got = cloud_to_base_points(
        np.array([[2.0, 0.0, 0.0]]), T_cloud_body, T_base_body)
    assert np.allclose(got, [[1.0, 0.0, 0.5]], atol=1e-9)


def test_cloud_to_base_points_applies_rotation():
    from pointlio_cloud_relay.transform import (
        cloud_to_base_points, transform_to_matrix)
    # body is rotated +90 deg about z w.r.t. the cloud frame.
    s = math.sin(math.pi / 4)
    c = math.cos(math.pi / 4)
    T_cloud_body = transform_to_matrix((0.0, 0.0, 0.0), (0.0, 0.0, s, c))
    T_base_body = body_to_base_matrix(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    got = cloud_to_base_points(
        np.array([[1.0, 0.0, 0.0]]), T_cloud_body, T_base_body)
    assert np.allclose(got, [[0.0, -1.0, 0.0]], atol=1e-9)
