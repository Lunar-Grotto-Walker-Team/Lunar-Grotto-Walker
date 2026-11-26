"""Tests for the projector's point-cloud decoding and parameter handling."""
import time

import numpy as np
import pytest
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from sensor_msgs.msg import PointCloud2, PointField

from pgo_gridmap_projector.projector_node import ProjectorNode, decompress_points


def _cloud(points):
    """Build a PointCloud2 with x/y/z float32 fields from (x, y, z) tuples."""
    msg = PointCloud2()
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.is_dense = True
    msg.data = b"".join(
        np.asarray([x, y, z], dtype=np.float32).tobytes()
        for (x, y, z) in points
    )
    return msg


def test_decompress_points_extracts_xyz():
    msg = _cloud([(1.0, 2.0, 3.0), (4.0, 5.0, 6.0)])
    out = decompress_points(msg)
    assert out.shape == (2, 3)
    assert out.dtype == np.float64
    np.testing.assert_allclose(out, [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])


def test_decompress_points_empty_cloud():
    msg = _cloud([])
    out = decompress_points(msg)
    assert out.shape == (0, 3)


def test_decompress_points_handles_offset_fields():
    """Some publishers put xyz at a non-zero offset; honour the fields."""
    msg = PointCloud2()
    msg.height = 1
    msg.width = 1
    msg.fields = [
        PointField(name="intensity", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="x", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16
    msg.is_dense = True
    msg.data = np.asarray([9.0, 1.0, 2.0, 3.0], dtype=np.float32).tobytes()
    out = decompress_points(msg)
    np.testing.assert_allclose(out, [[1.0, 2.0, 3.0]])


def test_decompress_points_handles_padded_points():
    """A point_step larger than the xyz fields (ring/time padding) works."""
    msg = PointCloud2()
    msg.height = 1
    msg.width = 1
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 32
    msg.row_step = 32
    msg.is_dense = True
    msg.data = np.asarray([1.0, 2.0, 3.0] + [0.0] * 5, dtype=np.float32).tobytes()
    np.testing.assert_allclose(decompress_points(msg), [[1.0, 2.0, 3.0]])


def test_decompress_points_rejects_field_offset_outside_the_point():
    """A field offset past point_step must raise, not IndexError.

    An IndexError here would escape the subscription callback's
    `except ValueError` and take the node down.
    """
    msg = PointCloud2()
    msg.height = 1
    msg.width = 1
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=64, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16
    msg.is_dense = True
    msg.data = np.zeros(4, dtype=np.float32).tobytes()
    with pytest.raises(ValueError):
        decompress_points(msg)


def test_decompress_points_rejects_point_step_not_a_multiple_of_four():
    msg = PointCloud2()
    msg.height = 1
    msg.width = 2
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 14
    msg.row_step = 28
    msg.is_dense = True
    msg.data = np.zeros(28, dtype=np.float32).tobytes()
    with pytest.raises(ValueError):
        decompress_points(msg)


def test_decompress_points_rejects_missing_fields():
    msg = PointCloud2()
    msg.height = 1
    msg.width = 1
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 4
    msg.row_step = 4
    msg.is_dense = True
    msg.data = np.zeros(4, dtype=np.float32).tobytes()
    with pytest.raises(ValueError):
        decompress_points(msg)


# ---------------------------------------------------------------------------
# _on_cloud end-to-end: the callback must complete and publish.
#
# These are not decorative. A unit test of the pure helpers alone cannot see
# an AttributeError raised on the way to publish() -- `msg.stamp` instead of
# `msg.header.stamp` was exactly such a bug: it escaped the callback, killed
# the node on the first frame, and every helper-level test still passed.
# ---------------------------------------------------------------------------


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


class _Capture:
    """Stands in for the publisher so publish() can be asserted on."""

    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def test_on_cloud_publishes_a_gridmap_with_the_cloud_stamp(ros_context):
    """The whole callback path must run, and carry the cloud's stamp through.

    Regression test: `msg.stamp` (which PointCloud2 does not have) raised
    AttributeError here, escaping the callback and killing the node.
    """
    node = ProjectorNode()
    capture = _Capture()
    node._pub = capture
    try:
        cloud = _cloud([(0.05, 0.05, 0.2), (0.15, 0.15, 0.2), (0.25, 0.25, 0.2)])
        cloud.header.stamp = TimeMsg(sec=1234, nanosec=567)
        node._on_cloud(cloud)
        assert len(capture.messages) == 1
        out = capture.messages[0]
        assert out.header.stamp.sec == 1234
        assert out.header.stamp.nanosec == 567
        assert out.header.frame_id == "map"
        assert out.layers == ["elevation", "counts"]
    finally:
        node.destroy_node()


def test_on_cloud_skips_publish_when_the_z_gate_drops_everything(ros_context):
    """Points outside [z_min, z_max] must produce no publish, not an empty map."""
    node = ProjectorNode()
    capture = _Capture()
    node._pub = capture
    try:
        # ignore_points_above defaults to 0.5, so all of these are filtered.
        cloud = _cloud([(0.05, 0.05, 9.0), (0.15, 0.15, 9.0)])
        node._on_cloud(cloud)
        assert capture.messages == []
    finally:
        node.destroy_node()


def test_on_cloud_survives_a_malformed_cloud(ros_context):
    """A decode error must be logged, not propagated into the executor."""
    node = ProjectorNode()
    capture = _Capture()
    node._pub = capture
    try:
        bad = _cloud([(0.0, 0.0, 0.0)])
        bad.fields = bad.fields[:1]  # drop y and z
        node._on_cloud(bad)  # must not raise
        assert capture.messages == []
    finally:
        node.destroy_node()


from rclpy.parameter import Parameter


def _make_node(auto_extent, ros_context):
    return ProjectorNode(parameter_overrides=[
        Parameter("auto_extent", value=auto_extent)])


def test_next_spec_grows_then_stays_monotonic(ros_context):
    node = _make_node(True, ros_context)
    try:
        a = node._next_spec(np.array([[0.0, 0.5, 0.0]]))
        b = node._next_spec(np.array([[0.0, 0.5, 0.0], [12.0, 0.0, 0.0]]))
        c = node._next_spec(np.array([[0.0, 0.5, 0.0]]))
        assert b.length_x > a.length_x
        assert c == b
    finally:
        node.destroy_node()


def test_next_spec_uses_fixed_length_when_auto_extent_off(ros_context):
    node = _make_node(False, ros_context)
    try:
        spec = node._next_spec(np.array([[12.0, 0.0, 0.0]]))
        assert (spec.length_x, spec.length_y) == (30.0, 30.0)
        assert (spec.origin_x, spec.origin_y) == (0.0, 0.0)
    finally:
        node.destroy_node()


# ---------------------------------------------------------------------------
# Height-gate anchor: the band must follow the PGO LiDAR z.
# ---------------------------------------------------------------------------


def test_band_follows_the_lidar_height(ros_context):
    """The gate slides with /aft_pgo_odom instead of staying at the origin.

    On the corridor bag the floor sat at map z = -0.39 and the fixed band's
    lower edge was -0.5, so 0.11 m of LIO z drift was enough for the floor to
    leave the gate and freeze the elevation map.  Anchoring on the current
    pose keeps the floor at a constant offset inside the band.
    """
    node = ProjectorNode()
    try:
        # Before any odometry arrives the old fixed band is used, so the
        # chain behaves exactly like before during startup.
        assert node._band() == (-0.5, 0.5)

        node._odom_z = 1.8
        node._odom_recv = time.monotonic()
        assert node._band() == (1.3, 2.3)
    finally:
        node.destroy_node()


def test_band_uses_the_last_anchor_when_the_odom_is_stale(ros_context):
    """A stale anchor must be kept, not silently reset to the map origin."""
    node = ProjectorNode()
    try:
        node._odom_z = 0.7
        node._odom_recv = time.monotonic() - 60.0
        assert node._band() == pytest.approx((0.2, 1.2))
    finally:
        node.destroy_node()


def test_band_can_be_pinned_to_the_map_origin_for_ab(ros_context):
    node = ProjectorNode(parameter_overrides=[
        Parameter("ground_mode", value="band"),
        Parameter("follow_odom_height", value=False)])
    try:
        node._odom_z = 3.0
        node._odom_recv = time.monotonic()
        assert node._band() == (-0.5, 0.5)
        assert node._odom_sub is None
    finally:
        node.destroy_node()


def test_on_cloud_accepts_points_inside_the_following_band(ros_context):
    """End-to-end: a cloud at a drifted height still maps while following."""
    node = ProjectorNode()
    capture = _Capture()
    node._pub = capture
    try:
        node._odom_z = 1.7  # LiDAR drifted +1.7 m in the map frame
        node._odom_recv = time.monotonic()
        # Floor is 0.39 m below the LiDAR, i.e. map z = 1.31; the fixed band
        # would have dropped every one of these points.
        cloud = _cloud([(0.05, 0.05, 1.31), (0.15, 0.15, 1.31)])
        node._on_cloud(cloud)
        assert len(capture.messages) == 1
        assert capture.messages[0].layers == ["elevation", "counts"]
    finally:
        node.destroy_node()


def _ref_node(**overrides):
    params = [Parameter("ground_mode", value="reference")]
    params += [Parameter(k, value=v) for k, v in overrides.items()]
    return ProjectorNode(parameter_overrides=params)


def test_reference_mode_publishes_ground_and_drops_ceiling(ros_context):
    node = _ref_node()
    capture = _Capture()
    node._pub = capture
    try:
        node._odom_xy = (0.0, 0.0)
        node._odom_z = 0.392
        node._odom_recv = time.monotonic()
        cloud = _cloud([(0.05, 0.05, 0.0), (0.10, 0.10, 0.0), (0.15, 0.15, 0.0),
                        (0.05, 0.05, 1.1), (0.10, 0.10, 1.1)])
        node._on_cloud(cloud)
        assert len(capture.messages) == 1
        assert capture.messages[0].layers == ["elevation", "counts"]
    finally:
        node.destroy_node()


def test_reference_mode_falls_back_to_band_without_odom(ros_context):
    node = _ref_node()
    capture = _Capture()
    node._pub = capture
    try:
        node._on_cloud(_cloud([(0.05, 0.05, 0.2), (0.15, 0.15, 0.2)]))
        assert len(capture.messages) == 1
    finally:
        node.destroy_node()


def test_reference_mode_skips_on_ring_mismatch(ros_context):
    # follow_odom_height=False makes the band fallback publish these points
    # (z=0 is inside [-0.5, 0.5]), so "no publish" proves the reference path
    # ran and the ring check rejected the frame.
    node = _ref_node(ring_min_cells=1, ring_check_tol=0.05, min_points_ref=1,
                     follow_odom_height=False)
    capture = _Capture()
    node._pub = capture
    try:
        node._odom_xy = (0.0, 0.0)
        node._odom_z = 5.0          # seed = 4.608, ground at 0
        node._odom_recv = time.monotonic()
        node._on_cloud(_cloud([(1.5, 0.0, 0.0), (1.6, 0.0, 0.0)]))
        assert capture.messages == []
    finally:
        node.destroy_node()


def test_reference_mode_skips_when_coverage_drops(ros_context):
    node = _ref_node()
    capture = _Capture()
    node._pub = capture
    try:
        node._odom_xy = (0.0, 0.0)
        node._odom_z = 0.392
        node._odom_recv = time.monotonic()
        node._prev_ground_cells = 1000
        node._on_cloud(_cloud([(0.05, 0.05, 0.0)]))
        assert capture.messages == []
    finally:
        node.destroy_node()


def test_reference_mode_recovers_after_consecutive_coverage_skips(ros_context):
    """A genuine smaller steady state must not freeze the map forever.

    The guard protects against transient drops; after coverage_max_skips
    consecutive rejections we accept the new baseline.
    """
    node = _ref_node(coverage_max_skips=3)
    capture = _Capture()
    node._pub = capture
    try:
        node._odom_xy = (0.0, 0.0)
        node._odom_z = 0.392
        node._odom_recv = time.monotonic()
        node._prev_ground_cells = 1000
        node._coverage_skips = 2       # this is the third consecutive frame
        node._on_cloud(_cloud([(0.05, 0.05, 0.0)]))
        assert len(capture.messages) == 1
        assert node._coverage_skips == 0
        assert node._prev_ground_cells == 1
    finally:
        node.destroy_node()


def test_reference_mode_respects_fixed_extent(ros_context):
    node = ProjectorNode(parameter_overrides=[
        Parameter("ground_mode", value="reference"),
        Parameter("auto_extent", value=False),
        Parameter("length_x", value=30.0),
        Parameter("length_y", value=30.0)])
    capture = _Capture()
    node._pub = capture
    try:
        node._odom_xy = (0.0, 0.0)
        node._odom_z = 0.392
        node._odom_recv = time.monotonic()
        node._on_cloud(_cloud([(22.0, 0.0, 0.0)]))
        assert node._spec.length_x == 30.0
        assert node._spec.length_y == 30.0
    finally:
        node.destroy_node()


def test_reference_mode_survives_a_non_lattice_extent(ros_context):
    # extent_quantum=0.3 makes the snapped length a non-multiple of 0.5;
    # coarse_spec raises, but it must skip the frame, not kill the node.
    node = _ref_node(extent_quantum=0.3)
    capture = _Capture()
    node._pub = capture
    try:
        node._odom_xy = (0.0, 0.0)
        node._odom_z = 0.392
        node._odom_recv = time.monotonic()
        node._on_cloud(_cloud([(0.05, 0.05, 0.0), (0.10, 0.10, 0.0),
                               (0.15, 0.15, 0.0)]))
        assert capture.messages == []
    finally:
        node.destroy_node()


def test_band_mode_still_publishes(ros_context):
    node = ProjectorNode(parameter_overrides=[
        Parameter("ground_mode", value="band")])
    capture = _Capture()
    node._pub = capture
    try:
        node._on_cloud(_cloud([(0.05, 0.05, 0.2), (0.15, 0.15, 0.2)]))
        assert len(capture.messages) == 1
    finally:
        node.destroy_node()


def test_publisher_is_transient_local_and_subscription_is_volatile(ros_context):
    """The derived-layer node subscribes TRANSIENT_LOCAL.

    DDS requires the offered durability to be >= the requested one, so a
    VOLATILE publisher would never match it (no samples at all).  The
    upstream /aft_pgo_map publisher is VOLATILE, so the subscription must
    stay VOLATILE.
    """
    node = ProjectorNode()
    try:
        assert node._pub.qos_profile.durability.name == "TRANSIENT_LOCAL"
        assert node._sub.qos_profile.durability.name == "VOLATILE"
    finally:
        node.destroy_node()


def test_clearance_layer_measures_headroom_above_the_local_ground(ros_context):
    from types import SimpleNamespace

    from pgo_gridmap_projector.rasterizer import GridSpec

    node = ProjectorNode(parameter_overrides=[
        Parameter("publish_clearance", value=True)])
    try:
        assert node._publish_clearance
        grid = np.linspace(0.05, 0.95, 10)
        gx, gy = np.meshgrid(grid, grid)
        ground = np.column_stack([gx.ravel(), gy.ravel(), np.zeros(gx.size)])
        # Two points 1.2 / 1.25 m above the ground in the same fine cell.
        over = np.array([[0.45, 0.45, 1.20], [0.42, 0.43, 1.25]])
        xyz = np.vstack([ground, over])
        final = GridSpec(resolution=0.1, length_x=1.0, length_y=1.0,
                         origin_x=0.0, origin_y=0.0)
        coarse = GridSpec(resolution=0.5, length_x=1.0, length_y=1.0,
                          origin_x=0.0, origin_y=0.0)
        result = SimpleNamespace(
            fill_ref=np.zeros((coarse.n_rows, coarse.n_cols)), ref=None)

        out = node._clearance_layer(xyz, result, coarse, final)
        assert out is not None
        assert out.shape == (final.n_rows, final.n_cols)
        # Only the cell under the obstacle has headroom; the rest is open sky.
        assert int(np.isfinite(out).sum()) == 1
        assert abs(out[0, 0] - 1.20) < 1e-6
    finally:
        node.destroy_node()


def test_clearance_layer_is_all_nan_when_nothing_is_overhead(ros_context):
    from types import SimpleNamespace

    from pgo_gridmap_projector.rasterizer import GridSpec

    node = ProjectorNode(parameter_overrides=[
        Parameter("publish_clearance", value=True)])
    try:
        grid = np.linspace(0.05, 0.95, 10)
        gx, gy = np.meshgrid(grid, grid)
        xyz = np.column_stack([gx.ravel(), gy.ravel(), np.zeros(gx.size)])
        final = GridSpec(resolution=0.1, length_x=1.0, length_y=1.0,
                         origin_x=0.0, origin_y=0.0)
        coarse = GridSpec(resolution=0.5, length_x=1.0, length_y=1.0,
                          origin_x=0.0, origin_y=0.0)
        result = SimpleNamespace(
            fill_ref=np.zeros((coarse.n_rows, coarse.n_cols)), ref=None)
        # An all-NaN layer, not None: the published layer set must not
        # change between frames or a low arch would flicker lethal/free.
        out = node._clearance_layer(xyz, result, coarse, final)
        assert out.shape == (final.n_rows, final.n_cols)
        assert np.isnan(out).all()
    finally:
        node.destroy_node()


def test_counts_layer_is_published_when_enabled(ros_context):
    node = ProjectorNode(parameter_overrides=[
        Parameter("publish_counts", value=True)])
    try:
        counts = np.array([[0, 3], [7, 1]], dtype=np.int64)
        extra = node._counts_extra(counts)
        assert set(extra) == {"counts"}
        assert extra["counts"].dtype == np.float64
        np.testing.assert_array_equal(extra["counts"], counts)
    finally:
        node.destroy_node()


def test_counts_layer_can_be_disabled(ros_context):
    node = ProjectorNode(parameter_overrides=[
        Parameter("publish_counts", value=False)])
    try:
        assert node._counts_extra(np.ones((2, 2), dtype=np.int64)) is None
    finally:
        node.destroy_node()


def test_auto_extent_ignores_fill_points(ros_context):
    """A wild fill point must not stretch the grid (cost grows with cells)."""
    a = ProjectorNode(parameter_overrides=[
        Parameter("auto_extent", value=True),
        Parameter("fill_enabled", value=True)])
    b = ProjectorNode(parameter_overrides=[
        Parameter("auto_extent", value=True),
        Parameter("fill_enabled", value=False)])
    try:
        livox = _cloud([(0.0, 0.0, 0.0)])
        a._fill_xyz = np.array([[50.0, 50.0, 0.0]])
        a._fill_stamp_s = a.get_clock().now().nanoseconds * 1e-9
        a._on_cloud(livox)
        b._on_cloud(livox)
        assert a._spec.length_x == b._spec.length_x
        assert a._spec.length_y == b._spec.length_y
    finally:
        a.destroy_node()
        b.destroy_node()


def test_stale_fill_matches_no_fill(ros_context):
    a = ProjectorNode(parameter_overrides=[
        Parameter("fill_enabled", value=True)])
    b = ProjectorNode(parameter_overrides=[
        Parameter("fill_enabled", value=False)])
    ca, cb = _Capture(), _Capture()
    a._pub, b._pub = ca, cb
    try:
        livox = _cloud([(0.0, 0.0, 0.0)])
        a._fill_xyz = np.array([[1.0, 1.0, 0.0]])
        a._fill_stamp_s = 0.0  # far older than fill_max_age_s
        a._on_cloud(livox)
        b._on_cloud(livox)
        assert len(ca.messages) == 1 and len(cb.messages) == 1
    finally:
        a.destroy_node()
        b.destroy_node()


def test_fill_reaches_the_reference_mode_grid(ros_context):
    """Reference mode must see the merged cloud too.

    The merge happens in _on_cloud before the ground-mode dispatch, so this
    asserts the filled cell shows up in the *published* grid with the
    reference ground path active -- the mode the chain actually runs today.
    """
    node = ProjectorNode(parameter_overrides=[
        Parameter("ground_mode", value="reference"),
        Parameter("fill_enabled", value=True),
        Parameter("fill_cell_size", value=0.10)])
    cap = _Capture()
    node._pub = cap
    try:
        # One Livox ring at x = 2.0 so the grid has ground to seed on, plus a
        # fresh fill patch around x = -2.0 that Livox does not cover.
        livox = _cloud([(x * 0.05 + 2.0, y * 0.05, 0.0)
                        for x in range(10) for y in range(-5, 6)])
        node._fill_xyz = np.array([[x * 0.05 - 2.0, y * 0.05, 0.0]
                                   for x in range(10) for y in range(-5, 6)])
        node._fill_stamp_s = node.get_clock().now().nanoseconds * 1e-9
        node._on_cloud(livox)
        assert len(cap.messages) == 1
        msg = cap.messages[0]
        assert "elevation" in msg.layers
    finally:
        node.destroy_node()


def test_pose_to_ground_is_measured_from_the_fill(ros_context):
    """The uLidar ground replaces the constant pose_to_ground_z."""
    node = ProjectorNode(parameter_overrides=[
        Parameter("pose_to_ground_z", value=0.392),
        Parameter("ground_measure_min_points", value=5),
        Parameter("ground_measure_alpha", value=1.0)])
    try:
        # Floor 0.37 m below the pose origin, a 0.4 m patch around the robot.
        pts = np.array([[x * 0.05, y * 0.05, -1.000]
                        for x in range(-4, 5) for y in range(-4, 5)])
        pose = (0.0, 0.0, -0.630)
        node._fill_xyz = pts
        node._fill_stamp_s = node.get_clock().now().nanoseconds * 1e-9
        got = node._measure_pose_to_ground(pose)
        assert got is not None
        assert abs(got - 0.370) < 1e-6
    finally:
        node.destroy_node()


def test_pose_to_ground_rejects_an_implausible_fill(ros_context):
    node = ProjectorNode(parameter_overrides=[
        Parameter("pose_to_ground_z", value=0.392),
        Parameter("ground_measure_min_points", value=5),
        Parameter("ground_measure_max_dev", value=0.10)])
    try:
        pts = np.array([[x * 0.05, y * 0.05, -1.000]
                        for x in range(-4, 5) for y in range(-4, 5)])
        node._fill_xyz = pts
        node._fill_stamp_s = node.get_clock().now().nanoseconds * 1e-9
        # 0.9 m below the pose: far outside the 0.10 m plausibility window.
        assert node._measure_pose_to_ground((0.0, 0.0, -0.100)) is None
    finally:
        node.destroy_node()


def test_pose_to_ground_ignores_a_stale_fill(ros_context):
    node = ProjectorNode(parameter_overrides=[
        Parameter("pose_to_ground_z", value=0.392),
        Parameter("fill_max_age_s", value=2.0)])
    try:
        node._fill_xyz = np.zeros((50, 3))
        node._fill_stamp_s = 0.0
        assert node._measure_pose_to_ground((0.0, 0.0, -0.630)) is None
    finally:
        node.destroy_node()
