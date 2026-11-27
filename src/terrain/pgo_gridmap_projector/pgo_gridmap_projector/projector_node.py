"""Project the PGO-corrected full map into a GridMap elevation layer.

Subscribes to /aft_pgo_map (the loop-closure-corrected full map, rebuilt
every second by simple_loop_closure) and publishes a one-layer
grid_map_msgs/GridMap on /elevation_map_raw.

The node is stateless: each incoming frame is rasterized into a fresh
grid. That is the whole point -- the upstream PGO correction is then
always reflected in the output, and no cell can hold a stale value from
before a loop closure.

No TF lookups are performed. The incoming cloud is relabelled to
`output_frame_id` on the assumption that the upstream's camera_init -> map
transform is the identity static TF it publishes.

Two ground modes (``ground_mode``):

* ``reference`` (default): a per-cell ground reference seeded from the PGO
  pose and propagated through the map's own points (see ``ground.py``).
  This decouples the gate from the global z drift: a cell is accepted at the
  height its own points were observed, so a multi-metre global offset no
  longer drops whole regions.  It does NOT flatten the drift; inter-region
  height differences remain (spec section 2).
* ``band``: the previous global gate.  With ``follow_odom_height`` it slides
  a +/- offset around the robot's current LiDAR height instead of the map
  origin; ``follow_odom_height: false`` restores the fixed-origin behaviour.
  Kept as the A/B baseline and as the fallback when the pose is missing or
  stale.
"""
from __future__ import annotations

import time
from dataclasses import replace
from typing import Optional

import numpy as np
import rclpy
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2, PointField

from .extent import ExtentConfig, update_extent, window_spec
from .fill_merge import merge_fill_points
from .gridmap_msg import build_gridmap_msg
from .ground import (
    GroundConfig,
    build_reference,
    cave_profile,
    coarse_spec,
    coverage_ok,
    despike_sparse,
    fill_sparse_from_reference,
    output_band_counts,
    select_ground,
)
from .rasterizer import (
    AGGREGATIONS,
    GridSpec,
    RasterError,
    native_available,
    rasterize,
)

# The upstream simple_loop_closure republishes /aft_pgo_map at 1 Hz
# (simple_loop_closure_node.cpp: visualizeThread uses rclcpp::Rate(1)).
SLOW_FRAME_WARN_SEC = 1.0


def decompress_points(msg: PointCloud2) -> np.ndarray:
    """Extract an (N, 3) float64 xyz array from a PointCloud2.

    Uses the field offsets from the message rather than assuming a fixed
    layout, because publishers vary (some put intensity first).
    """
    if msg.width == 0 or msg.height == 0 or len(msg.data) == 0:
        return np.zeros((0, 3), dtype=np.float64)

    offsets = {}
    for field in msg.fields:
        if field.name in ("x", "y", "z"):
            offsets[field.name] = (field.offset, field.datatype)
    missing = {"x", "y", "z"} - set(offsets)
    if missing:
        raise ValueError(f"PointCloud2 is missing fields: {sorted(missing)}")

    item = np.dtype(np.float32).itemsize
    if msg.point_step <= 0 or msg.point_step % item != 0:
        raise ValueError(
            f"point_step {msg.point_step} is not a positive multiple of {item}")
    for name, (offset, field_dtype) in offsets.items():
        if field_dtype != PointField.FLOAT32:
            raise ValueError(f"field {name} is not FLOAT32")
        if offset < 0 or offset + item > msg.point_step:
            raise ValueError(
                f"field {name} at offset {offset} does not fit in a "
                f"{msg.point_step}-byte point")

    buf = np.frombuffer(msg.data, dtype=np.float32)
    # Only the first point_step bytes of each point belong to that point.
    n_points = len(msg.data) // msg.point_step
    if n_points == 0:
        return np.zeros((0, 3), dtype=np.float64)
    usable = n_points * msg.point_step
    flat = buf[:usable].reshape(n_points, msg.point_step // item)

    cols = []
    for name in ("x", "y", "z"):
        offset, _ = offsets[name]
        cols.append(flat[:, offset // item].astype(np.float64))
    return np.stack(cols, axis=1)


class ProjectorNode(Node):
    """Rasterizes /aft_pgo_map into a GridMap elevation layer."""

    def __init__(self, **kwargs) -> None:
        super().__init__("pgo_gridmap_projector", **kwargs)

        self.declare_parameter("input_topic", "/aft_pgo_map")
        self.declare_parameter("output_topic", "/elevation_map_raw")
        self.declare_parameter("output_frame_id", "map")
        self.declare_parameter("resolution", 0.10)
        # Legacy fixed geometry. Only used when auto_extent is false; kept so
        # the 30 m behaviour can be restored for A/B comparisons. See
        # config/projector_params.yaml for why 30 m was the old value.
        self.declare_parameter("length_x", 30.0)
        self.declare_parameter("length_y", 30.0)
        self.declare_parameter("origin_x", 0.0)
        self.declare_parameter("origin_y", 0.0)
        # Adaptive geometry: the grid follows the observed bbox (spec 6.2).
        self.declare_parameter("auto_extent", True)
        self.declare_parameter("extent_margin", 2.0)
        self.declare_parameter("extent_quantum", 1.0)
        self.declare_parameter("extent_min_length", 10.0)
        self.declare_parameter("extent_max_length", 120.0)
        # 2026-11-14: fixed-size window centred on the robot. The accumulated
        # map reaches 100x100 m on a long corridor and every downstream node
        # is O(cells); at 1.0M cells the 2 Hz chain fell to 0.3-1 Hz. The
        # global costmap only consumes a 60x60 m rolling viewport, so a
        # robot-centred window slightly larger than that covers the consumer
        # while keeping the cell count fixed.
        self.declare_parameter("window_centered_on_pose", False)
        self.declare_parameter("window_length", 70.0)
        self.declare_parameter("window_quantum", 1.0)
        # Optional vertical-clearance layer (off by default). It answers a
        # question the elevation surface cannot: how much headroom is above
        # this cell. Only meaningful in ground_mode=reference.
        # Per-cell point count layer. The bridge uses it to tell a real
        # obstacle (many points) from a grazing-angle far point that only
        # looks like a 0.3 m step (1-2 points). ~2 MB/frame at 70 m.
        self.declare_parameter("publish_counts", True)
        self.declare_parameter("publish_clearance", False)
        self.declare_parameter("clearance_max_above", 1.5)
        self.declare_parameter("clearance_min_above", 0.30)
        self.declare_parameter("extent_shrink", False)
        self.declare_parameter("extent_outlier_trim", 1)
        self.declare_parameter("z_aggregation", "median")
        self.declare_parameter("ignore_points_above", 0.5)
        self.declare_parameter("ignore_points_below", -0.5)
        # Height-gate anchor.  With follow_odom_height the two values above
        # are OFFSETS around the current PGO LiDAR height, so the band tracks
        # the floor instead of staying glued to the map origin.  The PGO
        # odometry and the map are built from the same optimized poses, so
        # their z is directly comparable without any TF lookup.
        self.declare_parameter("follow_odom_height", True)
        self.declare_parameter("odom_topic", "/aft_pgo_odom")
        self.declare_parameter("odom_timeout_sec", 2.0)
        # Optional uLidar (Go2 built-in L2) near-field ground fill.  The fill
        # points are merged into the *input* cloud, before either ground mode
        # runs, and only in xy cells that no Livox point occupies.  That makes
        # it mode-agnostic: band and reference both see one merged cloud.
        self.declare_parameter("fill_topic", "/utlidar_terrain_fill")
        self.declare_parameter("fill_enabled", True)
        # 2026-11-19: 2.0 -> 5.0.  A stale fill is dropped entirely and the
        # near-field cells (which only the fill observes) revert to UNKNOWN;
        # on the standing bag the 0-0.8 m ring collapsed from 113 to 14 valid
        # cells once the age passed 2 s.  The fill's own rolling window is
        # 5 s, so accepting 5 s never mixes in older data than that.
        self.declare_parameter("fill_max_age_s", 5.0)
        self.declare_parameter("fill_cell_size", 0.10)
        # Measure the pose->ground distance from the uLidar fill instead of
        # trusting the constant.  pose_to_ground_z is really "how far the
        # floor is below the PGO pose origin", which changes with the stance
        # (body_height moves a few cm, more when the dog crouches); the fill
        # cloud is ground-only and in the map frame, so its median z around
        # the robot is a direct measurement.  A measurement further than
        # ground_measure_max_dev from the constant is rejected, so a broken
        # fill can never drag the reference with it.
        self.declare_parameter("ground_measure_from_fill", True)
        self.declare_parameter("ground_measure_radius", 1.5)
        self.declare_parameter("ground_measure_min_points", 20)
        self.declare_parameter("ground_measure_alpha", 0.2)
        self.declare_parameter("ground_measure_max_dev", 0.30)
        self.declare_parameter("ground_drift_warn_m", 0.05)

        # ---- Per-cell ground reference (spec 2026-11-07) ----
        # band: the pre-existing global height gate (A/B baseline).
        # reference: per-cell ground reference seeded from the PGO pose.
        self.declare_parameter("ground_mode", "reference")
        self.declare_parameter("profile", "default")
        self.declare_parameter("pose_to_ground_z", 0.392)
        self.declare_parameter("ref_resolution", 0.5)
        self.declare_parameter("seed_radius", 1.5)
        self.declare_parameter("seed_ground_tol", 0.30)
        self.declare_parameter("clearance_min", 0.8)
        self.declare_parameter("ring_inner", 1.2)
        self.declare_parameter("ring_outer", 3.0)
        self.declare_parameter("ring_check_tol", 0.15)
        self.declare_parameter("ring_min_cells", 8)
        self.declare_parameter("ring_cluster_gap", 0.40)
        self.declare_parameter("ring_reject_tol", 1.00)
        self.declare_parameter("ring_above_tol", 0.30)
        self.declare_parameter("ref_band_above", 0.30)
        self.declare_parameter("ref_band_below", 0.60)
        self.declare_parameter("out_band_above", 0.45)
        self.declare_parameter("out_band_below", 0.40)
        self.declare_parameter("min_points_ref", 3)
        self.declare_parameter("max_gap_cells", 4)
        self.declare_parameter("p_low", 20.0)
        self.declare_parameter("slope_exception_enabled", False)
        self.declare_parameter("slope_except_above", 0.45)
        self.declare_parameter("upright_spread_max", 0.25)
        self.declare_parameter("coverage_min_ratio", 0.60)
        self.declare_parameter("coverage_min_cells", 200)
        self.declare_parameter("coverage_max_skips", 3)
        self.declare_parameter("spike_filter_enabled", True)
        self.declare_parameter("spike_min_support", 3)
        self.declare_parameter("min_points_output", 3)

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._output_frame_id = str(self.get_parameter("output_frame_id").value)
        self._aggregation = str(self.get_parameter("z_aggregation").value)
        self._z_max = float(self.get_parameter("ignore_points_above").value)
        self._z_min = float(self.get_parameter("ignore_points_below").value)
        self._fill_topic = str(self.get_parameter("fill_topic").value)
        self._fill_enabled = bool(self.get_parameter("fill_enabled").value)
        self._fill_max_age_s = float(
            self.get_parameter("fill_max_age_s").value)
        self._fill_cell_size = float(
            self.get_parameter("fill_cell_size").value)
        self._fill_xyz: Optional[np.ndarray] = None
        self._fill_stamp_s: Optional[float] = None
        self._measure_ground = bool(
            self.get_parameter("ground_measure_from_fill").value)
        self._measure_radius = float(
            self.get_parameter("ground_measure_radius").value)
        self._measure_min_points = int(
            self.get_parameter("ground_measure_min_points").value)
        self._measure_alpha = float(
            self.get_parameter("ground_measure_alpha").value)
        self._measure_max_dev = float(
            self.get_parameter("ground_measure_max_dev").value)
        self._drift_warn_m = float(
            self.get_parameter("ground_drift_warn_m").value)
        self._pose_to_ground_nominal = float(
            self.get_parameter("pose_to_ground_z").value)
        self._measured_ptg: Optional[float] = None

        self._follow_odom = bool(self.get_parameter("follow_odom_height").value)
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._odom_timeout = float(self.get_parameter("odom_timeout_sec").value)
        # Latest observed LiDAR z in the map frame and when it arrived.
        # Deliberately keyed on ARRIVAL time, not on the message stamp: the
        # upstream publishes /aft_pgo_map with this->now() while /aft_pgo_odom
        # keeps the LiDAR stamp, so during bag replay the two stamps differ by
        # the recording age and a stamp-based match would never hit.
        self._odom_z: Optional[float] = None
        self._odom_xy: Optional[tuple[float, float]] = None
        self._odom_recv: Optional[float] = None

        self._ground_mode = str(self.get_parameter("ground_mode").value)
        if self._ground_mode not in ("reference", "band"):
            raise ValueError(
                f"ground_mode must be 'reference' or 'band', "
                f"got {self._ground_mode!r}")
        self._ground_cfg = self._build_ground_config()
        # Previous published frame's ground-cell count, for the coverage guard.
        self._prev_ground_cells: Optional[int] = None
        self._coverage_skips = 0
        # Output-band loss counters, reported once a minute at INFO so a bag
        # run can see what the band drops (spec Focus 5: the > out_band_above
        # gap must be measurable, not silent).
        self._band_above_total = 0
        self._band_below_total = 0
        self._last_band_report = time.monotonic()

        if self._aggregation not in AGGREGATIONS:
            raise ValueError(
                f"z_aggregation must be one of {AGGREGATIONS}, "
                f"got {self._aggregation!r}"
            )

        resolution = float(self.get_parameter("resolution").value)
        self._auto_extent = bool(self.get_parameter("auto_extent").value)
        self._window_mode = bool(
            self.get_parameter("window_centered_on_pose").value)
        self._window_length = float(self.get_parameter("window_length").value)
        # The coarse reference grid must tile the output lattice exactly, so
        # the window length has to be a multiple of ref_resolution too: a 70 m
        # window with a 0.3 m reference becomes 69.9 m (699 x 0.1 m cells).
        _ref_res = self._ground_cfg.ref_resolution
        if abs(self._window_length / _ref_res
               - round(self._window_length / _ref_res)) > 1e-9:
            snapped = round(self._window_length / _ref_res - 0.5) * _ref_res
            self.get_logger().warn(
                f"window_length={self._window_length} is not a multiple of "
                f"ref_resolution={_ref_res}; using {snapped:.1f} m so the "
                "reference grid nests in the output grid")
            self._window_length = snapped
        self._window_quantum = float(self.get_parameter("window_quantum").value)
        self._publish_counts = bool(
            self.get_parameter("publish_counts").value)
        self._publish_clearance = bool(
            self.get_parameter("publish_clearance").value)
        self._clearance_max_above = float(
            self.get_parameter("clearance_max_above").value)
        self._clearance_min_above = float(
            self.get_parameter("clearance_min_above").value)
        if self._publish_clearance and self._ground_mode != "reference":
            # Without a per-cell ground reference there is no "0" to measure
            # headroom from: a global band would make the layer meaningless
            # (every cell would report the same absolute z).
            self.get_logger().error(
                "publish_clearance requires ground_mode=reference; "
                "disabling the clearance layer")
            self._publish_clearance = False
        if self._window_mode and self._window_length <= 0.0:
            self.get_logger().error(
                f"window_length={self._window_length} is invalid; "
                "disabling the robot-centred window")
            self._window_mode = False
        self._extent_cfg = ExtentConfig(
            resolution=resolution,
            margin=float(self.get_parameter("extent_margin").value),
            quantum=float(self.get_parameter("extent_quantum").value),
            min_length=float(self.get_parameter("extent_min_length").value),
            max_length=float(self.get_parameter("extent_max_length").value),
            shrink=bool(self.get_parameter("extent_shrink").value),
            outlier_trim=int(self.get_parameter("extent_outlier_trim").value),
        )

        if self._auto_extent:
            self._spec = GridSpec(
                resolution=resolution,
                length_x=self._extent_cfg.min_length,
                length_y=self._extent_cfg.min_length,
                origin_x=0.0,
                origin_y=0.0,
            )
        else:
            self._spec = GridSpec(
                resolution=resolution,
                length_x=float(self.get_parameter("length_x").value),
                length_y=float(self.get_parameter("length_y").value),
                origin_x=float(self.get_parameter("origin_x").value),
                origin_y=float(self.get_parameter("origin_y").value),
            )

        # The subscription keeps the upstream contract: simple_loop_closure
        # publishes /aft_pgo_map RELIABLE + VOLATILE, and requesting
        # TRANSIENT_LOCAL here would make the pair incompatible (DDS requires
        # offered >= requested durability).
        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        # The publisher is TRANSIENT_LOCAL so late-joining consumers (the
        # derived-layer node subscribes TRANSIENT_LOCAL) match and get the
        # last frame.  A VOLATILE publisher here is incompatible with that
        # subscription and delivers zero samples.  The old filter chain's
        # VOLATILE subscriber remains compatible with a TRANSIENT_LOCAL
        # publisher.
        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._sub = self.create_subscription(
            PointCloud2, self._input_topic, self._on_cloud, sub_qos)
        self._fill_sub = None
        if self._fill_enabled:
            self._fill_sub = self.create_subscription(
                PointCloud2, self._fill_topic, self._on_fill,
                qos_profile_sensor_data)
        self._pub = self.create_publisher(GridMap, self._output_topic, pub_qos)

        self._odom_sub = None
        if (self._follow_odom or self._ground_mode == "reference"
                or self._window_mode):
            # BEST_EFFORT matches any publisher QoS (RELIABLE included); the
            # upstream publishes depth 1 at LiDAR rate.
            self._odom_sub = self.create_subscription(
                Odometry, self._odom_topic, self._on_odom,
                QoSProfile(
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    depth=10,
                ))

        self.get_logger().info(
            f"pgo_gridmap_projector up: {self._input_topic} -> "
            f"{self._output_topic}, frame='{self._output_frame_id}', "
            f"auto_extent={self._auto_extent}, "
            f"grid={self._spec.length_x}x{self._spec.length_y} @ "
            f"{self._spec.resolution} m ({self._spec.n_rows}x"
            f"{self._spec.n_cols} cells), agg={self._aggregation}, "
            f"z in [{self._z_min}, {self._z_max}]"
            + (f" + lidar z from {self._odom_topic}"
               if self._follow_odom else " (fixed to the map origin)")
            + (f", window(pose)={self._window_length:.0f} m"
               if self._window_mode else "")
            + f", ground_mode={self._ground_mode}"
            + (f" profile={self._ground_profile}"
               if self._ground_profile != "default" else "")
            + f", rasterizer={'native' if native_available() else 'numpy'}"
        )

    def _build_ground_config(self) -> GroundConfig:
        """Read the per-cell ground parameters once, then apply the profile."""
        cfg = GroundConfig(
            ref_resolution=float(self.get_parameter("ref_resolution").value),
            pose_to_ground_z=float(self.get_parameter("pose_to_ground_z").value),
            seed_radius=float(self.get_parameter("seed_radius").value),
            seed_ground_tol=float(self.get_parameter("seed_ground_tol").value),
            clearance_min=float(self.get_parameter("clearance_min").value),
            ring_inner=float(self.get_parameter("ring_inner").value),
            ring_outer=float(self.get_parameter("ring_outer").value),
            ring_check_tol=float(self.get_parameter("ring_check_tol").value),
            ring_min_cells=int(self.get_parameter("ring_min_cells").value),
            ring_cluster_gap=float(
                self.get_parameter("ring_cluster_gap").value),
            ring_reject_tol=float(
                self.get_parameter("ring_reject_tol").value),
            ring_above_tol=float(
                self.get_parameter("ring_above_tol").value),
            ref_band_above=float(self.get_parameter("ref_band_above").value),
            ref_band_below=float(self.get_parameter("ref_band_below").value),
            out_band_above=float(self.get_parameter("out_band_above").value),
            out_band_below=float(self.get_parameter("out_band_below").value),
            min_points_ref=int(self.get_parameter("min_points_ref").value),
            max_gap_cells=int(self.get_parameter("max_gap_cells").value),
            p_low=float(self.get_parameter("p_low").value),
            slope_exception_enabled=bool(
                self.get_parameter("slope_exception_enabled").value),
            slope_except_above=float(
                self.get_parameter("slope_except_above").value),
            upright_spread_max=float(
                self.get_parameter("upright_spread_max").value),
            coverage_min_ratio=float(
                self.get_parameter("coverage_min_ratio").value),
            coverage_min_cells=int(
                self.get_parameter("coverage_min_cells").value),
            coverage_max_skips=int(
                self.get_parameter("coverage_max_skips").value),
            spike_filter_enabled=bool(
                self.get_parameter("spike_filter_enabled").value),
            spike_min_support=int(
                self.get_parameter("spike_min_support").value),
            min_points_output=int(
                self.get_parameter("min_points_output").value),
        )
        profile = str(self.get_parameter("profile").value)
        self._ground_profile = profile
        if profile == "cave":
            cfg = cave_profile(cfg)
        elif profile != "default":
            raise ValueError(
                f"profile must be 'default' or 'cave', got {profile!r}")
        res = cfg.ref_resolution
        out_res = float(self.get_parameter("resolution").value)
        if res <= 0:
            raise ValueError(f"ref_resolution must be > 0, got {res}")
        ratio = res / out_res
        if abs(ratio - round(ratio)) > 1e-9 or round(ratio) < 1:
            raise ValueError(
                f"ref_resolution={res} must be an integer multiple of the "
                f"{out_res} m output lattice")
        return cfg

    def _latest_pose(self) -> Optional[tuple[float, float, float]]:
        """Fresh PGO pose, or None when the reference mode must fall back."""
        if (self._odom_xy is None or self._odom_z is None
                or self._odom_recv is None):
            self.get_logger().warn(
                f"no {self._odom_topic} yet, falling back to the band gate",
                throttle_duration_sec=10.0)
            return None
        age = time.monotonic() - self._odom_recv
        if age > self._odom_timeout:
            self.get_logger().warn(
                f"{self._odom_topic} is {age:.1f} s stale, falling back to "
                "the band gate", throttle_duration_sec=10.0)
            return None
        return (self._odom_xy[0], self._odom_xy[1], self._odom_z)

    def _measure_pose_to_ground(self,
                                pose: tuple[float, float, float]
                                ) -> Optional[float]:
        """Pose-origin-to-floor distance measured from the uLidar fill.

        Returns ``None`` (keep the constant) when the fill is missing, stale,
        too sparse around the robot, or implausible.  The last accepted value
        is smoothed so a single bad patch cannot step the reference.
        """
        if not self._measure_ground or self._fill_xyz is None:
            return None
        if self._fill_stamp_s is None:
            return None
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if now_s - self._fill_stamp_s > self._fill_max_age_s:
            return None
        pts = self._fill_xyz
        near = pts[np.hypot(pts[:, 0] - pose[0], pts[:, 1] - pose[1])
                   < self._measure_radius]
        if near.shape[0] < self._measure_min_points:
            return None
        value = float(pose[2]) - float(np.median(near[:, 2]))
        dev = value - self._pose_to_ground_nominal
        if abs(dev) > self._measure_max_dev:
            self.get_logger().warn(
                f"fill ground implies pose_to_ground_z={value:.3f} m, "
                f"{dev:+.3f} m from the nominal {self._pose_to_ground_nominal}"
                f" m; ignoring it and keeping the constant",
                throttle_duration_sec=10.0)
            return None
        if self._measured_ptg is None:
            self._measured_ptg = value
        else:
            self._measured_ptg = ((1.0 - self._measure_alpha) *
                                  self._measured_ptg +
                                  self._measure_alpha * value)
        if abs(dev) > self._drift_warn_m:
            # This is a STANCE measurement, not a drift one: the fill is
            # placed with map<-odom, which is derived from this very pose, so
            # a map-wide z drift moves both terms together and cancels.  What
            # it does show is the real body-to-floor distance changing (gait,
            # crouch, loading).  Vertical map drift is the ring check's job
            # ("ring ground ... differs from the pose seed").
            self.get_logger().info(
                f"pose->ground from the fill: {self._measured_ptg:.3f} m vs "
                f"nominal {self._pose_to_ground_nominal:.3f} m "
                f"({dev:+.3f} m) -- stance change, not map drift",
                throttle_duration_sec=10.0)
        return self._measured_ptg

    def _on_fill(self, msg: PointCloud2) -> None:
        """Cache the latest uLidar ground fill; consumed by _on_cloud only."""
        try:
            xyz = decompress_points(msg)
        except ValueError as exc:
            self.get_logger().error(f"could not decode fill cloud: {exc}")
            return
        frame = msg.header.frame_id.lstrip("/")
        if frame not in (self._output_frame_id, "map", "camera_init"):
            self.get_logger().warn(
                f"fill cloud frame {frame!r} is not {self._output_frame_id!r};"
                f" dropping", throttle_duration_sec=5.0)
            return
        stamp = msg.header.stamp
        self._fill_xyz = xyz
        self._fill_stamp_s = float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _on_odom(self, msg: Odometry) -> None:
        position = msg.pose.pose.position
        self._odom_z = float(position.z)
        self._odom_xy = (float(position.x), float(position.y))
        self._odom_recv = time.monotonic()

    def _band(self) -> tuple[float, float]:
        """Return the height gate (z_min, z_max) for this frame.

        Fixed mode returns the configured map-frame values; follow mode
        slides the same offsets around the current LiDAR height.
        """
        if not self._follow_odom:
            return self._z_min, self._z_max

        if self._odom_z is None:
            self.get_logger().warn(
                f"no {self._odom_topic} yet, falling back to the fixed "
                "map-origin height gate", throttle_duration_sec=10.0)
            return self._z_min, self._z_max

        age = time.monotonic() - self._odom_recv
        if age > self._odom_timeout:
            self.get_logger().warn(
                f"{self._odom_topic} is {age:.1f} s stale; still using its "
                f"last z={self._odom_z:.2f}", throttle_duration_sec=10.0)

        return self._z_min + self._odom_z, self._z_max + self._odom_z

    def _next_spec(self, xyz: np.ndarray,
                   z_min: Optional[float] = None,
                   z_max: Optional[float] = None) -> GridSpec:
        """Return (and remember) the grid geometry for this frame."""
        if self._window_mode:
            spec = window_spec(self._odom_xy, self._window_length,
                               self._spec.resolution, self._window_quantum)
            if spec is not None:
                self._spec = spec
            else:
                self.get_logger().warn(
                    f"no {self._odom_topic} yet; keeping the last "
                    f"{self._spec.length_x:.0f}x{self._spec.length_y:.0f} m "
                    "grid", throttle_duration_sec=10.0)
            return self._spec
        if not self._auto_extent:
            return self._spec
        if z_min is None or z_max is None:
            z_min, z_max = self._band()
        self._spec = update_extent(
            xyz, self._extent_cfg, self._spec, z_min, z_max)
        return self._spec

    def _on_cloud(self, msg: PointCloud2) -> None:
        try:
            livox_xyz = decompress_points(msg)
        except ValueError as exc:
            self.get_logger().error(f"could not decode point cloud: {exc}")
            return

        if livox_xyz.shape[0] == 0:
            self.get_logger().warn("empty point cloud, skipping publish")
            return

        # Merge the uLidar fill into the input cloud here, before either
        # ground mode runs, so band and reference see exactly the same data.
        # The fill is only used in xy cells with no Livox point and never
        # moves the grid extent (see extent_xyz below).
        xyz = livox_xyz
        if self._fill_enabled and self._fill_xyz is not None:
            # The z gate differs per mode: band mode clips around the odom z,
            # reference mode has no such gate (it filters against the per-cell
            # reference further down).  Clipping the fill with the band gate
            # in reference mode would reject fill the mode would have taken,
            # so only apply it where the mode itself applies it.  The fill
            # node already publishes ground-only points.
            if self._ground_mode == "reference":
                z_min, z_max = -np.inf, np.inf
            else:
                z_min, z_max = self._band()
            now_s = self.get_clock().now().nanoseconds * 1e-9
            xyz, kept, dropped = merge_fill_points(
                livox_xyz, self._fill_xyz, self._fill_stamp_s, now_s,
                self._fill_max_age_s, self._fill_cell_size, z_min, z_max)
            self.get_logger().debug(
                f"livox={livox_xyz.shape[0]} fill_kept={kept} "
                f"fill_dropped={dropped}")

        if self._ground_mode == "reference":
            self._on_cloud_reference(xyz, msg, extent_xyz=livox_xyz)
        else:
            self._on_cloud_band(xyz, msg, extent_xyz=livox_xyz)

    def _on_cloud_band(self, xyz: np.ndarray, msg: PointCloud2,
                       extent_xyz: Optional[np.ndarray] = None) -> None:
        """Pre-existing global height gate; kept intact as the A/B baseline."""
        started = time.monotonic()
        z_min, z_max = self._band()
        spec = self._next_spec(
            xyz if extent_xyz is None else extent_xyz, z_min, z_max)
        try:
            elevation, counts = rasterize(
                xyz, spec, aggregation=self._aggregation,
                z_min=z_min, z_max=z_max, return_counts=True)
        except RasterError as exc:
            self.get_logger().error(f"rasterization failed: {exc}")
            return

        if np.all(np.isnan(elevation)):
            # Do not publish an all-NaN map: consumers would read it as
            # "everything unknown" rather than "no new data".
            self.get_logger().warn(
                f"all points filtered out by the z gate "
                f"[{z_min:.2f}, {z_max:.2f}], skipping publish")
            return
        self._publish_grid(
            elevation, spec, msg, started, xyz.shape[0],
            extra_layers=self._counts_extra(counts))

    def _on_cloud_reference(self, xyz: np.ndarray, msg: PointCloud2,
                            extent_xyz: Optional[np.ndarray] = None) -> None:
        """Per-cell ground reference: seed from the pose, flood, then filter."""
        started = time.monotonic()
        pose = self._latest_pose()
        if pose is None:
            # The band path has no per-cell ground reference, so it cannot
            # build the clearance layer either: in a cave this frame loses
            # the overhead check entirely.  Say so, otherwise the only hint
            # is a generic "falling back to the band gate".
            if self._publish_clearance:
                self.get_logger().warn(
                    "clearance layer is NOT published on this frame: it needs "
                    "the per-cell ground reference, and the band fallback has "
                    "none", throttle_duration_sec=10.0)
            return self._on_cloud_band(xyz, msg, extent_xyz=extent_xyz)

        try:
            prov = self._next_spec(
                xyz if extent_xyz is None else extent_xyz, -np.inf, np.inf)
            coarse = coarse_spec(prov, self._ground_cfg.ref_resolution)
            # Seed the reference from the measured body-to-floor distance when
            # the uLidar fill provides one; fall back to the constant.
            seed_cfg = self._ground_cfg
            measured = self._measure_pose_to_ground(pose)
            if measured is not None:
                seed_cfg = replace(
                    self._ground_cfg, pose_to_ground_z=measured)
            result = build_reference(xyz, pose, coarse, seed_cfg)
        except ValueError as exc:
            # e.g. a non-5:1 extent quantum: skip the frame, do not let the
            # ValueError escape the subscription callback and kill the node.
            self.get_logger().error(
                f"ground reference setup failed: {exc}; skipping frame")
            return
        if not result.ring_ok:
            self.get_logger().warn(
                f"ring check failed (seed={result.seed_z:+.3f} "
                f"ring={result.ring_z} cells={result.stats['ring_cells']}), "
                "skipping frame", throttle_duration_sec=5.0)
            return
        if result.ring_z is None:
            self.get_logger().warn(
                f"ring too sparse ({result.stats['ring_cells']} cells); "
                "using the pose seed", throttle_duration_sec=30.0)
        if (result.ring_diff is not None
                and abs(result.ring_diff) > self._ground_cfg.ring_check_tol):
            self.get_logger().warn(
                f"ring ground {result.ring_z:+.2f} differs from the pose seed "
                f"by {result.ring_diff:+.2f} m; using the map ring",
                throttle_duration_sec=10.0)
        mask = select_ground(xyz, result, coarse, self._ground_cfg)
        ground = xyz[mask]
        if ground.shape[0] == 0:
            self.get_logger().warn(
                "no ground points, skipping frame", throttle_duration_sec=5.0)
            return

        # Output the RAW bbox (no z gate), not the accepted-ground bbox.
        # ground is a subset of raw, so this is a superset and stays on the
        # same lattice as the coarse reference; an accepted-only bbox would
        # re-centre the grid and force a second full reference build
        # (measured: 200k points went from ~0.6 s to ~1.9 s on the corridor).
        final = prov

        ground_cells = int(result.stats["ground_cells"])
        if not coverage_ok(ground_cells, self._prev_ground_cells,
                           self._ground_cfg):
            self._coverage_skips += 1
            if self._coverage_skips < self._ground_cfg.coverage_max_skips:
                self.get_logger().warn(
                    f"coverage dropped to {ground_cells} ground cells "
                    f"(previous {self._prev_ground_cells}), skipping frame "
                    f"({self._coverage_skips}/"
                    f"{self._ground_cfg.coverage_max_skips})",
                    throttle_duration_sec=5.0)
                return
            # The drop is persistent, not a transient: adopt the new baseline
            # instead of freezing the output forever.
            self.get_logger().warn(
                f"coverage still {ground_cells} after "
                f"{self._ground_cfg.coverage_max_skips} consecutive "
                "rejections; accepting the new baseline")
            self._coverage_skips = 0
        else:
            self._coverage_skips = 0

        self._spec = final
        try:
            elevation, counts = rasterize(
                ground, final, aggregation=self._aggregation,
                z_min=-np.inf, z_max=np.inf, return_counts=True)
        except RasterError as exc:
            self.get_logger().error(f"rasterization failed: {exc}")
            return
        if self._ground_cfg.spike_filter_enabled:
            # First-pass sparse cells are where isolated height spikes come
            # from; a 3x3 median over dense cells would erase 1-cell walls.
            elevation = despike_sparse(
                elevation, counts, self._ground_cfg.spike_min_support)
        if self._ground_cfg.min_points_output > 0:
            # A 1-2 point cell has a noisy median; use the flood-accepted
            # 0.5 m ground instead.  This is the stronger of the two sparse
            # mechanisms; set min_points_output=0 to A/B the 3x3 median.
            ratio = int(round(coarse.resolution / final.resolution))
            fill_ref = (result.fill_ref if result.fill_ref is not None
                        else result.ref)
            try:
                elevation = fill_sparse_from_reference(
                    elevation, counts, fill_ref, ratio,
                    self._ground_cfg.min_points_output)
            except ValueError as exc:
                self.get_logger().error(
                    f"sparse reference fill failed: {exc}; skipping frame")
                return
        if np.all(np.isnan(elevation)):
            self.get_logger().warn(
                "no ground cells rasterized, skipping publish")
            return

        self._prev_ground_cells = ground_cells
        stats = result.stats
        # Diagnostic only: output_band_counts is O(N) over the whole cloud
        # (122 ms at 1M points) and feeds a 60 s report, so sample it when
        # the report is due instead of every frame.
        now = time.monotonic()
        if now - self._last_band_report >= 60.0:
            above, below = output_band_counts(
                xyz, result, coarse, self._ground_cfg)
            self._band_above_total += above
            self._band_below_total += below
            self.get_logger().info(
                f"ground band dropped since last report: "
                f"{self._band_above_total} points above "
                f"+{self._ground_cfg.out_band_above:.2f} m, "
                f"{self._band_below_total} below "
                f"-{self._ground_cfg.out_band_below:.2f} m")
            self._band_above_total = 0
            self._band_below_total = 0
            self._last_band_report = now
        self.get_logger().debug(
            f"reference: seed={result.seed_z:.2f} ring_z={result.ring_z} "
            f"accepted={stats['accepted_cells']} ground={ground_cells} "
            f"pending={stats['pending_cells']} "
            f"unreachable={stats['unreachable_cells']} "
            f"clearance_hits={stats['clearance_hits']}")
        extra = self._counts_extra(counts)
        if self._publish_clearance:
            # Always include the layer, even when this frame saw no overhead
            # at all: a layer set that changes between frames makes the bridge
            # drop every clearance lethal for that frame, so a low arch would
            # flicker between blocked and free at 2 Hz.
            extra["clearance"] = self._clearance_layer(
                xyz, result, coarse, final)
        self._publish_grid(elevation, final, msg, started, xyz.shape[0],
                           extra_layers=extra)

    def _counts_extra(self, counts: np.ndarray) -> Optional[dict]:
        """Per-cell support as a publishable layer (float32 on the wire)."""
        if not self._publish_counts:
            return None
        return {"counts": np.asarray(counts, dtype=np.float64)}

    def _clearance_layer(self, xyz: np.ndarray, result, coarse: GridSpec,
                         final: GridSpec) -> Optional[np.ndarray]:
        """Headroom per cell: local ground -> lowest overhead point.

        Unlike the elevation surface, this looks at the points *above* the
        ground band: the underside of a cave arch is invisible to a height map
        (it is not the surface the robot drives on), so without this layer the
        costmap cannot tell a 1.0 m tunnel from open sky.

        Returns a (n_rows, n_cols) array of metres, NaN where the cell has no
        overhead (open sky) or no ground reference. An all-NaN layer is
        returned (not None) when nothing was observed overhead, so the
        published layer set never changes between frames.
        """
        blank = np.full((final.n_rows, final.n_cols), np.nan, dtype=np.float64)
        fill_ref = (result.fill_ref if result.fill_ref is not None
                    else result.ref)
        x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        # Same lattice rule as rasterize(): row follows x, col follows y.
        row = np.floor(
            (coarse.origin_x + coarse.half_x - x) / coarse.resolution
        ).astype(np.int64)
        col = np.floor(
            (coarse.origin_y + coarse.half_y - y) / coarse.resolution
        ).astype(np.int64)
        inside = (np.isfinite(xyz).all(axis=1)
                  & (row >= 0) & (row < coarse.n_rows)
                  & (col >= 0) & (col < coarse.n_cols))
        ref = np.full(xyz.shape[0], np.nan, dtype=np.float64)
        ref[inside] = fill_ref[row[inside], col[inside]]

        dz = z - ref
        # Ground band: points within +-0.30 m of the reference ARE the ground
        # (LiDAR noise plus the slope across one 0.5 m coarse cell). Anything
        # higher is overhead. The window top bounds the search: an arch whose
        # underside is above clearance_max_above is not a constraint for the
        # dog and would only make the layer expensive.
        over = (np.isfinite(ref)
                & (dz > self._clearance_min_above)
                & (dz <= self._clearance_max_above))
        if not np.any(over):
            return blank
        # Min, not median: the lowest point of an arch is the one the dog's
        # head would hit.
        try:
            lowest = rasterize(
                xyz[over], final, aggregation="min",
                z_min=-np.inf, z_max=np.inf)
        except RasterError as exc:
            self.get_logger().error(
                f"clearance rasterization failed: {exc}; publishing an "
                "empty clearance layer")
            return blank

        ratio = int(round(coarse.resolution / final.resolution))
        if ratio < 1 or (coarse.n_rows * ratio != final.n_rows
                         or coarse.n_cols * ratio != final.n_cols):
            self.get_logger().error(
                f"clearance grid mismatch: coarse {coarse.n_rows}x"
                f"{coarse.n_cols}, final {final.n_rows}x{final.n_cols}, "
                f"ratio {ratio}; publishing an empty clearance layer")
            return blank
        ref_fine = np.repeat(np.repeat(fill_ref, ratio, axis=0), ratio, axis=1)
        return lowest - ref_fine

    def _publish_grid(self, elevation: np.ndarray, spec: GridSpec,
                      msg: PointCloud2, started: float,
                      n_points: int,
                      extra_layers: Optional[dict] = None) -> None:
        # The stamp lives on the header, not on the message itself:
        # PointCloud2 has no `stamp` attribute (`msg.stamp` raises
        # AttributeError, which escapes the callback and kills the node).
        gridmap = build_gridmap_msg(
            elevation, spec, self._output_frame_id, msg.header.stamp,
            extra_layers=extra_layers)
        self._pub.publish(gridmap)

        filled = int(np.count_nonzero(~np.isnan(elevation)))

        # Budget check. The upstream republishes /aft_pgo_map at 1 Hz and the
        # map only grows, so a long run is where this node would fall behind.
        # Measured on the Orin: 84k points -> 45 ms, 240k -> 170 ms,
        # 800k -> 670 ms, 2M -> 1.9 s. With QoS depth 1, falling behind
        # degrades to "the map refreshes less often" rather than lagging, but
        # it must not be silent -- a stale elevation map looks exactly like a
        # healthy one downstream.
        elapsed = time.monotonic() - started
        if elapsed > SLOW_FRAME_WARN_SEC:
            self.get_logger().warn(
                f"frame took {elapsed * 1e3:.0f} ms, longer than the upstream's "
                f"1 s publish period ({n_points} points); the elevation map "
                f"is now refreshing slower than 1 Hz",
                throttle_duration_sec=10.0)

        self.get_logger().debug(
            f"published GridMap from {n_points} points, "
            f"{filled} cells filled, {elapsed * 1e3:.1f} ms")


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = ProjectorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ExternalShutdownException is what rclpy raises when its SIGTERM
        # handler shuts the context down (e.g. `timeout 10 ros2 run ...`,
        # or a launch teardown). Without catching it the process would exit
        # with an unhandled-exception traceback on every normal stop.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
