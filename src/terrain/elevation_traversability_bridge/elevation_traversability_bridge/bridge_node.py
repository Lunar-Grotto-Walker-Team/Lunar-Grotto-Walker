"""rclpy bridge node: elevation_mapping GridMap -> Nav2 traversability OccupancyGrid.

Subscribes /elevation_map_raw_post (grid_map_msgs/GridMap, BEST_EFFORT
depth=1) and publishes /terrain_traversability (nav_msgs/OccupancyGrid,
RELIABLE depth=1) so Nav2 costmap layers receive every published frame and
late-joining consumers can still grab the most recent one.

The input is elevation_mapping's *post-processed* map, because that is
where the terrain-analysis layers (slope / roughness) live -- the fused
map (/elevation_map) only carries elevation + confidence bounds.  See
`bridge_config` for the details.

Frame lock-step: every incoming GridMap must declare frame_id ==
`required_frame_id` (default "map"). On mismatch we emit a FATAL log and
exit with code 2 — silently emitting an OccupancyGrid with a wrong
frame_id would let a misconfigured TF tree corrupt the Nav2 costmap.

Lazy publish: on processing failure (missing layer, NaN geometry, etc.)
we do NOT publish an empty OccupancyGrid. Nav2 will keep using its
last-known costmap, which is safer than a one-frame blank.

The pure conversion logic lives in `bridge_processor.process_gridmap`.
This module is only the I/O shell.
"""
from __future__ import annotations

import math
import sys
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from grid_map_msgs.msg import GridMap as GridMapMsg
from nav_msgs.msg import OccupancyGrid, Odometry

from .bridge_classifier import ThresholdConfig
from .bridge_config import (
    DEFAULT_INPUT_TOPIC,
    DEFAULT_OUTPUT_TOPIC,
    DEFAULT_REQUIRED_FRAME_ID,
    DEFAULT_TARGET_RESOLUTION,
)
from .bridge_processor import process_gridmap


#: A frame slower than this is logged with its breakdown. At the 2 Hz target
#: the budget is 500 ms, so anything above half of it already means the next
#: INPUT will be replaced in the depth-1 queue (a silent dropped frame).
SLOW_FRAME_SEC = 0.25


class TerrainTraversabilityBridge(Node):
    """ROS2 node that republishes GridMap traversability as OccupancyGrid."""

    def __init__(self) -> None:
        super().__init__("terrain_traversability_bridge")

        # --- Parameters ---
        self.declare_parameter("input_topic", DEFAULT_INPUT_TOPIC)
        self.declare_parameter("output_topic", DEFAULT_OUTPUT_TOPIC)
        self.declare_parameter("required_frame_id", DEFAULT_REQUIRED_FRAME_ID)
        self.declare_parameter("target_resolution", DEFAULT_TARGET_RESOLUTION)

        # Threshold overrides — all optional, default to ThresholdConfig().
        self.declare_parameter("slope_soft_upper_deg", 30.0)
        self.declare_parameter("slope_lethal_upper_deg", 35.0)
        self.declare_parameter("roughness_soft_upper_m", 0.05)
        self.declare_parameter("roughness_lethal_upper_m", 0.15)
        # Small-cluster cleanup (see ThresholdConfig).
        self.declare_parameter("small_cluster_max_cells", 3)
        self.declare_parameter("small_cluster_min_height_m", 0.15)
        self.declare_parameter("small_cluster_min_slope_deg", 60.0)
        # Clearance layer; only used when the projector publishes one.
        self.declare_parameter("clearance_threshold_m", 1.0)
        self.declare_parameter("min_support_for_lethal", 8.0)
        self.declare_parameter("sparse_lethal_cost", 0)
        self.declare_parameter("height_soft_m", 0.0)
        self.declare_parameter("height_lethal_m", 0.30)
        self.declare_parameter("height_window_cells", 2)
        self.declare_parameter("height_min_neighbors", 2)
        self.declare_parameter("frontier_cells", 2)
        # Trail free-prior: the cells the robot has driven through cannot be
        # obstacles, so they override every lethal rule.  This is the defence
        # against upstream drift leaving two copies of a wall in
        # /aft_pgo_map: the corridor the robot just came down fills in, and
        # neither a live sensor nor the fill can undo it -- but the robot's
        # own trajectory can.
        self.declare_parameter("trail_clear_enabled", True)
        self.declare_parameter("trail_half_width_m", 0.25)
        self.declare_parameter("trail_window_s", 180.0)
        self.declare_parameter("trail_min_spacing_m", 0.05)
        self.declare_parameter("pose_topic", "/aft_pgo_odom")
        self.declare_parameter("pose_timeout_s", 2.0)

        # --- Cached config built once at startup ---
        self._cfg = ThresholdConfig(
            slope_soft_upper_deg=self.get_parameter("slope_soft_upper_deg").value,
            slope_lethal_upper_deg=self.get_parameter("slope_lethal_upper_deg").value,
            roughness_soft_upper_m=self.get_parameter("roughness_soft_upper_m").value,
            roughness_lethal_upper_m=self.get_parameter("roughness_lethal_upper_m").value,
            small_cluster_max_cells=int(
                self.get_parameter("small_cluster_max_cells").value),
            small_cluster_min_height_m=float(
                self.get_parameter("small_cluster_min_height_m").value),
            small_cluster_min_slope_deg=float(
                self.get_parameter("small_cluster_min_slope_deg").value),
            clearance_threshold_m=float(
                self.get_parameter("clearance_threshold_m").value),
            min_support_for_lethal=float(
                self.get_parameter("min_support_for_lethal").value),
            sparse_lethal_cost=int(
                self.get_parameter("sparse_lethal_cost").value),
            height_soft_m=float(
                self.get_parameter("height_soft_m").value),
            height_lethal_m=float(
                self.get_parameter("height_lethal_m").value),
            height_window_cells=int(
                self.get_parameter("height_window_cells").value),
            height_min_neighbors=int(
                self.get_parameter("height_min_neighbors").value),
            frontier_cells=int(self.get_parameter("frontier_cells").value),
        )
        self._target_res: float = float(self.get_parameter("target_resolution").value)
        self._required_frame_id: str = str(
            self.get_parameter("required_frame_id").value
        )

        # --- QoS profiles ---
        # Sub: RELIABLE, not BEST_EFFORT. The post layer is ~8 MB per frame;
        # with BEST_EFFORT the DDS layer silently drops some of them under
        # load (measured 2026-11-14: publisher at 2.0 Hz, this node saw
        # 1.55 Hz with 55 ms of processing and zero lag -- the loss was in
        # delivery, not in this node).
        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
        )
        # Pub: Nav2 costmap_2d subscribes RELIABLE; durability TRANSIENT_LOCAL
        # so late-joining consumers (e.g. RViz, Nav2 startup race) still get
        # the most recent published costmap.
        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        input_topic: str = str(self.get_parameter("input_topic").value)
        output_topic: str = str(self.get_parameter("output_topic").value)

        self._sub = self.create_subscription(
            GridMapMsg, input_topic, self._on_gridmap, sub_qos,
        )
        self._pub = self.create_publisher(
            OccupancyGrid, output_topic, pub_qos,
        )

        # --- trail free-prior state ---
        self._trail_on = bool(self.get_parameter("trail_clear_enabled").value)
        self._trail_half_width = float(
            self.get_parameter("trail_half_width_m").value)
        self._trail_window = float(self.get_parameter("trail_window_s").value)
        self._trail_spacing = float(
            self.get_parameter("trail_min_spacing_m").value)
        self._pose_timeout = float(self.get_parameter("pose_timeout_s").value)
        self._trail_t: list[float] = []
        self._trail_xy: list[tuple[float, float]] = []
        self._pose_last = None
        self._pose_last_rx = 0.0
        self._pose_sub = None
        if self._trail_on:
            self._pose_sub = self.create_subscription(
                Odometry, str(self.get_parameter("pose_topic").value),
                self._on_pose, 20)

        self._frames = 0
        self._sum_proc = 0.0
        self._max_proc = 0.0
        self._max_lag = 0.0
        self._last_report = time.monotonic()

        self.get_logger().info(
            f"terrain_traversability_bridge up: {input_topic} -> {output_topic}, "
            f"frame='{self._required_frame_id}', target_res={self._target_res} m, "
            f"thresholds=slope({self._cfg.slope_soft_upper_deg}/"
            f"{self._cfg.slope_lethal_upper_deg} deg) "
            f"rough({self._cfg.roughness_soft_upper_m}/"
            f"{self._cfg.roughness_lethal_upper_m} m) "
            f"height({self._cfg.height_soft_m}/"
            f"{self._cfg.height_lethal_m} m, "
            f"{self._cfg.height_window_cells} cells)"
        )

    def _note_frame(self, t0: float, t_proc: float, t_pub: float,
                    msg: GridMapMsg) -> None:
        """Per-frame timing, so a dropped frame is attributable.

        The chain is measured end to end elsewhere; this node was the only
        silent one. A slow process_gridmap keeps the input queued (depth 1),
        so frames disappear without any log line -- the exact failure the
        2 Hz target cannot afford.
        """
        now = time.monotonic()
        total = now - t0
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        lag = now - stamp if stamp > 0 else float('nan')
        self._frames += 1
        self._sum_proc += t_proc - t0
        self._max_proc = max(self._max_proc, t_proc - t0)
        self._max_lag = max(self._max_lag, lag)
        if total > SLOW_FRAME_SEC:
            self.get_logger().warn(
                f"frame took {total * 1e3:.0f} ms "
                f"(process {1e3 * (t_proc - t0):.0f}, "
                f"publish {1e3 * (t_pub - t_proc):.0f}), "
                f"lag {lag:.2f} s, {msg.info.length_x}x{msg.info.length_y} m")
        if now - self._last_report >= 30.0:
            self.get_logger().info(
                f"{self._frames} frames in {now - self._last_report:.0f} s "
                f"({self._frames / (now - self._last_report):.2f} Hz), "
                f"process avg {1e3 * self._sum_proc / max(self._frames, 1):.0f}"
                f" ms, max {1e3 * self._max_proc:.0f} ms, "
                f"max lag {self._max_lag:.2f} s")
            self._frames = 0
            self._sum_proc = 0.0
            self._max_proc = 0.0
            self._max_lag = 0.0
            self._last_report = now

    def _on_pose(self, msg: Odometry) -> None:
        """Append the robot's map-frame position to the rolling trail."""
        p = msg.pose.pose.position
        now_s = time.monotonic()
        if (self._trail_xy
                and math.hypot(p.x - self._trail_xy[-1][0],
                               p.y - self._trail_xy[-1][1])
                < self._trail_spacing):
            return
        self._trail_t.append(now_s)
        self._trail_xy.append((float(p.x), float(p.y)))
        self._pose_last_rx = now_s
        cutoff = now_s - self._trail_window
        if self._trail_t and self._trail_t[0] < cutoff:
            # Drop from the front; the deque is append-only and ordered.
            i = 0
            while i < len(self._trail_t) and self._trail_t[i] < cutoff:
                i += 1
            del self._trail_t[:i]
            del self._trail_xy[:i]

    def _trail_for_frame(self):
        """Trail points if the pose is fresh, else None (no clearing)."""
        if not self._trail_on or not self._trail_xy:
            return None
        if time.monotonic() - self._pose_last_rx > self._pose_timeout:
            return None
        return self._trail_xy

    def _on_gridmap(self, msg: GridMapMsg) -> None:
        # --- Frame validation ---
        # GridMapInfo has no header field; frame_id lives on msg.header.
        # Trust msg.header.frame_id (set by elevation_mapping's publisher).
        fid: Optional[str] = msg.header.frame_id or None
        if fid != self._required_frame_id:
            self.get_logger().fatal(
                f"GridMap frame_id mismatch: got '{fid}', expected "
                f"'{self._required_frame_id}'. Refusing to bridge into a "
                f"misconfigured TF tree — exiting."
            )
            # Raise SystemExit(2) so the launch system / rclpy shutdown
            # path reports a non-zero exit code and restarts do not loop
            # silently. Use rclpy.shutdown first to clean up DDS.
            try:
                rclpy.shutdown()
            except Exception:  # noqa: BLE001 — best-effort cleanup
                pass
            # sys.exit raises SystemExit; rclpy init / spin thread will
            # see it on the next iteration.
            sys.exit(2)

        # --- Pure conversion ---
        t0 = time.monotonic()
        trail_stats: dict = {}
        try:
            out: OccupancyGrid = process_gridmap(
                msg, cfg=self._cfg, target_resolution=self._target_res,
                trail_xy=self._trail_for_frame(),
                trail_half_width_m=self._trail_half_width,
                stats=trail_stats,
                robot_xy=self._trail_xy[-1] if self._trail_xy else None,
            )
        except ValueError as exc:
            # Layer missing, shape mismatch, non-integer upsample, etc.
            # Lazy publish: log and drop this frame, do NOT publish an
            # empty OccupancyGrid (Nav2 would briefly treat all cells
            # as UNKNOWN/LETHAL and plan around nothing).
            self.get_logger().error(
                f"process_gridmap failed for frame_id={fid}: {exc}. "
                f"Dropping frame, NOT publishing empty OccupancyGrid."
            )
            return
        except Exception as exc:  # noqa: BLE001 — last-resort guard
            self.get_logger().error(
                f"unexpected error in process_gridmap: {exc!r}. "
                f"Dropping frame."
            )
            return
        t_proc = time.monotonic()

        self._pub.publish(out)
        if trail_stats:
            self.get_logger().info(
                f"trail free-prior: {trail_stats.get('trail_cells', 0)} cells "
                f"within {self._trail_half_width:.2f} m of the path, "
                f"{trail_stats.get('trail_changed', 0)} forced free "
                f"({trail_stats.get('trail_changed_far', 0)} of them >2 m away)",
                throttle_duration_sec=10.0)
        self.get_logger().debug(
            f"published OccupancyGrid: {out.info.width}x{out.info.height} "
            f"@ {out.info.resolution:.3f} m, frame='{out.header.frame_id}'"
        )
        self._note_frame(t0, t_proc, time.monotonic(), msg)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = TerrainTraversabilityBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()