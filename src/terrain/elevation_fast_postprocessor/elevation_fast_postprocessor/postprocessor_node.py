"""ROS node wrapper around :mod:`postprocessor_core`.

Subscribes to the projector's single-layer ``/elevation_map_raw`` and
republishes ``/elevation_map_raw_post`` with the derived layers the
traversability bridge consumes.  The topic names, frame and grid geometry
are passed through unchanged, so the bridge needs no modification.
"""
from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Optional

import numpy as np
import rclpy
from grid_map_msgs.msg import GridMap
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from .gridmap_out import build_multilayer_gridmap
from .postprocessor_core import compute_layers

#: Frames slower than this are logged: the upstream projector refreshes at
#: 1 Hz, so the whole chain is meant to keep up with 1 s.  Message
#: construction in rclpy is the dominant cost at large grids -- see
#: user-log.md §9.9 for the measured numbers.
SLOW_FRAME_WARN_SEC = 1.0


@dataclass(frozen=True)
class _Spec:
    resolution: float
    length_x: float
    length_y: float
    origin_x: float
    origin_y: float

    @property
    def n_rows(self) -> int:
        return int(round(self.length_x / self.resolution))

    @property
    def n_cols(self) -> int:
        return int(round(self.length_y / self.resolution))


class PostprocessorNode(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("elevation_fast_postprocessor", **kwargs)

        self.declare_parameter("input_topic", "/elevation_map_raw")
        self.declare_parameter("output_topic", "/elevation_map_raw_post")
        self.declare_parameter("mean_radius_m", 0.15)
        self.declare_parameter("publish_debug_layers", False)
        # 1 = slope from the raw (inpainted) surface; 3 = 3x3 box mean first.
        self.declare_parameter("slope_smooth_cells", 1)
        # >0 fits a plane over (2k+1)^2 cells instead of a two-cell gradient.
        self.declare_parameter("slope_fit_cells", 1)
        # Holes wider than this many cells stay unknown instead of being
        # filled by the nearest observed cell. 0 = fill everything (old).
        self.declare_parameter("fill_max_cells", 3)

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._mean_radius_m = float(self.get_parameter("mean_radius_m").value)
        self._publish_debug = bool(
            self.get_parameter("publish_debug_layers").value)
        self._slope_smooth = int(
            self.get_parameter("slope_smooth_cells").value)
        self._slope_fit = int(self.get_parameter("slope_fit_cells").value)
        self._fill_max_cells = int(self.get_parameter("fill_max_cells").value)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._sub = self.create_subscription(
            GridMap, self._input_topic, self._on_msg, qos)
        self._pub = self.create_publisher(GridMap, self._output_topic, qos)

        self.get_logger().info(
            f"elevation_fast_postprocessor up: {self._input_topic} -> "
            f"{self._output_topic}, mean_radius={self._mean_radius_m} m, "
            f"debug_layers={self._publish_debug}")

    def _extract_elevation(self, msg: GridMap, spec: _Spec) -> Optional[np.ndarray]:
        if "elevation" not in msg.layers:
            self.get_logger().error(
                f"input GridMap has no 'elevation' layer (got {list(msg.layers)})")
            return None
        index = msg.layers.index("elevation")
        flat = np.asarray(msg.data[index].data, dtype=np.float64)
        return flat.reshape((spec.n_rows, spec.n_cols), order="F")

    def _process(self, msg: GridMap) -> Optional[GridMap]:
        started = time.monotonic()
        spec = _Spec(
            resolution=float(msg.info.resolution),
            length_x=float(msg.info.length_x),
            length_y=float(msg.info.length_y),
            origin_x=float(msg.info.pose.position.x),
            origin_y=float(msg.info.pose.position.y),
        )
        if spec.n_rows <= 0 or spec.n_cols <= 0:
            return None

        elevation = self._extract_elevation(msg, spec)
        if elevation is None:
            return None
        if np.isnan(elevation).all():
            # Same rule as the projector: do not publish an all-unknown map.
            return None

        layers = compute_layers(
            elevation,
            resolution=spec.resolution,
            mean_radius_m=self._mean_radius_m,
            publish_debug=self._publish_debug,
            slope_smooth_cells=self._slope_smooth,
            slope_fit_cells=self._slope_fit,
            fill_max_cells=self._fill_max_cells,
        )
        # Pass through layers this node does not compute (e.g. the projector's
        # optional "clearance"). Dropping them here would silently break the
        # bridge's clearance check, which reads the layer by name further down
        # the chain -- the map would look perfectly traversable.
        for index, name in enumerate(msg.layers):
            if name in layers:
                continue
            flat = np.asarray(msg.data[index].data, dtype=np.float64)
            if flat.size != spec.n_rows * spec.n_cols:
                self.get_logger().error(
                    f"layer '{name}' has {flat.size} values, expected "
                    f"{spec.n_rows * spec.n_cols}; dropping it",
                    throttle_duration_sec=30.0)
                continue
            layers[name] = flat.reshape((spec.n_rows, spec.n_cols), order="F")
        out = build_multilayer_gridmap(
            layers, spec, msg.header.frame_id, msg.header.stamp)

        elapsed = time.monotonic() - started
        if elapsed > SLOW_FRAME_WARN_SEC:
            self.get_logger().warn(
                f"frame took {elapsed:.0f} ms on a {spec.n_rows}x{spec.n_cols} "
                f"grid; the chain is refreshing slower than the upstream 1 Hz",
                throttle_duration_sec=10.0)
        return out

    def _on_msg(self, msg: GridMap) -> None:
        out = self._process(msg)
        if out is not None:
            self._pub.publish(out)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = PostprocessorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
