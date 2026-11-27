"""Relay the terrain traversability grid into a frontier-friendly map.

``/terrain_traversability`` carries Nav2 Costmap2D-internal bytes
(0 = FREE, 85 = UNKNOWN, 128 = SOFT, 254 = LETHAL).  ``frontier_exploration_ros2``
reads a plain occupancy grid: negative means unknown, ``>= occ_threshold``
means occupied.  Feeding the raw bytes through is wrong in both directions:

* 85 is **positive**, so every never-observed cell would look like an
  obstacle;
* 128 / 254 are stored in an ``int8`` field, so they arrive as -128 / -2 and
  every slope or wall would look like unknown space.

2026-11-19: **128 (SOFT) maps to 50, not 100.** The bridge classifies a
15-20 cm height jump or a 0.12-0.35 m rough patch as SOFT because the Go2
can still drive it; turning that into OCCUPIED cut the corridor in two for
the frontier explorer. 50 is passable-but-costly: the decision map treats
``cost < occ_threshold`` as free, while RViz/debug can still tell soft
terrain apart from flat ground.

This node re-maps the bytes and optionally pools them to a coarser
resolution (the frontier decision map is O(cells) and a 100 m map at 0.1 m
is a million cells).
"""
from __future__ import annotations

import array as array_mod
from typing import Optional

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

FREE = 0
UNKNOWN = -1
# 2026-11-19: SOFT 地形(128) 是 Go2 能过的 15-20 cm 高度突变/粗糙带, 不能再
# 当墙。50 是"可通行但高代价"的独立值: frontier 决策图把 cost < occ_threshold
# (65) 判为 free, 所以 50 对搜索等价于可通行, 同时 RViz/调试仍能区分软地形。
SOFT = 50
OCCUPIED = 100

_BYTE_FREE = 0
_BYTE_UNKNOWN = 85
_BYTE_SOFT = 128
_BYTE_LETHAL = 254


def _byte_lut() -> np.ndarray:
    """Map every raw byte to our output value (unknown unless stated)."""
    lut = np.full(256, OCCUPIED, dtype=np.int8)
    lut[_BYTE_FREE] = FREE
    lut[_BYTE_UNKNOWN] = UNKNOWN
    lut[_BYTE_SOFT] = SOFT
    lut[_BYTE_LETHAL] = OCCUPIED
    return lut


def convert_and_pool(data, width: int, height: int, pool: int
                     ) -> tuple[np.ndarray, int, int]:
    """Convert the cost bytes and pool ``pool x pool`` blocks.

    Returns ``(flat, out_width, out_height)`` where ``flat`` is row-major,
    exactly the layout ``OccupancyGrid.data`` uses.  The block reduction is
    "any obstacle -> obstacle, else any soft -> soft, else any free -> free,
    else unknown": soft wins over free (conservative -- a block that mixes a
    passable rough patch with flat ground keeps the cost hint) and free wins
    over unknown so the unknown ring around the robot is not turned into a
    frontier, at the cost of shrinking frontiers by < one coarse cell.

    The leftover row/column when the size is not a multiple of ``pool`` is
    cropped at the FAR edge, so the origin (the low corner) never moves.
    """
    pool = max(1, int(pool))
    # rclpy hands us an int8 array; 128/254 must be reinterpreted, not cast.
    raw = np.asarray(data, dtype=np.int8).view(np.uint8).reshape(height, width)
    costs = _byte_lut()[raw]

    if pool == 1:
        return costs.reshape(-1), width, height

    out_w = width // pool
    out_h = height // pool
    if out_w == 0 or out_h == 0:
        return np.zeros((0,), dtype=np.int8), 0, 0

    blocks = costs[:out_h * pool, :out_w * pool].reshape(
        out_h, pool, out_w, pool)
    out = np.full((out_h, out_w), UNKNOWN, dtype=np.int8)
    out[np.any(blocks == FREE, axis=(1, 3))] = FREE
    out[np.any(blocks == SOFT, axis=(1, 3))] = SOFT
    out[np.any(blocks == OCCUPIED, axis=(1, 3))] = OCCUPIED
    return out.reshape(-1), out_w, out_h


class PersistentMap:
    """Accumulate windowed exploration grids into one growing full map.

    ``/terrain_traversability`` became a robot-centred 70 m window when the
    terrain chain moved to a fixed-size grid, but /exploration_map is what
    the frontier explorer reasons about: it must keep everything that has
    ever been observed, otherwise unexplored pockets the robot has walked
    past simply vanish from the map and exploration stops early.

    Cells are merged in world coordinates at a fixed resolution.  UNKNOWN
    never overwrites a known cell and the origin snaps to the resolution
    lattice, so repeated merges are idempotent.
    """

    def __init__(self, resolution: float, max_cells: int) -> None:
        self.resolution = float(resolution)
        self.max_cells = int(max_cells)
        self._origin_x = 0.0
        self._origin_y = 0.0
        self._data: Optional[np.ndarray] = None    # (rows, cols) int8

    def merge(self, flat, width: int, height: int,
              origin_x: float, origin_y: float) -> bool:
        """Merge one row-major grid; False when the size guard trips."""
        incoming = np.asarray(flat, dtype=np.int8).reshape(height, width)
        res = self.resolution
        # Snap the incoming corner to the lattice so merges stay idempotent.
        ix = int(round(origin_x / res))
        iy = int(round(origin_y / res))

        if self._data is None:
            self._origin_x = ix * res
            self._origin_y = iy * res
            self._data = incoming.copy()
            return self._check_size()

        cur_x = int(round(self._origin_x / res))
        cur_y = int(round(self._origin_y / res))
        cur_rows, cur_cols = self._data.shape

        new_x = min(cur_x, ix)
        new_y = min(cur_y, iy)
        new_cols = max(cur_x + cur_cols, ix + width) - new_x
        new_rows = max(cur_y + cur_rows, iy + height) - new_y
        if new_cols * new_rows > self.max_cells:
            return False

        grown = np.full((new_rows, new_cols), UNKNOWN, dtype=np.int8)
        grown[cur_y - new_y:cur_y - new_y + cur_rows,
              cur_x - new_x:cur_x - new_x + cur_cols] = self._data
        # Known cells win; the incoming UNKNOWN leaves the existing value.
        off_r = iy - new_y
        off_c = ix - new_x
        window = grown[off_r:off_r + height, off_c:off_c + width]
        known = incoming != UNKNOWN
        window[known] = incoming[known]

        self._data = grown
        self._origin_x = new_x * res
        self._origin_y = new_y * res
        return True

    def _check_size(self) -> bool:
        return self._data.size <= self.max_cells

    def to_msg(self, header) -> Optional[OccupancyGrid]:
        if self._data is None:
            return None
        rows, cols = self._data.shape
        out = OccupancyGrid()
        if header is not None:
            out.header = header
        out.info.resolution = self.resolution
        out.info.width = cols
        out.info.height = rows
        out.info.origin.position.x = self._origin_x
        out.info.origin.position.y = self._origin_y
        out.info.origin.orientation.w = 1.0
        out._data = array_mod.array("b", self._data.reshape(-1).tobytes())
        return out


class ExplorationMapRelay(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("exploration_map_relay", **kwargs)

        self.declare_parameter("input_topic", "/terrain_traversability")
        self.declare_parameter("output_topic", "/exploration_map")
        self.declare_parameter("output_resolution", 0.2)
        # 2026-11-14: the terrain grid became a 70 m robot-centred window, so
        # this relay accumulates the windows into one growing map. Without it
        # a pocket the robot has walked past leaves /exploration_map and the
        # frontier search concludes the map is complete.
        self.declare_parameter("accumulate", True)
        self.declare_parameter("max_cells", 4000000)

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._output_resolution = float(
            self.get_parameter("output_resolution").value)
        self._accumulate = bool(self.get_parameter("accumulate").value)
        self._max_cells = int(self.get_parameter("max_cells").value)
        self._map = PersistentMap(self._output_resolution, self._max_cells)
        self._size_warned = False

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._sub = self.create_subscription(
            OccupancyGrid, self._input_topic, self._on_msg, qos)
        self._pub = self.create_publisher(
            OccupancyGrid, self._output_topic, qos)

        self.get_logger().info(
            f"exploration_map_relay up: {self._input_topic} -> "
            f"{self._output_topic}, target_resolution="
            f"{self._output_resolution} m")

    def _process(self, msg: OccupancyGrid) -> Optional[OccupancyGrid]:
        width = int(msg.info.width)
        height = int(msg.info.height)
        if width <= 0 or height <= 0:
            return None
        if len(msg.data) != width * height:
            self.get_logger().error(
                f"grid is {width}x{height} but carries {len(msg.data)} cells")
            return None

        resolution = float(msg.info.resolution)
        if resolution <= 0.0:
            return None
        pool = max(1, int(round(self._output_resolution / resolution)))

        flat, out_w, out_h = convert_and_pool(msg.data, width, height, pool)
        if out_w == 0 or out_h == 0:
            return None

        if self._accumulate:
            ok = self._map.merge(
                flat, out_w, out_h,
                float(msg.info.origin.position.x),
                float(msg.info.origin.position.y))
            if not ok:
                if not self._size_warned:
                    self._size_warned = True
                    self.get_logger().error(
                        f"accumulated exploration map would exceed "
                        f"max_cells={self._max_cells}; publishing the "
                        "windowed grid instead")
                return self._windowed(msg, flat, out_w, out_h, resolution, pool)
            return self._map.to_msg(msg.header)

        return self._windowed(msg, flat, out_w, out_h, resolution, pool)

    @staticmethod
    def _windowed(msg, flat, out_w, out_h, resolution, pool) -> OccupancyGrid:
        out = OccupancyGrid()
        out.header = msg.header
        out.info.resolution = resolution * pool
        out.info.width = out_w
        out.info.height = out_h
        out.info.origin = msg.info.origin
        out.data = array_mod.array("b", np.asarray(flat, np.int8).tobytes())
        return out

    def _on_msg(self, msg: OccupancyGrid) -> None:
        out = self._process(msg)
        if out is not None:
            self._pub.publish(out)


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = ExplorationMapRelay()
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
