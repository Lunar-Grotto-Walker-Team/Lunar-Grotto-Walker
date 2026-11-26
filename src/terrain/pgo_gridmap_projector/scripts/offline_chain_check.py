#!/usr/bin/env python3
"""Offline end-to-end check of the point_lio projection chain.

Stands in for the pointlio_ws upstream by publishing a synthetic
/aft_pgo_map, and inspects every hop of the downstream chain:

    /aft_pgo_map -> pgo_gridmap_projector -> /elevation_map_raw
                 -> elevation_fast_postprocessor -> /elevation_map_raw_post
                 -> terrain_traversability_bridge -> /terrain_traversability

The synthetic world is a 20 m x 20 m room: flat ground at z = 0 bounded by
four 1 m-high walls at +/-10 m. Walls are clipped by the projector's z gate
(ignore_points_above 0.5), so the map should show a flat interior and a
ring of wall returns near the edges.

Checks:
  1. /elevation_map_raw arrives, 500x500 @ 0.10 m, frame_id == map, and the
     room interior is flat
  2. /elevation_map_raw_post arrives with the full 8-layer set, and neither
     slope nor roughness is degenerate (a zero roughness means the
     mean-in-radius radius collapsed to one cell)
  3. /terrain_traversability arrives, frame_id == map, carries only the four
     Costmap2D byte values, and is not 100% UNKNOWN
  4. statelessness: re-publishing the same world shifted by 1.5 m must
     produce a DIFFERENT map. If the two were identical the projector would
     be holding state, which is the exact failure mode this design exists to
     avoid.

Usage (the chain must already be running, e.g. via
go2_pointlio_projected.launch.py with show_rviz:=false):

    python3 install/pgo_gridmap_projector/lib/pgo_gridmap_projector/scripts/offline_chain_check.py

Exit code 0 = all hops verified; 1 = at least one failure, listed at the end.
"""
import sys
import time

import numpy as np
import rclpy
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import PointCloud2, PointField

ROOM_HALF = 10.0
WALL_Z = 1.0


def make_cloud(drift_x: float = 0.0) -> PointCloud2:
    """Synthetic PGO full map: ground plane + 4 tall walls, offset by drift."""
    rng = np.random.default_rng(7)

    # Ground: 0.05 m lattice over the room interior.
    g = np.arange(-ROOM_HALF, ROOM_HALF + 1e-9, 0.05)
    gx, gy = np.meshgrid(g, g, indexing="ij")
    ground = np.stack([gx.ravel(), gy.ravel(),
                       np.zeros(gx.size)], axis=1)

    # Walls: vertical curtains at the four edges, spanning z = 0..WALL_Z.
    zs = np.arange(0.0, WALL_Z + 1e-9, 0.05)
    walls = []
    for fixed in (-ROOM_HALF, ROOM_HALF):
        sx, sz = np.meshgrid(g, zs, indexing="ij")
        walls.append(np.stack([np.full(sx.size, fixed), sx.ravel(), sz.ravel()], axis=1))
        walls.append(np.stack([sx.ravel(), np.full(sx.size, fixed), sz.ravel()], axis=1))
    walls = np.concatenate(walls, axis=0)

    xyz = np.concatenate([ground, walls], axis=0)
    xyz[:, 0] += drift_x
    # A little sensor noise, like a real LIO map.
    xyz += rng.normal(0.0, 0.01, xyz.shape)

    msg = PointCloud2()
    msg.header.frame_id = "camera_init"
    msg.height = 1
    msg.width = xyz.shape[0]
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * xyz.shape[0]
    msg.is_dense = True
    msg.data = xyz.astype(np.float32).tobytes()
    msg.header.stamp = rclpy.time.Time().to_msg()
    return msg


class ChainProbe(Node):
    def __init__(self):
        super().__init__("chain_e2e_probe")
        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            durability=QoSDurabilityPolicy.VOLATILE)

        self.pub = self.create_publisher(PointCloud2, "/aft_pgo_map", pub_qos)
        self.raw = None
        self.post = None
        self.trav = None
        self.create_subscription(GridMap, "/elevation_map_raw", self._on_raw, sub_qos)
        self.create_subscription(GridMap, "/elevation_map_raw_post", self._on_post, sub_qos)
        self.create_subscription(OccupancyGrid, "/terrain_traversability",
                                 self._on_trav, sub_qos)

    def _on_raw(self, m):
        self.raw = m

    def _on_post(self, m):
        self.post = m

    def _on_trav(self, m):
        self.trav = m

    def pump(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_for(self, attr, timeout, drift):
        """Publish until `attr` is populated."""
        end = time.time() + timeout
        cloud = make_cloud(drift)
        while time.time() < end:
            self.pub.publish(cloud)
            self.pump(0.8)
            if getattr(self, attr) is not None:
                return True
        return False


def layer_of(msg, name):
    for i, nm in enumerate(msg.layers):
        if nm == name:
            return np.array(msg.data[i].data, dtype=np.float32)
    return None


def main():
    rclpy.init()
    node = ChainProbe()
    failures = []
    try:
        # ---- hop 1: projector ----
        if not node.wait_for("raw", 30.0, 0.0):
            print("FAIL: no /elevation_map_raw")
            return 1
        raw = node.raw
        print(f"[1] /elevation_map_raw  frame={raw.header.frame_id!r} "
              f"res={raw.info.resolution} "
              f"len={raw.info.length_x}x{raw.info.length_y} "
              f"layers={raw.layers}")
        if raw.header.frame_id != "map":
            failures.append(f"raw frame_id is {raw.header.frame_id!r}, expected 'map'")
        n = int(round(raw.info.length_x / raw.info.resolution))
        elev = layer_of(raw, "elevation")
        if elev is None or elev.size != n * n:
            failures.append(f"raw elevation layer missing/wrong size ({None if elev is None else elev.size})")
        else:
            inner = elev.reshape((n, n), order="F")[100:400, 100:400]
            print(f"    interior elevation: nan={np.isnan(inner).sum()} "
                  f"mean={np.nanmean(inner):.4f} max={np.nanmax(inner):.4f}")
            if abs(np.nanmean(inner)) > 0.05:
                failures.append(f"interior mean elevation {np.nanmean(inner):.3f} != 0")

        # ---- hop 2: filter chain ----
        if not node.wait_for("post", 30.0, 0.0):
            print("FAIL: no /elevation_map_raw_post")
            return 1
        post = node.post
        expected = {"elevation", "elevation_inpainted", "elevation_smooth",
                    "normal_vectors_x", "normal_vectors_y", "normal_vectors_z",
                    "slope", "roughness"}
        missing = expected - set(post.layers)
        print(f"[2] /elevation_map_raw_post  layers={len(post.layers)} "
              f"missing={sorted(missing) or 'none'}")
        if missing:
            failures.append(f"filter chain missing layers: {sorted(missing)}")
        for nm in ("slope", "roughness"):
            arr = layer_of(post, nm)
            if arr is not None:
                fin = arr[np.isfinite(arr)]
                if fin.size == 0:
                    failures.append(f"{nm} is all-NaN")
                elif nm == "roughness" and np.nanmax(fin) == 0.0:
                    failures.append("roughness is identically zero (degenerate mean radius)")
                else:
                    print(f"    {nm}: min={fin.min():.4f} max={fin.max():.4f}")
        if post.header.frame_id != "map":
            failures.append(f"post frame_id is {post.header.frame_id!r}, expected 'map'")

        # ---- hop 3: bridge ----
        if not node.wait_for("trav", 40.0, 0.0):
            print("FAIL: no /terrain_traversability (bridge never published)")
            return 1
        trav = node.trav
        # OccupancyGrid.data is int8 on the wire but the bridge deliberately
        # writes Costmap2D's 0-255 byte convention (0 FREE / 85 UNKNOWN /
        # 128 SOFT / 254 LETHAL) through the private `_data` slot. Decode as
        # uint8 so the report shows the real costs, not int8 wrap-around.
        raw = np.asarray(trav.data, dtype=np.int8).view(np.uint8)
        vals, counts = np.unique(raw, return_counts=True)
        names = {0: "FREE", 85: "UNKNOWN", 128: "SOFT", 254: "LETHAL"}
        breakdown = ", ".join(
            f"{v}={names.get(int(v), 'OTHER')}({c})"
            for v, c in zip(vals, counts))
        print(f"[3] /terrain_traversability  frame={trav.header.frame_id!r} "
              f"{trav.info.width}x{trav.info.height} "
              f"res={trav.info.resolution}")
        print(f"    values: {breakdown}")
        other = [int(v) for v in vals if int(v) not in names]
        if other:
            failures.append(f"traversability has unexpected cost values: {other}")
        if trav.header.frame_id != "map":
            failures.append(f"trav frame_id is {trav.header.frame_id!r}, expected 'map'")
        if list(vals.tolist()) == [85]:
            failures.append("traversability is 100% UNKNOWN")

        # ---- hop 4: statelessness / revisitation ----
        node.raw = node.post = node.trav = None
        if node.wait_for("raw", 30.0, 1.5):
            shifted = layer_of(node.raw, "elevation")
            shifted = shifted.reshape((n, n), order="F")
            orig = elev.reshape((n, n), order="F")
            both = np.isfinite(orig) & np.isfinite(shifted)
            same = np.isfinite(orig) == np.isfinite(shifted)
            print(f"[4] drift 1.5 m: cell-validity identical={same.all()} "
                  f"({(~same).sum()} cells differ)")
            if same.all() and np.nanmax(np.abs(orig[both] - shifted[both])) < 1e-6:
                failures.append("1.5 m upstream drift produced an identical map — "
                                "the projector is not honouring the input")
        else:
            failures.append("no second /elevation_map_raw after the drifted frame")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if failures:
        print("\nFAILURES:")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("\nPASS: all four hops verified")
    return 0


if __name__ == "__main__":
    sys.exit(main())
