#!/usr/bin/env python3
"""Long-run resource monitor for the point_lio + elevation stack.

Read-only: subscribes to nothing it publishes, sends no commands, kills no
processes. Safe to run in a second terminal while the robot is driving.

What it answers, in one table:

  * does /aft_pgo_map keep growing (the upstream PGO full-map rebuild), and
    how fast
  * does the filter chain keep up with it, or does it fall behind as the map
    grows
  * is simple_loop_closure's memory climbing (it is known to do ~3 GB/hour)
  * how much headroom is left on CPU / RAM / thermals after N minutes

Two costs are deliberately kept near zero so the monitor does not perturb
what it measures:

  * /proc is read directly for CPU and RSS -- no `ps`, `top` or extra
    subprocesses per sample
  * the point-cloud subscription only reads width/height; the 2.7 MB payload
    is discarded immediately by scalar message fields

Usage:

    # in a separate terminal, while the stack is running
    # (<repo> = 本仓库在容器内的路径, 例如 /my_ws/l4t/Lunar-Grotto-Walker)
    docker exec -it self_l4t bash -c \
      "source /my_ws/l4t/l4t_setup.sh && \
       python3 <repo>/tools/monitor_stack.py --duration 1500"

    # quieter, CSV only
    ... --duration 1500 --print-every 0 --csv /tmp/stack.csv

Press Ctrl-C at any time; the summary is printed on the way out.
"""
from __future__ import annotations

import argparse
import csv
import os
import signal
import sys
import time
from typing import Optional

import rclpy
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import PointCloud2

CLK_TCK = os.sysconf("SC_CLK_TCK")
PAGE_SIZE = os.sysconf("SC_PAGE_SIZE")

# Process names as they appear in /proc/<pid>/comm (15 chars, truncated).
# Order matters only for the column layout.
WATCHED = [
    ("slc", "simple_loop_clo"),
    ("lio", "pointlio_mappin"),
    ("filt", "filters_demo"),
    ("proj", "projector_node"),
    ("brdg", "bridge_node"),
    ("acc", "map_accumulator"),
    ("rviz", "rviz2"),
]

THERMAL_ZONES = ("tj-therm", "CPU-therm", "SOC0-therm", "SOC1-therm")


# --------------------------------------------------------------------------
# /proc readers (no subprocesses)
# --------------------------------------------------------------------------
def find_pids():
    """Map watched short-name -> pid (first match wins)."""
    found = {}
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        try:
            with open(f"/proc/{entry}/comm") as f:
                comm = f.read().strip()
        except (OSError, IOError):
            continue
        for key, want in WATCHED:
            if comm == want and key not in found:
                found[key] = int(entry)
    return found


def proc_cpu_seconds(pid: int) -> Optional[float]:
    """Total CPU time (user+nice+sys) in seconds, or None if gone."""
    try:
        with open(f"/proc/{pid}/stat") as f:
            data = f.read()
    except (OSError, IOError):
        return None
    # comm may contain spaces; everything after the last ')' is stable.
    fields = data[data.rindex(")") + 2:].split()
    # Fields after comm: state is [0], so utime=[11], stime=[12].
    return (int(fields[11]) + int(fields[12])) / CLK_TCK


def proc_rss_mb(pid: int) -> Optional[float]:
    try:
        with open(f"/proc/{pid}/statm") as f:
            return int(f.read().split()[1]) * PAGE_SIZE / (1024 * 1024)
    except (OSError, IOError, IndexError, ValueError):
        return None


def available_mb() -> float:
    try:
        with open("/proc/meminfo") as f:
            for line in f:
                if line.startswith("MemAvailable:"):
                    return int(line.split()[1]) / 1024.0
    except (OSError, IOError):
        pass
    return float("nan")


def temp_c() -> dict:
    out = {}
    for zone in os.listdir("/sys/class/thermal"):
        if not zone.startswith("thermal_zone"):
            continue
        try:
            with open(f"/sys/class/thermal/{zone}/type") as f:
                name = f.read().strip()
            if name not in THERMAL_ZONES:
                continue
            with open(f"/sys/class/thermal/{zone}/temp") as f:
                out[name] = int(f.read().strip()) / 1000.0
        except (OSError, IOError, ValueError):
            continue
    return out


# --------------------------------------------------------------------------
# ROS side: metadata only
# --------------------------------------------------------------------------
class TopicWatch(Node):
    """Counts messages and remembers the last message's shape."""

    def __init__(self):
        super().__init__("stack_monitor")
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            durability=QoSDurabilityPolicy.VOLATILE)
        self.t0 = time.time()
        self.n = {"upstream": 0, "raw": 0, "post": 0, "trav": 0}
        self.pts = 0
        self.raw_filled = float("nan")
        self.raw_cells = 0
        self.post_layers = 0

        self.create_subscription(PointCloud2, "/aft_pgo_map",
                                 self._on_cloud, qos)
        self.create_subscription(GridMap, "/elevation_map_raw",
                                 self._on_raw, qos)
        self.create_subscription(GridMap, "/elevation_map_raw_post",
                                 self._on_post, qos)
        self.create_subscription(OccupancyGrid, "/terrain_traversability",
                                 self._on_trav, qos)

    def _on_cloud(self, m):
        self.n["upstream"] += 1
        # Scalar fields only -- never touch m.data.
        self.pts = m.width * m.height

    def _on_raw(self, m):
        self.n["raw"] += 1
        n = int(round(m.info.length_x / m.info.resolution))
        self.raw_cells = n * n
        self.raw_len = m.info.length_x
        elev = None
        for i, nm in enumerate(m.layers):
            if nm == "elevation" and i < len(m.data):
                elev = m.data[i].data
                break
        if elev is not None:
            # Counting non-NaN in Python over 90k floats is ~5 ms; fine at
            # this cadence and cheaper than numpy import churn.
            filled = sum(1 for v in elev if v == v)
            self.raw_filled = filled

    def _on_post(self, m):
        self.n["post"] += 1
        self.post_layers = len(m.layers)

    def _on_trav(self, m):
        self.n["trav"] += 1

    def rates(self, elapsed):
        if elapsed <= 0:
            return {k: float("nan") for k in self.n}
        return {k: v / elapsed for k, v in self.n.items()}


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=1500.0,
                    help="Seconds to run (default 1500 = 25 min, covers a "
                         "20-minute run plus warm-up).")
    ap.add_argument("--interval", type=float, default=5.0,
                    help="Sampling period in seconds (default 5).")
    ap.add_argument("--print-every", type=float, default=60.0,
                    help="Print a table row every N seconds (0 = only CSV). "
                         "Default 60.")
    ap.add_argument("--csv", default="",
                    help="Write every sample to this CSV path.")
    args = ap.parse_args()

    rclpy.init()
    watch = TopicWatch()

    # CPU is measured by differencing /proc counters between samples.
    prev_cpu = {}
    prev_time = None
    prev_rss = {}
    samples = []

    started = time.time()
    last_print = 0.0
    csv_file = csv_writer = None
    if args.csv:
        csv_file = open(args.csv, "w", newline="")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(
            ["t_s", "slc_cpu", "slc_mb", "filt_cpu", "filt_mb",
             "lio_cpu", "proj_cpu", "rviz_cpu", "acc_cpu",
             "upstream_pts", "raw_filled", "raw_cells", "post_layers",
             "rate_up", "rate_raw", "rate_post", "rate_trav",
             "avail_mb", "tj_c", "cpu_c"])

    stop = {"v": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__("v", True))
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__("v", True))

    def header():
        print(f"\n{'t(s)':>6} {'slc%':>6} {'slcMB':>7} {'filt%':>6} "
              f"{'pts':>7} {'fill%':>6} {'up':>5} {'raw':>5} {'post':>5} "
              f"{'availMB':>8} {'tjC':>5}")
        print("-" * 84)

    header()

    try:
        while not stop["v"]:
            now = time.time()
            if now - started > args.duration:
                break
            rclpy.spin_once(watch, timeout_sec=min(args.interval, 0.2))

            elapsed = now - started
            if elapsed < args.interval:
                continue

            pids = find_pids()
            cpu_now, rss_now = {}, {}
            for key, _ in WATCHED:
                pid = pids.get(key)
                if pid is None:
                    continue
                c = proc_cpu_seconds(pid)
                if c is not None:
                    cpu_now[key] = c
                r = proc_rss_mb(pid)
                if r is not None:
                    rss_now[key] = r

            cpu_pct = {}
            if prev_time is not None:
                dt = now - prev_time
                if dt > 0:
                    for key, c in cpu_now.items():
                        if key in prev_cpu:
                            cpu_pct[key] = 100.0 * (c - prev_cpu[key]) / dt

            prev_cpu, prev_time, prev_rss = cpu_now, now, rss_now

            rates = watch.rates(elapsed)
            temps = temp_c()
            avail = available_mb()
            fill_pct = (100.0 * watch.raw_filled / watch.raw_cells
                        if watch.raw_cells else float("nan"))

            row = {
                "t_s": round(elapsed, 1),
                "slc_cpu": round(cpu_pct.get("slc", float("nan")), 1),
                "slc_mb": round(rss_now.get("slc", float("nan")), 1),
                "filt_cpu": round(cpu_pct.get("filt", float("nan")), 1),
                "filt_mb": round(rss_now.get("filt", float("nan")), 1),
                "lio_cpu": round(cpu_pct.get("lio", float("nan")), 1),
                "proj_cpu": round(cpu_pct.get("proj", float("nan")), 1),
                "rviz_cpu": round(cpu_pct.get("rviz", float("nan")), 1),
                "acc_cpu": round(cpu_pct.get("acc", float("nan")), 1),
                "upstream_pts": watch.pts,
                "raw_filled": watch.raw_filled,
                "raw_cells": watch.raw_cells,
                "post_layers": watch.post_layers,
                "rate_up": round(rates["upstream"], 2),
                "rate_raw": round(rates["raw"], 2),
                "rate_post": round(rates["post"], 2),
                "rate_trav": round(rates["trav"], 2),
                "avail_mb": round(avail, 0),
                "tj_c": round(temps.get("tj-therm", float("nan")), 1),
                "cpu_c": round(temps.get("CPU-therm", float("nan")), 1),
            }
            samples.append(row)
            if csv_writer:
                csv_writer.writerow([row[k] for k in row])

            if args.print_every > 0 and elapsed - last_print >= args.print_every:
                last_print = elapsed
                print(f"{row['t_s']:6.0f} {row['slc_cpu']:6.1f} "
                      f"{row['slc_mb']:7.0f} {row['filt_cpu']:6.1f} "
                      f"{row['upstream_pts']:7d} {fill_pct:6.1f} "
                      f"{row['rate_up']:5.2f} {row['rate_raw']:5.2f} "
                      f"{row['rate_post']:5.2f} "
                      f"{row['avail_mb']:8.0f} {row['tj_c']:5.1f}",
                      flush=True)
            # Keep spin_once from being called more often than the interval.
            while time.time() - now < args.interval and not stop["v"]:
                rclpy.spin_once(watch, timeout_sec=0.2)
    finally:
        watch.destroy_node()
        rclpy.shutdown()
        if csv_file:
            csv_file.close()

    # ---------------- summary ----------------
    if not samples:
        print("\n(no samples collected)")
        return 1

    def first_last(key):
        vals = [s[key] for s in samples
                if isinstance(s[key], (int, float)) and s[key] == s[key]]
        return (vals[0], vals[-1]) if vals else (float("nan"), float("nan"))

    print("\n" + "=" * 84)
    print("SUMMARY  ({} samples over {:.0f} s)".format(
        len(samples), samples[-1]["t_s"]))

    pts0, pts1 = first_last("upstream_pts")
    print(f"  /aft_pgo_map points : {pts0:,.0f} -> {pts1:,.0f}   "
          f"({pts1 / max(pts0, 1):.2f}x)")
    if samples[-1]["t_s"] > 60 and pts1 > pts0:
        per_min = (pts1 - pts0) / (samples[-1]["t_s"] / 60.0)
        print(f"    growth            : {per_min:,.0f} pts/min -> "
              f"{per_min * 20:,.0f} pts at 20 min (extrapolated)")

    m0, m1 = first_last("slc_mb")
    print(f"  simple_loop_closure : {m0:.0f} MB -> {m1:.0f} MB")
    if samples[-1]["t_s"] > 60:
        per_h = (m1 - m0) / (samples[-1]["t_s"] / 3600.0)
        print(f"    growth            : {per_h:+.0f} MB/hour -> "
              f"{m1 + per_h * 20 / 60:.0f} MB at 20 min (extrapolated)")

    for label, key in (("filters_demo", "filt_cpu"),
                       ("simple_loop_closure", "slc_cpu"),
                       ("point_lio", "lio_cpu"),
                       ("projector", "proj_cpu"),
                       ("rviz2", "rviz_cpu")):
        vals = [s[key] for s in samples
                if isinstance(s[key], (int, float)) and s[key] == s[key]]
        if vals:
            print(f"  {label:20s}: avg {sum(vals) / len(vals):5.1f}%  "
                  f"peak {max(vals):5.1f}%")

    for key, label in (("rate_up", "/aft_pgo_map"),
                       ("rate_raw", "/elevation_map_raw"),
                       ("rate_post", "/elevation_map_raw_post"),
                       ("rate_trav", "/terrain_traversability")):
        vals = [s[key] for s in samples
                if isinstance(s[key], (int, float)) and s[key] == s[key]]
        if vals:
            print(f"  {label:24s}: avg {sum(vals) / len(vals):.2f} Hz  "
                  f"min {min(vals):.2f}  max {max(vals):.2f}")

    a0, a1 = first_last("avail_mb")
    print(f"  MemAvailable        : {a0:.0f} MB -> {a1:.0f} MB")
    t_vals = [s["tj_c"] for s in samples if s["tj_c"] == s["tj_c"]]
    if t_vals:
        print(f"  tj-therm            : max {max(t_vals):.1f} C")

    # The one comparison that matters: did the filter chain fall behind only
    # as the map grew?
    first_third = samples[:max(1, len(samples) // 3)]
    last_third = samples[-max(1, len(samples) // 3):]

    def avg(rows, key):
        v = [r[key] for r in rows if isinstance(r[key], (int, float))
             and r[key] == r[key]]
        return sum(v) / len(v) if v else float("nan")

    print(f"\n  filter rate   early {avg(first_third, 'rate_post'):.2f} Hz"
          f"  ->  late {avg(last_third, 'rate_post'):.2f} Hz")
    print(f"  filter CPU    early {avg(first_third, 'filt_cpu'):.1f}%"
          f"  ->  late {avg(last_third, 'filt_cpu'):.1f}%")

    if args.csv:
        print(f"\n  CSV written to {args.csv}")
    print("=" * 84)
    return 0


if __name__ == "__main__":
    sys.exit(main())
