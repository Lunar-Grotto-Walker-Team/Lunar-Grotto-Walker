"""Pure-numpy core for the uLidar near-field ground fill.

The fill node rasterizes the dense near-field uLidar ground into a small
odom-frame grid, estimates a floor height per cell, rejects cells that are
dominated by a wall or a leg (large vertical spread), and keeps only points
close to the local floor.  The projector later uses these points only where
Livox has no points, so Livox walls and slopes are not overwritten.
"""
from __future__ import annotations

import threading
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class GroundConfig:
    grid_size: float = 12.0
    cell: float = 0.25
    percentile: float = 20.0
    min_points: int = 3
    spread_max: float = 0.20
    fill_iterations: int = 4
    h_min: float = -0.10
    h_max: float = 0.15
    max_range: float = 3.0
    voxel_leaf: float = 0.10
    window_s: float = 5.0
    z_offset: float = 0.0
    jump_trans: float = 0.5
    jump_rot: float = 0.3
    # 2026-11-19: publish the fill as a dense ground *surface* (one point per
    # dense_spacing cell inside the trusted/neighbour-filled area) instead of
    # the raw L2 returns.  The returns give ~1 point per 0.1 m cell, so only
    # ~half of the near-field cells had any fill point and the rest stayed
    # UNKNOWN in the elevation grid (measured: 103-201 of ~200 cells inside
    # 0.8 m).  dense_spacing <= 0 means "use voxel_leaf".
    dense_output: bool = True
    dense_spacing: float = 0.0


@dataclass
class GroundGrid:
    z: np.ndarray
    trusted: np.ndarray
    origin: np.ndarray
    cell: float
    n: int
    #: Cells that had enough points but a wide z spread (wall / leg).  They
    #: are NaN before :func:`fill_ground` and borrow a neighbour floor after
    #: it; the dense surface must skip them instead of flooring over the
    #: obstacle.
    rejected: np.ndarray | None = None


def estimate_ground_grid(xyz: np.ndarray, origin: np.ndarray, cell: float,
                         n: int, percentile: float = 20.0,
                         min_points: int = 3,
                         spread_max: float = 0.20) -> GroundGrid:
    """Per-cell low percentile of z, trusted only for compact cells.

    ``trusted`` is False for a cell with fewer than ``min_points`` points or
    with ``p80 - p20 > spread_max``.  The latter rejects vertical walls and
    legs, whose cell contains a wide z range; a 30 deg slope in a 0.25 m cell
    has p80-p20 ~= 0.14 m and stays trusted.
    """
    if cell <= 0.0 or n <= 0:
        raise ValueError("cell and n must be positive")
    z_out = np.full((n, n), np.nan, dtype=np.float64)
    trusted = np.zeros((n, n), dtype=bool)
    rejected = np.zeros((n, n), dtype=bool)
    if xyz.shape[0] == 0:
        return GroundGrid(z_out, trusted, np.asarray(origin, dtype=float),
                          float(cell), int(n), rejected)

    origin = np.asarray(origin, dtype=np.float64)[:2]
    ix = np.floor((xyz[:, 0] - origin[0]) / cell).astype(np.int64)
    iy = np.floor((xyz[:, 1] - origin[1]) / cell).astype(np.int64)
    inside = ((ix >= 0) & (ix < n) & (iy >= 0) & (iy < n))
    if not np.any(inside):
        return GroundGrid(z_out, trusted, origin, float(cell), int(n),
                          rejected)

    keys = ix[inside] * n + iy[inside]
    z = xyz[inside, 2]
    # lexsort's LAST key is the primary one: group by cell, sort z inside.
    order = np.lexsort((z, keys))
    keys_s = keys[order]
    z_s = z[order]
    starts = np.flatnonzero(np.r_[True, keys_s[1:] != keys_s[:-1]])
    counts = np.diff(np.r_[starts, keys_s.size])

    frac = float(np.clip(percentile / 100.0, 0.0, 1.0))
    # Vectorised linear percentile per cell (same convention as
    # np.percentile's default 'linear'), instead of a Python loop over up to
    # 2300 cells per 15 Hz scan.  The loop held the GIL long enough to starve
    # the 1 Hz publish timer when A+B+uLidar shared the CPU.
    valid = counts >= min_points
    if not np.any(valid):
        return GroundGrid(z_out, trusted, origin, float(cell), int(n),
                          rejected)
    starts_v = starts[valid]
    counts_v = counts[valid].astype(np.float64)

    def _pct(frac_v: float) -> np.ndarray:
        pos = frac_v * (counts_v - 1.0)
        lo = np.floor(pos).astype(np.int64)
        hi = np.ceil(pos).astype(np.int64)
        w = pos - lo
        return (z_s[starts_v + lo] * (1.0 - w)
                + z_s[starts_v + hi] * w)

    lo_z = _pct(frac)
    hi_z = _pct(0.8)
    keep = (hi_z - lo_z) <= spread_max
    row_all, col_all = np.divmod(keys_s[starts_v], n)
    rejected[row_all[~keep], col_all[~keep]] = True
    if np.any(keep):
        keys_v = keys_s[starts_v[keep]]
        row, col = np.divmod(keys_v, n)
        z_out[row, col] = lo_z[keep]
        trusted[row, col] = True
    return GroundGrid(z_out, trusted, origin, float(cell), int(n), rejected)


def fill_ground(grid: GroundGrid, iterations: int) -> GroundGrid:
    """Fill untrusted/NaN cells from the minimum valid 8-neighbour.

    Only NaN cells are written; trusted cells keep their own estimate.  A
    wall/leg cell therefore borrows the nearby floor and its points become
    h > h_max in :func:`classify_ground` instead of being called ground.
    """
    z = grid.z.copy()
    for _ in range(max(0, int(iterations))):
        if not np.any(np.isnan(z)):
            break
        pad = np.pad(z, 1, constant_values=np.nan)
        neigh = np.full_like(z, np.inf)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                neigh = np.fmin(neigh,
                                pad[1 + dy:z.shape[0] + 1 + dy,
                                    1 + dx:z.shape[1] + 1 + dx])
        fill = np.isnan(z) & np.isfinite(neigh)
        if not np.any(fill):
            break
        z[fill] = neigh[fill]
    return GroundGrid(z, grid.trusted.copy(), grid.origin, grid.cell, grid.n,
                      None if grid.rejected is None else grid.rejected.copy())


def classify_ground(xyz: np.ndarray, grid: GroundGrid, h_min: float,
                    h_max: float, max_range: float,
                    robot_xy: tuple[float, float]) -> np.ndarray:
    """Keep points close to the (filled) local ground and within range."""
    if xyz.shape[0] == 0:
        return np.zeros((0, 3), dtype=np.float64)
    ix = np.floor((xyz[:, 0] - grid.origin[0]) / grid.cell).astype(np.int64)
    iy = np.floor((xyz[:, 1] - grid.origin[1]) / grid.cell).astype(np.int64)
    inside = ((ix >= 0) & (ix < grid.n) & (iy >= 0) & (iy < grid.n))
    sel = np.flatnonzero(inside)
    if sel.size == 0:
        return np.zeros((0, 3), dtype=np.float64)
    ground_z = grid.z[ix[sel], iy[sel]]
    known = np.isfinite(ground_z)
    sel = sel[known]
    ground_z = ground_z[known]
    h = xyz[sel, 2] - ground_z
    r = np.hypot(xyz[sel, 0] - robot_xy[0], xyz[sel, 1] - robot_xy[1])
    keep = (h >= h_min) & (h <= h_max) & (r <= max_range)
    return xyz[sel[keep]]


def dense_ground_surface(grid: GroundGrid, robot_xy: tuple[float, float],
                         max_range: float, spacing: float) -> np.ndarray:
    """One point per ``spacing`` cell over the local ground estimate.

    The L2 returns roughly one point per 0.1 m cell, so publishing the raw
    returns leaves about half of the near-field cells without any point and
    the elevation grid marks them UNKNOWN (measured on the standing corner
    bag: 103-201 of ~200 cells inside 0.8 m).  The ground grid is a surface
    *estimate*, not a set of returns: emit a point at the centre of each
    ``spacing`` sub-cell wherever the estimate is finite.

    Cells whose own points were rejected as a wall / leg are skipped: after
    :func:`fill_ground` they borrow a neighbour floor, and flooring over an
    obstacle would erase it from the 2D map.  Points stay within
    ``max_range`` of the robot, so the patch never grows past what the uLidar
    can support.
    """
    if spacing <= 0.0:
        raise ValueError("spacing must be positive")
    z = grid.z
    if z is None or not np.any(np.isfinite(z)):
        return np.zeros((0, 3), dtype=np.float64)
    include = np.isfinite(z)
    if grid.rejected is not None:
        include &= ~grid.rejected
    ix, iy = np.nonzero(include)  # row index is x, column index is y
    if ix.size == 0:
        return np.zeros((0, 3), dtype=np.float64)
    cell = float(grid.cell)
    k = max(1, int(np.ceil(cell / spacing)))
    off = (np.arange(k, dtype=np.float64) + 0.5) * (cell / k)
    x0 = grid.origin[0] + ix * cell
    y0 = grid.origin[1] + iy * cell
    zc = z[ix, iy]
    shape = (x0.size, k, k)
    xs = np.broadcast_to(x0[:, None, None] + off[None, :, None], shape)
    ys = np.broadcast_to(y0[:, None, None] + off[None, None, :], shape)
    zz = np.broadcast_to(zc[:, None, None], shape)
    pts = np.stack((xs.ravel(), ys.ravel(), zz.ravel()), axis=-1)
    r = np.hypot(pts[:, 0] - robot_xy[0], pts[:, 1] - robot_xy[1])
    return pts[r <= max_range]


def voxel_downsample(xyz: np.ndarray, leaf: float) -> np.ndarray:
    """Keep one point per ``leaf`` cube (first occurrence in key order)."""
    if leaf <= 0.0:
        raise ValueError("leaf must be positive")
    if xyz.shape[0] == 0:
        return np.zeros((0, 3), dtype=np.float64)
    q = np.floor(xyz / leaf).astype(np.int64)
    offset = 1 << 20
    keys = ((q[:, 0] + offset) |
            ((q[:, 1] + offset) << 21) |
            ((q[:, 2] + offset) << 42))
    _, idx = np.unique(keys, return_index=True)
    return xyz[idx]


class RollingBuffer:
    """Time-windowed list of odom-frame point arrays."""

    def __init__(self, window_s: float):
        self._window_s = float(window_s)
        self._items: list[tuple[float, np.ndarray]] = []

    def add(self, stamp: float, points: np.ndarray) -> None:
        if points.size:
            self._items.append((float(stamp), points))

    def prune(self, now: float) -> None:
        cutoff = float(now) - self._window_s
        self._items = [(s, p) for s, p in self._items if s >= cutoff]

    def points(self) -> np.ndarray:
        if not self._items:
            return np.zeros((0, 3), dtype=np.float64)
        return np.vstack([p for _, p in self._items])

    def clear(self) -> None:
        self._items.clear()

    def __len__(self) -> int:
        return len(self._items)


class FillPipeline:
    """Ground-fill pipeline: per-scan ground extraction + rolling odom cache.

    The buffer holds **odom** points.  A map<-odom correction (PGO/loop
    closure jump) therefore never invalidates what is stored: ``build_fill``
    applies the current transform, so the published patch moves with the
    corrected map instead of being thrown away.  Clearing on a jump used to
    lose about 1 s of every 5 s window on this stack (the correction arrives
    every few seconds), which is why the design was changed to relocation.

    ``add_scan`` still takes the transform the scan was acquired with: it is
    used only as a fallback when ``update_tf`` has not been called yet.
    """

    def __init__(self, cfg: GroundConfig):
        self.cfg = cfg
        self.buffer = RollingBuffer(cfg.window_s)
        self._last_T: np.ndarray | None = None
        # The node runs the cloud callback and the 1 Hz publish timer in a
        # MultiThreadedExecutor; the heavy ground extraction stays outside
        # this lock, only the short buffer/T operations are protected.
        self._lock = threading.Lock()

    def add_scan(self, xyz_odom: np.ndarray, robot_xy_odom: tuple[float, float],
                 stamp: float, t_map_odom: np.ndarray) -> int:
        if xyz_odom.shape[0] == 0:
            return 0
        n = max(1, int(round(self.cfg.grid_size / self.cfg.cell)))
        half = 0.5 * n * self.cfg.cell
        origin = np.asarray(robot_xy_odom, dtype=np.float64)[:2] - half
        grid = estimate_ground_grid(
            xyz_odom, origin, self.cfg.cell, n, self.cfg.percentile,
            self.cfg.min_points, self.cfg.spread_max)
        grid = fill_ground(grid, self.cfg.fill_iterations)
        if self.cfg.dense_output:
            spacing = (self.cfg.dense_spacing
                       if self.cfg.dense_spacing > 0.0
                       else self.cfg.voxel_leaf)
            pts = dense_ground_surface(
                grid, robot_xy_odom, self.cfg.max_range, spacing)
        else:
            pts = classify_ground(
                xyz_odom, grid, self.cfg.h_min, self.cfg.h_max,
                self.cfg.max_range, robot_xy_odom)
        pts = voxel_downsample(pts, self.cfg.voxel_leaf)
        if pts.shape[0] == 0:
            return 0
        with self._lock:
            # Buffer ODOM points: a later map<-odom correction must move the
            # whole patch, not invalidate it.  t_map_odom is only a fallback
            # for callers that never call update_tf.
            if self._last_T is None:
                self._last_T = np.array(
                    t_map_odom, dtype=np.float64, copy=True)
            self.buffer.add(stamp, pts)
        return int(pts.shape[0])

    def update_tf(self, t_map_odom: np.ndarray, now: float) -> bool:
        """Record map<-odom.  Returns True only when the buffer was cleared.

        The buffer holds odom points, so a PGO correction does not invalidate
        them: the next ``build_fill`` applies the new transform and the patch
        relocates with the corrected map.  Nothing is cleared here, hence the
        constant False -- the jump handling lives in the build-time transform.
        """
        with self._lock:
            self._last_T = np.array(
                t_map_odom, dtype=np.float64, copy=True)
        return False

    def build_fill(self, now: float) -> np.ndarray:
        with self._lock:
            self.buffer.prune(now)
            pts = self.buffer.points()
            t_map_odom = (None if self._last_T is None
                          else self._last_T.copy())
        if pts.shape[0] == 0:
            return np.zeros((0, 3), dtype=np.float64)
        if t_map_odom is not None:
            pts = pts @ t_map_odom[:3, :3].T + t_map_odom[:3, 3]
        pts = voxel_downsample(pts, self.cfg.voxel_leaf)
        out = pts.copy()
        out[:, 2] += self.cfg.z_offset
        return out

    def clear(self) -> None:
        with self._lock:
            self.buffer.clear()
            self._last_T = None
