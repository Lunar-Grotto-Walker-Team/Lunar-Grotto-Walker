"""Per-cell ground reference for the PGO projector.

Pure numpy; no ROS imports.  The reference grid is a coarse (default 0.5 m)
view of the same lattice as the 0.1 m output grid: both use
``rasterizer.GridSpec``'s row=x / col=y convention and the same
``origin + half`` anchor, so a 0.1 m cell nests in exactly one coarse cell.

The reference is seeded from the robot pose (``pose.z - pose_to_ground_z``)
and propagated outward with a bounded flood fill.  It is deliberately NOT a
global odom offset: a far cell keeps the height at which its own points were
observed, which is how the gate decouples from the upstream z drift without
flattening the map (see the spec, section 2).
"""
from __future__ import annotations

import bisect
import ctypes
import os
from ctypes import POINTER, c_double, c_int, c_long

import warnings
from collections import deque
from dataclasses import dataclass, replace

import numpy as np

from .rasterizer import GridSpec


@dataclass(frozen=True)
class GroundConfig:
    """Thresholds for the per-cell ground reference."""

    ref_resolution: float = 0.5
    pose_to_ground_z: float = 0.392
    seed_radius: float = 1.5
    seed_ground_tol: float = 0.30
    clearance_min: float = 0.8
    ring_inner: float = 1.2
    ring_outer: float = 3.0
    ring_check_tol: float = 0.15
    ring_min_cells: int = 8
    #: Sorted ring p50s farther apart than this belong to different surfaces
    #: (floor vs ceiling).  The cluster nearest the pose seed wins.
    ring_cluster_gap: float = 0.40
    #: Beyond this the nearest cluster is not plausible ground -> skip frame.
    #: Normal map/pose lag (vis-cache staleness) can reach ~0.7 m, so this is
    #: deliberately much wider than ring_check_tol.
    ring_reject_tol: float = 1.00
    #: The map's local ground may sit BELOW the pose seed (cache lag), never
    #: significantly above it: the dog stands on the floor.  A cluster above
    #: the seed by more than this is a ceiling/table, not ground.
    ring_above_tol: float = 0.30
    ref_band_above: float = 0.30
    ref_band_below: float = 0.60
    out_band_above: float = 0.45
    out_band_below: float = 0.40
    min_points_ref: int = 3
    max_gap_cells: int = 4
    p_low: float = 20.0
    slope_exception_enabled: bool = False
    slope_except_above: float = 0.45
    upright_spread_max: float = 0.25
    coverage_min_ratio: float = 0.60
    coverage_min_cells: int = 200
    #: After this many consecutive guard rejections, accept the new (smaller)
    #: steady state.  Without this, a legitimate map shrink would freeze the
    #: output forever because ``previous`` never moves.
    coverage_max_skips: int = 3
    #: Replace first-pass spikes only in cells with fewer accepted points than
    #: this.  A 3x3 median over every cell would erase a 1-cell-wide wall.
    spike_filter_enabled: bool = True
    spike_min_support: int = 3
    #: Output cells with fewer accepted points than this inherit the coarse
    #: reference ground instead of their noisy 0.1 m median.  0 disables it.
    min_points_output: int = 3


def cave_profile(cfg: GroundConfig) -> GroundConfig:
    """Return the 1.5 m cave overrides of ``cfg``.

    2026-11-14: out_band_above is now 1.00 m rather than 0.50.  The dog is
    ~0.8 m tall with the arm stowed (taller with the camera deployed), so a
    0.5 m band silently dropped every beam between 0.5 and 1.0 m above the
    ground: the planner would drive under something it cannot fit under.
    A 1.5 m cave ceiling is still 0.5 m above the band and stays cut.
    """
    return replace(
        cfg,
        seed_radius=0.6,
        ring_check_tol=0.25,
        ring_outer=1.2,
        ref_band_above=0.35,
        out_band_above=1.00,
        max_gap_cells=3,
    )


def coarse_spec(spec: GridSpec, res: float = 0.5) -> GridSpec:
    """Coarse reference grid on the SAME lattice as ``spec``.

    ``res`` defaults to 0.5 m (5x the 0.1 m output lattice) but any integer
    multiple of the output resolution works: a 0.1 m cell must nest in
    exactly one coarse cell and nothing else. On a descending slope a smaller
    cell keeps the local ground local, at the cost of more cells in the flood
    (0.3 m is 2.8x the cells of 0.5 m). The side lengths are unchanged, so
    ``n_rows`` / ``n_cols`` must be exactly divisible by ``ratio``.
    """
    ratio = res / spec.resolution
    if abs(ratio - round(ratio)) > 1e-9:
        raise ValueError(
            f"coarse resolution {res} is not an integer multiple of the "
            f"output resolution {spec.resolution}; cells would not nest")
    if abs(round(spec.length_x / res) - spec.length_x / res) > 1e-9 or \
       abs(round(spec.length_y / res) - spec.length_y / res) > 1e-9:
        raise ValueError(
            f"grid length {spec.length_x}x{spec.length_y} is not a multiple "
            f"of the coarse resolution {res}")
    return GridSpec(
        resolution=res,
        length_x=spec.length_x,
        length_y=spec.length_y,
        origin_x=spec.origin_x,
        origin_y=spec.origin_y,
    )


_NATIVE = None
_NATIVE_TRIED = False
# Below this many points the numpy lexsort is cheap enough that the ctypes
# round trip is not worth it.
_NATIVE_MIN_POINTS = 50_000
# The flood is only worth a ctypes round trip above this many coarse cells.
_NATIVE_MIN_CELLS = 1_000


def _load_native_cell_sort():
    """Load libpgo_raster.so for pgo_cell_sort (PGO_RASTER_NATIVE=0 forces off)."""
    global _NATIVE, _NATIVE_TRIED
    if _NATIVE_TRIED:
        return _NATIVE
    _NATIVE_TRIED = True
    if os.environ.get("PGO_RASTER_NATIVE") == "0":
        return None
    try:
        from ament_index_python.packages import get_package_prefix
        path = os.path.join(
            get_package_prefix("pgo_raster_cpp"), "lib", "libpgo_raster.so")
        lib = ctypes.CDLL(path)
        lib.pgo_cell_sort.argtypes = [
            POINTER(c_double), c_long, c_int, c_int, c_double,
            c_double, c_double, c_double, c_double,
            POINTER(c_long), POINTER(c_long), POINTER(c_double)]
        lib.pgo_cell_sort.restype = c_int
        lib.pgo_ground_flood.argtypes = [
            c_int, c_int,
            POINTER(c_long), POINTER(c_long), POINTER(c_double),
            POINTER(c_long), c_long,
            c_double, c_double, c_int, c_double, c_int,
            c_int, c_double, c_double,
            POINTER(c_double), POINTER(c_double), POINTER(c_double),
            POINTER(c_double), POINTER(c_double),
            POINTER(ctypes.c_ubyte), POINTER(c_long)]
        lib.pgo_ground_flood.restype = c_int
        _NATIVE = lib
    except Exception:      # noqa: BLE001 -- missing lib is a normal fallback
        _NATIVE = None
    return _NATIVE


@dataclass(frozen=True)
class CellStats:
    """Per-coarse-cell z statistics, grouped the way the flood needs them."""

    counts: np.ndarray
    p20: np.ndarray
    p50: np.ndarray
    p80: np.ndarray
    starts: np.ndarray
    flat: np.ndarray
    sorted_z: np.ndarray
    #: Start index of each cell's slice in ``sorted_z``; -1 for empty cells.
    #: Dense ``(n_cells,)`` array indexed by the flat cell id, so the seed
    #: and the flood can look a cell up in O(1).
    first: np.ndarray


def _cell_indices(xy: np.ndarray, spec: GridSpec):
    """Return (row, col, inside) for world xy on ``spec``'s lattice."""
    row = np.floor(
        (spec.origin_x + spec.half_x - xy[:, 0]) / spec.resolution
    ).astype(np.int64)
    col = np.floor(
        (spec.origin_y + spec.half_y - xy[:, 1]) / spec.resolution
    ).astype(np.int64)
    inside = ((row >= 0) & (row < spec.n_rows) &
              (col >= 0) & (col < spec.n_cols))
    return row, col, inside


def _percentile_index(starts: np.ndarray, counts: np.ndarray,
                      fraction: float) -> np.ndarray:
    return starts + np.floor(fraction * (counts - 1)).astype(np.int64)


def cell_stats(xyz: np.ndarray, spec: GridSpec) -> CellStats:
    """Bin ``xyz`` into ``spec``'s coarse cells and sort z within each cell.

    Shape of ``counts`` / ``p20`` / ``p80`` is ``(n_rows, n_cols)`` (row=x,
    col=y).  Empty cells are NaN.  ``sorted_z`` is grouped by flat cell and
    sorted by z inside each group; ``starts`` indexes the first point of
    each occupied cell (in the order of ``np.unique(flat)``).
    """
    n_rows, n_cols = spec.n_rows, spec.n_cols
    n_cells = n_rows * n_cols
    counts_flat = np.zeros(n_cells, dtype=np.int64)
    p20_flat = np.full(n_cells, np.nan, dtype=np.float64)
    p50_flat = np.full(n_cells, np.nan, dtype=np.float64)
    p80_flat = np.full(n_cells, np.nan, dtype=np.float64)
    first_flat = np.full(n_cells, -1, dtype=np.int64)

    def _empty() -> CellStats:
        return CellStats(
            counts=counts_flat.reshape((n_rows, n_cols), order="F"),
            p20=p20_flat.reshape((n_rows, n_cols), order="F"),
            p50=p50_flat.reshape((n_rows, n_cols), order="F"),
            p80=p80_flat.reshape((n_rows, n_cols), order="F"),
            starts=np.zeros(0, dtype=np.int64),
            flat=np.zeros(0, dtype=np.int64),
            sorted_z=np.zeros(0, dtype=np.float64),
            first=first_flat,
        )

    if xyz.shape[0] == 0:
        return _empty()

    finite = np.isfinite(xyz).all(axis=1)
    xy = xyz[finite, :2]
    z = xyz[finite, 2]
    row, col, inside = _cell_indices(xy, spec)
    flat = col[inside] * n_rows + row[inside]
    z = z[inside]

    if flat.size == 0:
        return _empty()

    # Group by cell and sort z inside each cell.  np.lexsort is O(N log N)
    # and cost 1.08 s for 1.5M points (82% of the reference build); the
    # native kernel does the same grouping with a counting sort plus a small
    # std::sort per cell.  The numpy path stays as the fallback and is
    # bit-identical (same grouping, same within-cell order).
    lib = _load_native_cell_sort()
    sorted_flat = None
    if lib is not None and flat.size >= _NATIVE_MIN_POINTS:
        # The kernel does its own isfinite/window filtering, so pass the
        # array as-is: xyz[finite] would copy ~36 MB per frame.
        xyz_c = np.ascontiguousarray(xyz, dtype=np.float64)
        counts_c = np.zeros(n_cells, dtype=np.int64)
        first_c = np.full(n_cells, -1, dtype=np.int64)
        z_out = np.empty(xyz_c.shape[0], dtype=np.float64)
        rc = lib.pgo_cell_sort(
            xyz_c.ctypes.data_as(POINTER(c_double)), xyz_c.shape[0],
            n_rows, n_cols, float(spec.resolution),
            float(spec.origin_x), float(spec.origin_y),
            float(spec.length_x), float(spec.length_y),
            counts_c.ctypes.data_as(POINTER(c_long)),
            first_c.ctypes.data_as(POINTER(c_long)),
            z_out.ctypes.data_as(POINTER(c_double)))
        if rc == 0:
            uniq = np.flatnonzero(counts_c)
            counts = counts_c[uniq]
            starts = first_c[uniq]
            sorted_z = z_out[:int(counts.sum())]
            counts_flat[uniq] = counts
            first_flat[uniq] = starts
            sorted_flat = np.repeat(uniq, counts)

    if sorted_flat is None:
        order = np.lexsort((z, flat))
        sorted_flat = flat[order]
        sorted_z = z[order]
        starts = np.flatnonzero(
            np.r_[True, sorted_flat[1:] != sorted_flat[:-1]])
        counts = np.diff(np.r_[starts, sorted_flat.size])
        uniq = sorted_flat[starts]
        counts_flat[uniq] = counts
        first_flat[uniq] = starts
    frac20 = float(np.clip(20.0 / 100.0, 0.0, 1.0))
    frac50 = 0.5
    frac80 = float(np.clip(80.0 / 100.0, 0.0, 1.0))
    p20_flat[uniq] = sorted_z[_percentile_index(starts, counts, frac20)]
    p50_flat[uniq] = sorted_z[_percentile_index(starts, counts, frac50)]
    p80_flat[uniq] = sorted_z[_percentile_index(starts, counts, frac80)]

    return CellStats(
        counts=counts_flat.reshape((n_rows, n_cols), order="F"),
        p20=p20_flat.reshape((n_rows, n_cols), order="F"),
        p50=p50_flat.reshape((n_rows, n_cols), order="F"),
        p80=p80_flat.reshape((n_rows, n_cols), order="F"),
        starts=starts,
        flat=sorted_flat,
        sorted_z=sorted_z,
        first=first_flat,
    )


def seed_z_from_pose(pose: tuple[float, float, float],
                     cfg: GroundConfig) -> float:
    """Ground height at the robot, from the PGO body pose."""
    return float(pose[2]) - cfg.pose_to_ground_z


@dataclass(frozen=True)
class RingResult:
    ok: bool
    z: float | None
    cells: int


def _cell_centers(spec: GridSpec) -> tuple[np.ndarray, np.ndarray]:
    """World xy of every coarse cell centre, shape (n_rows, n_cols)."""
    xs = (spec.origin_x + spec.half_x -
          (np.arange(spec.n_rows) + 0.5) * spec.resolution)
    ys = (spec.origin_y + spec.half_y -
          (np.arange(spec.n_cols) + 0.5) * spec.resolution)
    return np.meshgrid(xs, ys, indexing="ij")


def ring_check(stats: CellStats, spec: GridSpec,
               pose_xy: tuple[float, float], seed_z: float,
               cfg: GroundConfig) -> RingResult:
    """Compare the map's ground ring around the robot with the pose seed.

    Only cells that look like ground (enough points, small vertical spread)
    participate: a narrow cave's ring also contains wall cells.  A sparse
    ring trusts the seed.
    """
    cx, cy = _cell_centers(spec)
    dist = np.hypot(cx - pose_xy[0], cy - pose_xy[1])
    groundlike = (
        (stats.counts >= cfg.min_points_ref)
        & np.isfinite(stats.p20)
        & ((stats.p80 - stats.p20) <= cfg.upright_spread_max)
    )
    mask = ((dist >= cfg.ring_inner) & (dist <= cfg.ring_outer)
            & groundlike)
    cells = int(np.count_nonzero(mask))
    if cells < cfg.ring_min_cells:
        return RingResult(True, None, cells)
    # Per-cell MEDIAN, not p20: p20 on a sloped cell sits ~0.8*cell*tan(theta)
    # below the cell-centre ground (~0.14 m at 22 deg, >0.25 m at 35 deg), which
    # would false-trip the tolerance on exactly the slopes we must keep.
    #
    # A corridor ring also contains ceiling-only cells (a flat ceiling has a
    # small spread, so the wall filter does not remove it).  Cluster the
    # per-cell p50s and keep the cluster nearest the seed; a plain median
    # jumps to the ceiling whenever the ceiling cells dominate (observed on
    # the corridor bag: ground ~-1.2, ceiling ~+2.16).
    vals = np.sort(stats.p50[mask])
    splits = np.flatnonzero(np.diff(vals) > cfg.ring_cluster_gap) + 1
    groups = np.split(vals, splits)
    centers = np.array([float(np.median(g)) for g in groups])
    z = float(centers[int(np.argmin(np.abs(centers - seed_z)))])
    # Distance alone is not enough: in a 1.5 m cave a pose seed 0.8 m above
    # the map floor is 0.7 m from the CEILING and 0.8 m from the floor, so the
    # nearest cluster is the ceiling.  The dog cannot stand 0.7 m below the
    # map's ground -- a cluster above the seed by more than ring_above_tol is
    # not a floor.  Skip rather than silently map the ceiling as walkable.
    ok = (abs(z - seed_z) <= cfg.ring_reject_tol
          and z <= seed_z + cfg.ring_above_tol)
    return RingResult(ok, z, cells)


def seed_cells(stats: CellStats, spec: GridSpec,
               pose_xy: tuple[float, float], seed_z: float,
               cfg: GroundConfig
               ) -> tuple[np.ndarray, np.ndarray, np.ndarray, int]:
    """Seed the disc around the robot; return (ref, accepted, pending, hits).

    A cell with no points is the sensor blind zone under the robot: write the
    pose prior.  A cell with ground-like points is seeded at their median (not
    at ``seed_z``, which would bias it).  The window is ASYMMETRIC: the map's
    floor may sit up to ``ref_band_below`` BELOW the pose seed (the vis cache
    lags the pose), but only ``seed_ground_tol`` above it.  A cave column with
    both floor and ceiling has a large spread; its floor is still below the
    pose estimate, so this window keeps the seed alive.  Otherwise the cell is
    left pending for the flood.
    """
    cx, cy = _cell_centers(spec)
    dist = np.hypot(cx - pose_xy[0], cy - pose_xy[1])
    disc = dist <= cfg.seed_radius

    ref = np.full(stats.counts.shape, np.nan, dtype=np.float64)
    accepted = np.zeros(stats.counts.shape, dtype=bool)
    pending = np.zeros(stats.counts.shape, dtype=bool)
    counts_flat = stats.counts.ravel(order="F")
    clearance_hits = 0

    for row, col in np.argwhere(disc):
        flat = int(col) * spec.n_rows + int(row)
        n = int(counts_flat[flat])
        if n == 0:
            ref[row, col] = seed_z
            accepted[row, col] = True
            continue
        start = int(stats.first[flat])
        zs = stats.sorted_z[start:start + n]
        lo = np.searchsorted(zs, seed_z - cfg.ref_band_below, side="left")
        hi = np.searchsorted(zs, seed_z + cfg.seed_ground_tol, side="right")
        if hi > lo:
            ref[row, col] = float(np.median(zs[lo:hi]))
            accepted[row, col] = True
        else:
            pending[row, col] = True
        if zs[-1] > seed_z + cfg.clearance_min:
            clearance_hits += 1

    return ref, accepted, pending, clearance_hits


@dataclass(frozen=True)
class ReferenceResult:
    ref: np.ndarray
    accepted: np.ndarray
    gap: np.ndarray
    seed_z: float
    ring_z: float | None
    ring_ok: bool
    stats: dict
    #: ring_z - (pose.z - pose_to_ground_z), before the ring replaces the seed.
    #: Diagnostic: a large value means the vis cache lags the pose.
    ring_diff: float | None = None
    #: Coarse per-cell fill value for low-support output cells.  It is the
    #: MEDIAN of the in-band candidates (not the p20 gate reference), so it
    #: aligns with the dense cells' 0.1 m medians and avoids a p20 step at the
    #: fill boundary.  None means "use ref".
    fill_ref: np.ndarray | None = None


_NEIGHBORS = ((-1, -1), (-1, 0), (-1, 1),
              (0, -1), (0, 1),
              (1, -1), (1, 0), (1, 1))


def _flood_python(stats, n_rows, n_cols, seed_flat, ref, fill_ref,
                  accepted, cfg):
    """Reference flood without the native kernel (fallback + test oracle)."""
    n_cells = n_rows * n_cols
    ref_l = ref.ravel(order="F").tolist()
    fill_l = fill_ref.ravel(order="F").tolist()
    acc_l = accepted.ravel(order="F").tolist()
    gap_l = [0] * n_cells
    counts_l = stats.counts.ravel(order="F").tolist()
    first_l = stats.first.ravel(order="F").tolist()
    z_list = stats.sorted_z.tolist()
    p20_l = stats.p20.ravel(order="F").tolist()
    p50_l = stats.p50.ravel(order="F").tolist()
    p80_l = stats.p80.ravel(order="F").tolist()

    queue = deque(int(i) for i in seed_flat)
    p_frac = float(np.clip(cfg.p_low / 100.0, 0.0, 1.0))
    max_gap = int(cfg.max_gap_cells)
    band_below = cfg.ref_band_below
    band_above = cfg.ref_band_above
    min_pts = int(cfg.min_points_ref)
    slope_ex = bool(cfg.slope_exception_enabled)
    spread_max = cfg.upright_spread_max
    slope_above = cfg.slope_except_above

    while queue:
        cur = queue.popleft()
        col, row = divmod(cur, n_rows)
        ref_nb = ref_l[cur]
        fill_nb = fill_l[cur]
        gap_nb = gap_l[cur]
        for dr, dc in _NEIGHBORS:
            rr = row + dr
            cc = col + dc
            if rr < 0 or rr >= n_rows or cc < 0 or cc >= n_cols:
                continue
            nb = cc * n_rows + rr
            if acc_l[nb]:
                continue
            n = counts_l[nb]
            if n == 0:
                if gap_nb + 1 <= max_gap:
                    ref_l[nb] = ref_nb
                    fill_l[nb] = fill_nb
                    gap_l[nb] = gap_nb + 1
                    acc_l[nb] = True
                    queue.append(nb)
                continue
            start = first_l[nb]
            end = start + n
            lo = bisect.bisect_left(z_list, ref_nb - band_below, start, end)
            hi = bisect.bisect_right(z_list, ref_nb + band_above, lo, end)
            count = hi - lo
            if count >= min_pts:
                ref_l[nb] = z_list[lo + int(p_frac * (count - 1))]
                mid = count // 2
                if count % 2 == 1:
                    fill_l[nb] = z_list[lo + mid]
                else:
                    fill_l[nb] = 0.5 * (z_list[lo + mid - 1] + z_list[lo + mid])
                gap_l[nb] = 0
                acc_l[nb] = True
                queue.append(nb)
                continue
            if not slope_ex:
                continue
            if p80_l[nb] - p20_l[nb] > spread_max:
                continue
            ref_cand = p20_l[nb]
            if abs(ref_cand - ref_nb) <= slope_above:
                ref_l[nb] = ref_cand
                fill_l[nb] = p50_l[nb]
                gap_l[nb] = 0
                acc_l[nb] = True
                queue.append(nb)

    shape = (n_rows, n_cols)
    return (np.asarray(ref_l, dtype=np.float64).reshape(shape, order="F"),
            np.asarray(acc_l, dtype=bool).reshape(shape, order="F"),
            np.asarray(gap_l, dtype=np.int16).reshape(shape, order="F"),
            np.asarray(fill_l, dtype=np.float64).reshape(shape, order="F"))


def _flood(stats, n_rows, n_cols, seed_flat, ref, fill_ref, accepted, cfg):
    """Dispatch: native C++ flood when the library is present and worth it."""
    lib = _load_native_cell_sort()
    if lib is None or n_rows * n_cols < _NATIVE_MIN_CELLS:
        return _flood_python(stats, n_rows, n_cols, seed_flat, ref, fill_ref,
                             accepted, cfg)
    n_cells = n_rows * n_cols
    counts = np.ascontiguousarray(
        stats.counts.ravel(order="F"), dtype=np.int64)
    first = np.ascontiguousarray(stats.first, dtype=np.int64)
    z = np.ascontiguousarray(stats.sorted_z, dtype=np.float64)
    p20 = np.ascontiguousarray(stats.p20.ravel(order="F"), dtype=np.float64)
    p50 = np.ascontiguousarray(stats.p50.ravel(order="F"), dtype=np.float64)
    p80 = np.ascontiguousarray(stats.p80.ravel(order="F"), dtype=np.float64)
    ref_l = np.ascontiguousarray(
        ref.ravel(order="F"), dtype=np.float64).copy()
    fill_l = np.ascontiguousarray(
        fill_ref.ravel(order="F"), dtype=np.float64).copy()
    acc_l = np.ascontiguousarray(
        accepted.ravel(order="F"), dtype=np.uint8).copy()
    gap_l = np.zeros(n_cells, dtype=np.int64)
    seeds = np.ascontiguousarray(np.asarray(seed_flat, dtype=np.int64))
    rc = lib.pgo_ground_flood(
        n_rows, n_cols,
        counts.ctypes.data_as(POINTER(c_long)),
        first.ctypes.data_as(POINTER(c_long)),
        z.ctypes.data_as(POINTER(c_double)),
        seeds.ctypes.data_as(POINTER(c_long)), int(seeds.size),
        float(cfg.ref_band_below), float(cfg.ref_band_above),
        int(cfg.min_points_ref),
        float(np.clip(cfg.p_low / 100.0, 0.0, 1.0)),
        int(cfg.max_gap_cells),
        int(bool(cfg.slope_exception_enabled)),
        float(cfg.upright_spread_max), float(cfg.slope_except_above),
        p20.ctypes.data_as(POINTER(c_double)),
        p50.ctypes.data_as(POINTER(c_double)),
        p80.ctypes.data_as(POINTER(c_double)),
        ref_l.ctypes.data_as(POINTER(c_double)),
        fill_l.ctypes.data_as(POINTER(c_double)),
        acc_l.ctypes.data_as(POINTER(ctypes.c_ubyte)),
        gap_l.ctypes.data_as(POINTER(c_long)))
    if rc != 0:
        return _flood_python(stats, n_rows, n_cols, seed_flat, ref, fill_ref,
                             accepted, cfg)
    shape = (n_rows, n_cols)
    return (ref_l.reshape(shape, order="F"),
            acc_l.astype(bool).reshape(shape, order="F"),
            gap_l.astype(np.int16).reshape(shape, order="F"),
            fill_l.reshape(shape, order="F"))


def build_reference(xyz: np.ndarray, pose: tuple[float, float, float],
                    spec: GridSpec, cfg: GroundConfig) -> ReferenceResult:
    """Build the per-cell ground reference for one map frame.

    The seed anchors the reference at the robot; the flood then propagates
    it through the map's own points.  Accepted cells are decided once; a
    rejected cell-with-points can be retried by another accepted neighbour
    (at most 8 band lookups), so "reachable" is independent of BFS order.
    """
    stats = cell_stats(xyz, spec)
    pose_seed = seed_z_from_pose(pose, cfg)
    ring = ring_check(stats, spec, pose[:2], pose_seed, cfg)
    ring_diff = None if ring.z is None else ring.z - pose_seed
    if not ring.ok:
        return ReferenceResult(
            ref=np.full(stats.counts.shape, np.nan),
            accepted=np.zeros(stats.counts.shape, dtype=bool),
            gap=np.zeros(stats.counts.shape, dtype=np.int16),
            seed_z=pose_seed,
            ring_z=ring.z,
            ring_ok=False,
            stats={"seed_cells": 0, "accepted_cells": 0, "ground_cells": 0,
                   "pending_cells": 0, "unreachable_cells":
                       int(np.count_nonzero(stats.counts)),
                   "clearance_hits": 0, "ring_cells": ring.cells},
            ring_diff=ring_diff,
        )
    # The ring measured the MAP's local ground; the pose seed is only a model
    # of it.  The vis cache lags the pose by up to its rebuild interval, so a
    # 0.3-0.7 m difference is normal -- start the flood on the map's floor.
    seed_z = pose_seed if ring.z is None else ring.z

    ref, accepted, pending, clearance_hits = seed_cells(
        stats, spec, pose[:2], seed_z, cfg)
    seed_accepted = accepted.copy()
    # Sparse output cells inherit this value: the in-band MEDIAN (p50), not
    # the p20 gate reference.  p20 is deliberately low-biased for ground
    # selection; filling with it would step down at every dense/fill boundary.
    fill_ref = ref.copy()

    n_rows, n_cols = stats.counts.shape
    seed_flat = np.array(
        [int(cc) * n_rows + int(rr) for rr, cc in np.argwhere(accepted)],
        dtype=np.int64)
    ref, accepted, gap, fill_ref = _flood(
        stats, n_rows, n_cols, seed_flat, ref, fill_ref, accepted, cfg)

    finite = np.isfinite(ref)
    stats_out = {
        "seed_cells": int(np.count_nonzero(seed_accepted)),
        "accepted_cells": int(np.count_nonzero(accepted & finite)),
        "ground_cells": int(np.count_nonzero(accepted & (stats.counts > 0))),
        "pending_cells": int(np.count_nonzero(pending)),
        "unreachable_cells": int(np.count_nonzero(
            (stats.counts > 0) & ~accepted)),
        "clearance_hits": clearance_hits,
        "ring_cells": ring.cells,
    }
    return ReferenceResult(
        ref=ref, accepted=accepted, gap=gap, seed_z=seed_z,
        ring_z=ring.z, ring_ok=True, stats=stats_out, ring_diff=ring_diff,
        fill_ref=fill_ref,
    )


def select_ground(xyz: np.ndarray, result: ReferenceResult,
                  spec: GridSpec, cfg: GroundConfig) -> np.ndarray:
    """Return the subset mask of ``xyz`` that is ground for its cell.

    ``spec`` is the COARSE grid.  Points outside it, or in a cell with no
    reference, stay unknown (False).
    """
    n = xyz.shape[0]
    keep = np.zeros(n, dtype=bool)
    if n == 0:
        return keep
    finite = np.isfinite(xyz).all(axis=1)
    if not np.any(finite):
        return keep
    xy = xyz[finite, :2]
    z = xyz[finite, 2]
    row, col, inside = _cell_indices(xy, spec)
    ref = np.full(xy.shape[0], np.nan, dtype=np.float64)
    ref[inside] = result.ref[row[inside], col[inside]]
    sel = (inside & np.isfinite(ref)
           & (z >= ref - cfg.out_band_below)
           & (z <= ref + cfg.out_band_above))
    keep[np.flatnonzero(finite)] = sel
    return keep


def output_band_counts(xyz: np.ndarray, result: ReferenceResult,
                       spec: GridSpec, cfg: GroundConfig
                       ) -> tuple[int, int]:
    """Count known-ref points dropped above / below the output band.

    High points are the > out_band_above obstacles that become unknown
    (plannable-through) instead of a visible step; low points are dropped
    pits.  The node reports both so a bag run can see what the band loses.
    """
    if xyz.shape[0] == 0:
        return 0, 0
    finite = np.isfinite(xyz).all(axis=1)
    if not np.any(finite):
        return 0, 0
    xy = xyz[finite, :2]
    z = xyz[finite, 2]
    row, col, inside = _cell_indices(xy, spec)
    ref = np.full(xy.shape[0], np.nan, dtype=np.float64)
    ref[inside] = result.ref[row[inside], col[inside]]
    known = inside & np.isfinite(ref)
    above = known & (z > ref + cfg.out_band_above)
    below = known & (z < ref - cfg.out_band_below)
    return int(np.count_nonzero(above)), int(np.count_nonzero(below))


def despike_sparse(elevation: np.ndarray, counts: np.ndarray,
                   min_support: int = 3, radius: int = 1) -> np.ndarray:
    """Median-filter only the cells with fewer than ``min_support`` points.

    An unconditional 3x3 median would erase a 1-cell-wide wall (the wall's
    vertical surface projects to a single line of cells); walls are dense, so
    leaving supported cells untouched keeps them.  Sparse cells are where
    first-pass spikes come from, and their neighbourhood median is a safe
    replacement.  Unobserved cells (NaN) stay NaN.
    """
    elevation = np.asarray(elevation, dtype=np.float64)
    counts = np.asarray(counts)
    if elevation.shape != counts.shape:
        raise ValueError(
            f"elevation {elevation.shape} and counts {counts.shape} differ")
    if radius < 1:
        return elevation.copy()

    candidate = ((counts > 0) & (counts < min_support)
                 & np.isfinite(elevation))
    out = elevation.copy()
    if not candidate.any():
        return out

    # np.nanmedian over a (9, H, W) stack sorts every one of the ~490k
    # columns and cost 924 ms per frame at 700x700, which alone capped
    # reference mode at ~1 Hz. Only the sparse cells are candidates, so the
    # window is gathered for those rows/cols only and the median runs on a
    # (9, k) array instead.
    pad = np.pad(elevation, radius, mode="constant", constant_values=np.nan)
    rows, cols = np.nonzero(candidate)
    rows_p = rows + radius
    cols_p = cols + radius
    stack = np.stack([
        pad[rows_p + dy, cols_p + dx]
        for dy in range(-radius, radius + 1)
        for dx in range(-radius, radius + 1)
    ])
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        median = np.nanmedian(stack, axis=0)
    out[rows, cols] = median
    return out


def fill_sparse_from_reference(elevation: np.ndarray, counts: np.ndarray,
                               ref_coarse: np.ndarray, ratio: int,
                               min_support: int) -> np.ndarray:
    """Give low-support output cells the coarse reference ground.

    A cell with 1-2 accepted points has a noisy median and is the main source
    of first-observation phantom obstacles.  The 0.5 m coarse ref is a
    statistically supported local ground (tens of points) accepted by the
    flood, so using it for those cells is both smoother and semantically
    consistent.  Cells with ``count >= min_support`` keep their own median;
    unobserved cells (count 0) stay NaN.
    """
    elevation = np.asarray(elevation, dtype=np.float64)
    counts = np.asarray(counts)
    ref_coarse = np.asarray(ref_coarse, dtype=np.float64)
    if ratio < 1:
        raise ValueError(f"ratio must be >= 1, got {ratio}")
    if elevation.shape != counts.shape:
        raise ValueError(
            f"elevation {elevation.shape} and counts {counts.shape} differ")
    ref_fine = np.repeat(np.repeat(ref_coarse, ratio, axis=0),
                         ratio, axis=1)
    if ref_fine.shape != elevation.shape:
        raise ValueError(
            f"coarse ref expanded to {ref_fine.shape} does not match "
            f"elevation {elevation.shape} (ratio {ratio})")
    sparse = (counts > 0) & (counts < min_support) & np.isfinite(ref_fine)
    out = elevation.copy()
    out[sparse] = ref_fine[sparse]
    return out


def coverage_ok(current: int, previous: int | None,
                cfg: GroundConfig) -> bool:
    """Guard against publishing a frame whose ground area collapsed.

    ``previous`` is the previous *published* frame's ``ground_cells``; None
    or a tiny previous frame disables the guard (startup / first frames).
    """
    if previous is None or previous < cfg.coverage_min_cells:
        return True
    return current >= max(cfg.coverage_min_cells,
                          int(cfg.coverage_min_ratio * previous))
