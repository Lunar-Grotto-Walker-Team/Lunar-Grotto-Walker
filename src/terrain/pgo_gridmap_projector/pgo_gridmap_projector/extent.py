"""Adaptive grid extent for the PGO projector.

The projector used to rasterize into a fixed 30 m x 30 m grid centred on
the map origin.  Exploration needs the grid to grow with what has actually
been observed, so this module computes the grid geometry from the point
cloud's bounding box instead.

Rules (see the design spec section 6.2):

* the extent follows the observed bbox, expanded by ``margin`` and rounded
  outward to ``quantum``;
* it never shrinks unless ``shrink=True``; growth is a union with the
  previous geometry, so a loop closure that moves points slightly cannot
  make the grid oscillate;
* the lower-left corner is snapped outward to a multiple of ``resolution``.
  This is load-bearing: if the grid origin is not on the cell lattice, the
  same world point lands in a different cell from frame to frame and the
  derived slope / roughness layers sprout fake steps;
* ``max_length`` is a runaway guard.  Once the union reaches it the grid is
  capped and its centre stays at the bbox centre of the accumulated map.
  Because ``/aft_pgo_map`` always carries the whole map, this is a HARD cap:
  the grid does not follow the robot into territory beyond it, and the far
  side is clipped.  Raise the cap (or split the map) if a site can exceed
  it.  A WARN is logged by the projector when a frame is clipped.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .rasterizer import GridSpec


@dataclass(frozen=True)
class ExtentConfig:
    """Parameters controlling how the grid grows."""

    resolution: float
    margin: float = 2.0
    quantum: float = 1.0
    min_length: float = 10.0
    max_length: float = 120.0  # 0.0 = unlimited
    shrink: bool = False
    #: Number of extreme samples trimmed per axis before taking the bbox.
    #: 1 is enough for a single stray point; dense clouds can use more.
    outlier_trim: int = 1


def _robust_bounds(values: np.ndarray, trim: int) -> tuple[float, float]:
    """Min/max after dropping ``trim`` extremes on each side."""
    n = values.size
    if trim <= 0 or n <= 2 * trim + 1:
        return float(values.min()), float(values.max())
    ordered = np.partition(values, (trim, n - 1 - trim))
    return float(ordered[trim]), float(ordered[n - 1 - trim])


def _ceil_to(value: float, step: float) -> float:
    return math.ceil(value / step - 1e-9) * step


def _floor_to(value: float, step: float) -> float:
    return math.floor(value / step + 1e-9) * step


def _snap_spec(cfg: ExtentConfig, xmin: float, xmax: float,
               ymin: float, ymax: float) -> GridSpec:
    """Turn a bounding box into a cell-aligned GridSpec."""
    res = cfg.resolution
    cx = 0.5 * (xmin + xmax)
    cy = 0.5 * (ymin + ymax)
    length_x = max(xmax - xmin, cfg.min_length)
    length_y = max(ymax - ymin, cfg.min_length)

    if cfg.max_length > 0.0:
        length_x = min(length_x, cfg.max_length)
        length_y = min(length_y, cfg.max_length)

    length_x = _ceil_to(length_x, cfg.quantum)
    length_y = _ceil_to(length_y, cfg.quantum)

    # Snap the lower-left corner OUTWARD (floor) so the requested bbox is
    # always covered, then make the side lengths whole cells.  origin_* is
    # the grid CENTRE, matching grid_map's convention.
    lower_left_x = _floor_to(cx - 0.5 * length_x, res)
    lower_left_y = _floor_to(cy - 0.5 * length_y, res)
    length_x = _ceil_to(length_x, res)
    length_y = _ceil_to(length_y, res)

    return GridSpec(
        resolution=res,
        length_x=length_x,
        length_y=length_y,
        origin_x=lower_left_x + 0.5 * length_x,
        origin_y=lower_left_y + 0.5 * length_y,
    )


def update_extent(xyz: np.ndarray, cfg: ExtentConfig,
                  previous: GridSpec | None,
                  z_min: float = -np.inf,
                  z_max: float = np.inf) -> GridSpec:
    """Return the grid geometry for this frame.

    Args:
        xyz: (N, 3) points in the map frame (may contain NaN).
        cfg: growth parameters.
        previous: geometry of the last frame, or None on the first frame.
        z_min, z_max: points outside this height gate do not influence the
            extent (they are dropped by the rasterizer as well).
    """
    xyz = np.asarray(xyz, dtype=np.float64)

    if previous is None:
        prev_min_x = prev_max_x = prev_min_y = prev_max_y = None
    else:
        prev_min_x = previous.origin_x - 0.5 * previous.length_x
        prev_max_x = previous.origin_x + 0.5 * previous.length_x
        prev_min_y = previous.origin_y - 0.5 * previous.length_y
        prev_max_y = previous.origin_y + 0.5 * previous.length_y

    if xyz.ndim != 2 or xyz.shape[1] != 3 or xyz.shape[0] == 0:
        if previous is not None:
            return previous
        return _snap_spec(cfg, 0.0, 0.0, 0.0, 0.0)

    finite = np.isfinite(xyz).all(axis=1)
    z = xyz[:, 2]
    keep = finite & (z >= z_min) & (z <= z_max)
    if not np.any(keep):
        if previous is not None:
            return previous
        return _snap_spec(cfg, 0.0, 0.0, 0.0, 0.0)

    kept = xyz[keep]
    # Trim the extreme samples before taking the bbox.  /aft_pgo_map is a
    # dense point cloud, so a handful of stray points (registration
    # outliers) would otherwise drag the grid by kilometres and leave the
    # real map outside it.  Trimming k points per axis costs nothing and
    # removes exactly that class of point.
    xmin, xmax = _robust_bounds(kept[:, 0], cfg.outlier_trim)
    ymin, ymax = _robust_bounds(kept[:, 1], cfg.outlier_trim)
    xmin -= cfg.margin
    xmax += cfg.margin
    ymin -= cfg.margin
    ymax += cfg.margin

    if prev_min_x is not None and cfg.shrink:
        # Shrink mode: the observed bbox alone decides the geometry.
        pass
    elif prev_min_x is not None:
        xmin = min(xmin, prev_min_x)
        xmax = max(xmax, prev_max_x)
        ymin = min(ymin, prev_min_y)
        ymax = max(ymax, prev_max_y)

    return _snap_spec(cfg, xmin, xmax, ymin, ymax)


def window_spec(pose_xy, length: float, resolution: float,
                quantum: float = 1.0) -> "GridSpec | None":
    """Fixed ``length`` x ``length`` grid centred on the robot pose.

    The accumulated map keeps growing (100 m+ on a long corridor), and every
    downstream node is O(cells), so a growing grid makes the chain miss its
    rate.  This returns a constant-size window instead: the global costmap
    only ever consumes a 60 x 60 m rolling viewport, so a window slightly
    larger than that (70 m by default) covers everything the consumer can
    see while keeping the cell count fixed.

    ``quantum`` snaps the centre to a whole-metre lattice.  Without it the
    origin would move by a fraction of a cell every frame and the same world
    point would land in different cells from one frame to the next, which is
    the same fake-step problem ``_snap_spec`` guards against for the
    bbox-driven extent.  ``quantum <= 0`` disables the snapping.

    Returns ``None`` when the pose is not known yet (caller keeps its
    previous geometry).
    """
    if pose_xy is None:
        return None
    cx = float(pose_xy[0])
    cy = float(pose_xy[1])
    if quantum > 0.0:
        cx = round(cx / quantum) * quantum
        cy = round(cy / quantum) * quantum
    return GridSpec(resolution=resolution, length_x=length, length_y=length,
                    origin_x=cx, origin_y=cy)
