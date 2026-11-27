"""Pure-numpy rasterization of a point cloud into a 2-D elevation grid.

No ROS imports. The grid geometry matches grid_map's own convention for a
map of `length` centred at `origin` (see the plan's "Measured grid geometry"
table): cell (row, col) for world (x, y) is

    row = floor((origin_x + half_x - x) / resolution)
    col = floor((origin_y + half_y - y) / resolution)

The `origin_x` / `origin_y` terms are REQUIRED: grid_map's
``getIndexFromPosition`` computes ``(position - offset - mapPosition) /
resolution``, where ``offset`` is half the map length and ``mapPosition`` is
the grid's centre, so a grid that is not centred on (0, 0) shifts its cells
by the origin. With the default ``origin_x = origin_y = 0.0`` the formula
reduces to the origin-blind one.

and the cell's value is the aggregation of all z of the points that land in
it. Empty cells are NaN, which is grid_map's own "invalid" sentinel
(``GridMap::isValid`` is ``std::isfinite``).
"""
from __future__ import annotations

import os
from dataclasses import dataclass

import numpy as np


AGGREGATIONS = ("median", "mean", "max", "min")

# Optional native median kernel (pgo_raster_cpp).  Loaded lazily so the
# pure-numpy tests keep working when the package is not built, and so a
# missing library degrades to numpy instead of failing.
_NATIVE = None
_NATIVE_TRIED = False


def _load_native():
    """Return the ctypes library handle, or None when unavailable."""
    global _NATIVE, _NATIVE_TRIED
    if _NATIVE_TRIED:
        return _NATIVE
    _NATIVE_TRIED = True
    if os.environ.get("PGO_RASTER_NATIVE", "1") == "0":
        return None
    try:
        import ctypes

        from ament_index_python.packages import get_package_prefix

        path = os.path.join(get_package_prefix("pgo_raster_cpp"), "lib",
                            "libpgo_raster.so")
        lib = ctypes.CDLL(path)
        d = ctypes.POINTER(ctypes.c_double)
        n = ctypes.POINTER(ctypes.c_long)
        lib.pgo_rasterize_median.restype = ctypes.c_int
        lib.pgo_rasterize_median.argtypes = [
            d, ctypes.c_long,          # xyz, n
            ctypes.c_int, ctypes.c_int,  # n_rows, n_cols
            ctypes.c_double,             # resolution
            ctypes.c_double, ctypes.c_double,   # origin_x, origin_y
            ctypes.c_double, ctypes.c_double,   # length_x, length_y
            ctypes.c_double, ctypes.c_double,   # z_min, z_max
            d, n,                        # out_elev, out_counts
            d,                           # out_max_z (may be NULL)
        ]
        _NATIVE = lib
    except Exception:  # noqa: BLE001 - any failure means "use numpy"
        _NATIVE = None
    return _NATIVE


def native_available() -> bool:
    return _load_native() is not None


class RasterError(ValueError):
    """Raised for an invalid aggregation name or malformed input."""


@dataclass(frozen=True)
class GridSpec:
    """Geometry of the rasterization grid.

    The grid is centred on (origin_x, origin_y) and spans length_x by
    length_y metres.
    """

    resolution: float
    length_x: float
    length_y: float
    origin_x: float = 0.0
    origin_y: float = 0.0

    @property
    def n_rows(self) -> int:
        """Number of cells along x (the grid's row axis)."""
        return int(round(self.length_x / self.resolution))

    @property
    def n_cols(self) -> int:
        """Number of cells along y (the grid's column axis)."""
        return int(round(self.length_y / self.resolution))

    @property
    def half_x(self) -> float:
        return 0.5 * self.length_x

    @property
    def half_y(self) -> float:
        return 0.5 * self.length_y


def _aggregate(flat: np.ndarray, zs: np.ndarray, n_cells: int,
               aggregation: str) -> np.ndarray:
    """Reduce per-cell z values into a flat array of length n_cells.

    `flat` holds the flat cell index of each point; `zs` the matching z.
    The result is indexed by flat cell index (grid_map's column-major
    order: col * n_rows + row).
    """
    out = np.full(n_cells, np.nan, dtype=np.float64)
    if zs.size == 0:
        return out

    # Sort by (flat, z) — lexsort's LAST key is the primary one. Sorting by
    # flat alone would leave z unordered WITHIN each cell group, which makes
    # the median pick the wrong element. (Verified: a 3-point cell with
    # z = 0.1, 0.3, 0.2 returns 0.3 with a plain argsort on flat.)
    order = np.lexsort((zs, flat))
    sorted_flat = flat[order]
    sorted_z = zs[order]

    # Start offset of each occupied cell within the sorted arrays.
    starts = np.flatnonzero(np.r_[True, sorted_flat[1:] != sorted_flat[:-1]])
    counts = np.diff(np.r_[starts, sorted_flat.size])
    uniq = sorted_flat[starts]
    # Group number (0..n_groups-1) of each sorted point, for the ufunc.at
    # reductions. NOTE: `uniq` is a SPARSE index array (the flat cell
    # indices), not a dense 0..n_groups-1 range, so it must not be passed
    # to np.bincount as the index.
    group = np.repeat(np.arange(uniq.size), counts)

    if aggregation == "median":
        # For an even count, average the two middle values.
        mid = starts + (counts - 1) // 2
        values = sorted_z[mid].astype(np.float64)
        even = (counts % 2) == 0
        if np.any(even):
            values[even] = 0.5 * (sorted_z[mid[even]] + sorted_z[mid[even] + 1])
    elif aggregation == "mean":
        totals = np.bincount(group, weights=sorted_z, minlength=uniq.size)
        values = totals / counts
    elif aggregation == "max":
        values = np.full(uniq.size, -np.inf)
        np.maximum.at(values, group, sorted_z)
    elif aggregation == "min":
        values = np.full(uniq.size, np.inf)
        np.minimum.at(values, group, sorted_z)
    else:
        raise RasterError(
            f"unknown aggregation {aggregation!r}; expected one of {AGGREGATIONS}"
        )

    out[uniq] = values
    return out


def rasterize(xyz: np.ndarray, spec: GridSpec, aggregation: str = "median",
              z_min: float = -0.5, z_max: float = 0.5,
              return_counts: bool = False, return_max_z: bool = False):
    """Rasterize a point cloud into a (n_rows, n_cols) elevation grid.

    Args:
        xyz: (N, 3) array of world-frame x, y, z.
        spec: grid geometry.
        aggregation: one of AGGREGATIONS.
        z_min: points with z < z_min are dropped.
        z_max: points with z > z_max are dropped.
        return_counts: also return the per-cell accepted-point counts.

    Returns:
        2-D float64 array of shape (n_rows, n_cols); NaN where empty. The
        result is F-contiguous (Fortran order): the flat cell index of cell
        (row, col) is ``col * n_rows + row``, so a caller flattening the
        array must use ``arr.flatten(order="F")`` — the default C-order
        ``arr.flatten()`` would transpose the whole map.  With
        ``return_counts`` the result is ``(elevation, counts)`` with
        ``counts`` the same shape, int64, flat-indexed the same way.
    """
    if aggregation not in AGGREGATIONS:
        raise RasterError(
            f"unknown aggregation {aggregation!r}; expected one of {AGGREGATIONS}"
        )

    n_rows, n_cols = spec.n_rows, spec.n_cols
    out = np.full((n_rows, n_cols), np.nan, dtype=np.float64)

    def _ret(values):
        if return_counts:
            return values, np.zeros((n_rows, n_cols), dtype=np.int64)
        return values

    xyz = np.asarray(xyz, dtype=np.float64)
    # Shape validation comes FIRST: a malformed EMPTY array (e.g.
    # np.zeros((0, 4))) must raise RasterError rather than silently
    # returning a blank grid.
    if xyz.ndim == 1 and xyz.size == 0:
        # A 1-D empty array (np.asarray([])) is the canonical "no points"
        # input; treat it as an empty (0, 3) cloud.
        xyz = xyz.reshape(0, 3)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise RasterError(f"xyz must have shape (N, 3), got {xyz.shape}")
    if xyz.size == 0:
        return _ret(out)

    # Native median path: counting sort + per-cell nth_element, ~10x faster
    # than np.lexsort on the large maps this stack publishes.  It must be
    # bit-compatible with _aggregate(..., "median"); test_rasterizer checks
    # that.  Other aggregations stay in numpy.
    if aggregation == "median":
        native = _load_native()
        if native is not None:
            import ctypes

            xyz_c = np.ascontiguousarray(xyz, dtype=np.float64)
            n_cells = n_rows * n_cols
            elev = np.empty(n_cells, dtype=np.float64)
            counts = np.empty(n_cells, dtype=np.int64)
            d = ctypes.POINTER(ctypes.c_double)
            max_z = (np.full((n_rows, n_cols), np.nan, dtype=np.float64)
                     if return_max_z else None)
            rc = native.pgo_rasterize_median(
                xyz_c.ctypes.data_as(d), xyz_c.shape[0],
                n_rows, n_cols, spec.resolution,
                spec.origin_x, spec.origin_y, spec.length_x, spec.length_y,
                z_min, z_max,
                elev.ctypes.data_as(d),
                counts.ctypes.data_as(ctypes.POINTER(ctypes.c_long)),
                (max_z.ctypes.data_as(d) if return_max_z
                 else ctypes.POINTER(ctypes.c_double)()))
            if rc == 0:
                result = elev.reshape((n_rows, n_cols), order="F")
                if return_max_z:
                    # Fortran order because the kernel fills the flat buffer
                    # in grid_map's column-major order.
                    max_z = max_z.reshape((n_rows, n_cols), order="F")
                    if return_counts:
                        return (result,
                                counts.reshape((n_rows, n_cols), order="F"),
                                max_z)
                    return result, max_z
                if return_counts:
                    return result, counts.reshape((n_rows, n_cols),
                                                  order="F")
                return result

    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    # Drop non-finite coordinates explicitly: NaN/inf would otherwise
    # produce a bogus integer index (and can silently contaminate a real
    # cell instead of being rejected by the window mask).
    keep = np.isfinite(xyz).all(axis=1) & (z >= z_min) & (z <= z_max)
    if not np.any(keep):
        return _ret(out)
    x, y, z = x[keep], y[keep], z[keep]

    # grid_map's assignment rule (measured, not derived). grid_map computes
    #   index = (position - offset - mapPosition) / resolution
    # with `offset` = half the map length and `mapPosition` = the grid's
    # centre, hence the origin term:
    #   row = floor((origin_x + half_x - x) / resolution)
    #   col = floor((origin_y + half_y - y) / resolution)
    row = np.floor(
        (spec.origin_x + spec.half_x - x) / spec.resolution).astype(np.int64)
    col = np.floor(
        (spec.origin_y + spec.half_y - y) / spec.resolution).astype(np.int64)

    inside = (row >= 0) & (row < n_rows) & (col >= 0) & (col < n_cols)
    if not np.any(inside):
        return _ret(out)
    row, col, z = row[inside], col[inside], z[inside]

    # Flat index in grid_map's column-major buffer: col * n_rows + row.
    flat = col * n_rows + row
    values = _aggregate(flat, z, n_rows * n_cols, aggregation)
    # Inverse of the flat indexing: values[col * n_rows + row] is the
    # value of cell (row, col), so the reshape must be Fortran-order.
    result = values.reshape((n_rows, n_cols), order="F")
    if return_counts:
        counts = np.bincount(flat, minlength=n_rows * n_cols)
        return result, counts.reshape((n_rows, n_cols), order="F")
    return result
