"""O(N) post-processing of the elevation grid.

Replaces the ``grid_map_filters_demo`` chain, whose ``MeanInRadiusFilter``
sweeps every cell of the grid and scans a circle around each one
(O(cells * radius^2)).  Everything here is a vectorised numpy / OpenCV pass:
cost is linear in the number of cells and independent of the radius.

Conventions match ``pgo_gridmap_projector.rasterizer``:

* ``elevation[row, col]``;
* **row is the x axis, col is the y axis**;
* NaN means "never observed".
"""
from __future__ import annotations

import cv2
import numpy as np
from scipy import ndimage


def _radius_cells(radius_m: float, resolution: float) -> int:
    # FLOOR, not round: the old circle filter with radius 0.15 m at
    # 0.10 m/cell took the 3x3 neighbourhood (diagonal 0.1414 m), and a
    # round() here is fragile -- round(0.15 / 0.1) happens to be 1 only
    # because the float division lands on 1.4999999999999998.
    return max(1, int(radius_m / resolution))


def fill_invalid(elevation: np.ndarray, max_cells: int = 0) -> np.ndarray:
    """Fill NaN cells with the value of the nearest finite cell.

    Nearest-neighbour filling is the cheap stand-in for the old
    ``gridMapCv/InpaintFilter``: it only feeds the derived layers, the raw
    ``elevation`` layer is never modified.  An all-NaN input is returned
    unchanged (there is nothing to fill from).
    """
    arr = np.asarray(elevation, dtype=np.float64)
    invalid = ~np.isfinite(arr)
    if not invalid.any() or invalid.all():
        return arr.copy()
    indices = ndimage.distance_transform_edt(
        invalid, return_distances=False, return_indices=True)
    filled = arr[tuple(indices)]
    if max_cells > 0:
        # Bound the fill. An unbounded nearest-neighbour fill turns every
        # large hole -- the space behind a cave wall, the strip a crest
        # occludes -- into flat ground, and leaves a one-cell cliff at its
        # border. Holes wider than `max_cells` stay NaN so the derived slope /
        # roughness stay NaN there and the bridge reports UNKNOWN.
        dist = ndimage.distance_transform_edt(invalid)
        filled[dist > float(max_cells)] = np.nan
    return filled


def _box_mean_filled(filled: np.ndarray, radius_m: float,
                     resolution: float) -> np.ndarray:
    if not np.isfinite(filled).any():
        return filled
    k = 2 * _radius_cells(radius_m, resolution) + 1
    return cv2.blur(filled, (k, k), borderType=cv2.BORDER_REPLICATE)


def box_mean(elevation: np.ndarray, radius_m: float,
             resolution: float) -> np.ndarray:
    """Mean over a square window, edges replicated.

    A square window instead of the old exact circle keeps the cost O(N)
    (``cv2.blur`` is separable).  With radius 0.15 m at 0.10 m/cell the
    kernel is 3x3 -- exactly the neighbourhood the old circle contained.
    """
    return _box_mean_filled(fill_invalid(elevation), radius_m, resolution)


def surface_normals(
    elevation: np.ndarray, resolution: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Unit surface normals with a positive z component.

    The gradient along axis 0 is dz/dx (row is x), axis 1 is dz/dy.

    The rasterizer puts +x at DECREASING row (``row = (origin + half - x) /
    resolution``), so ``np.gradient(axis=0)`` is -dz/dx.  The outward normal
    is (-dz/dx, -dz/dy, 1) / |...|, which in row/col gradients is
    (+d/drow, +d/dcol, 1).  Getting this sign wrong flips nx/ny by 180 deg
    in the horizontal plane while leaving slope intact -- invisible in the
    slope numbers, visible in RViz and in any consumer of the normals.
    """
    arr = np.asarray(elevation, dtype=np.float64)
    dz_drow = np.gradient(arr, axis=0) / resolution
    dz_dcol = np.gradient(arr, axis=1) / resolution
    nx = dz_drow
    ny = dz_dcol
    nz = np.ones_like(arr)
    norm = np.sqrt(nx * nx + ny * ny + nz * nz)
    return nx / norm, ny / norm, nz / norm


def surface_normals_window(
    elevation: np.ndarray, resolution: float, half_cells: int
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Least-squares plane normal over a (2k+1)^2 window.

    A central difference compares two cells, so one bad cell (a crest's
    occlusion shadow bridged by the nearest-neighbour fill, a single sparse
    median) sets the whole slope. Fitting z = a*x + b*y + c over a window
    averages that out: a stage of 0.4 m inside a 0.5 m window reads ~39 deg
    instead of 75 deg, and a genuine wall (many cells) still reads steep.

    All quantities come from cv2.boxFilter, so the cost is O(N) regardless of
    the window size. Rows run along -x (the rasterizer's convention), which
    only flips the sign of ``a`` and leaves the slope invariant.
    """
    z = np.asarray(elevation, dtype=np.float64)
    k = 2 * int(half_cells) + 1
    rows = np.arange(z.shape[0], dtype=np.float64)[:, None] * resolution
    cols = np.arange(z.shape[1], dtype=np.float64)[None, :] * resolution
    xx = np.broadcast_to(rows, z.shape)
    yy = np.broadcast_to(cols, z.shape)

    def box(a):
        return cv2.boxFilter(a, -1, (k, k), normalize=True,
                             borderType=cv2.BORDER_REPLICATE)

    mz = box(z)
    mx = box(xx)
    my = box(yy)
    var_x = box(xx * xx) - mx * mx
    var_y = box(yy * yy) - my * my
    cov_xz = box(xx * z) - mx * mz
    cov_yz = box(yy * z) - my * mz
    # A constant-in-a-direction surface gives var == 0; keep the fit finite.
    a = cov_xz / np.maximum(var_x, 1e-12)
    b = cov_yz / np.maximum(var_y, 1e-12)
    nx = -a
    ny = -b
    nz = np.ones_like(z)
    norm = np.sqrt(nx * nx + ny * ny + nz * nz)
    return nx / norm, ny / norm, nz / norm


def compute_layers(elevation: np.ndarray, resolution: float,
                   mean_radius_m: float = 0.15,
                   publish_debug: bool = False,
                   slope_smooth_cells: int = 1,
                   slope_fit_cells: int = 0,
                   fill_max_cells: int = 3) -> dict[str, np.ndarray]:
    """Derive the layers the traversability bridge and RViz consume.

    Always returns ``elevation`` (untouched, NaN where unobserved),
    ``elevation_inpainted``, ``slope`` and ``roughness``.  With
    ``publish_debug`` it also returns ``elevation_smooth`` and
    ``normal_vectors_x/y/z``.

    ``elevation_inpainted`` is always published because the bridge's
    small-cluster cleanup only runs when that layer is present
    (``bridge_processor.py``); hiding it behind the debug flag would
    silently disable the cleanup and change the LETHAL output.
    """
    arr = np.asarray(elevation, dtype=np.float64)
    invalid = ~np.isfinite(arr)

    # cv2 box filters propagate NaN across the whole image, so the surface the
    # gradients are computed on must stay hole-free: fill everything for the
    # maths, then mask the holes that are too wide to trust.
    filled = fill_invalid(arr)
    too_wide = ~np.isfinite(fill_invalid(arr, fill_max_cells))
    inpainted = np.where(too_wide, np.nan, filled)
    # Already filled -- do not pay for a second distance transform.
    smooth = _box_mean_filled(filled, mean_radius_m, resolution)
    # Slope comes from the surface's gradient. A 1-cell, 0.4 m step -- what a
    # convex crest's occlusion shadow leaves behind when the nearest-neighbour
    # fill bridges it -- reads as 70-75 deg. Smoothing the surface first turns
    # that single step into a ramp (a 3-cell box roughly halves it) while a
    # real wall spans several cells and stays steep. 1 = no smoothing.
    slope_source = filled
    if slope_smooth_cells > 1:
        k = 2 * int(slope_smooth_cells) + 1
        slope_source = cv2.blur(filled, (k, k),
                                borderType=cv2.BORDER_REPLICATE)
    if slope_fit_cells > 0:
        nx, ny, nz = surface_normals_window(slope_source, resolution,
                                            slope_fit_cells)
    else:
        nx, ny, nz = surface_normals(slope_source, resolution)

    slope = np.arccos(np.clip(nz, -1.0, 1.0))
    roughness = np.abs(filled - smooth)

    layers = {
        "elevation": arr.copy(),
        "elevation_inpainted": inpainted,
        "slope": slope,
        "roughness": roughness,
    }
    if publish_debug:
        layers["elevation_smooth"] = smooth
        layers["normal_vectors_x"] = nx
        layers["normal_vectors_y"] = ny
        layers["normal_vectors_z"] = nz

    # Cells that were never observed -- or that sit in a hole too wide to
    # fill -- must not carry a slope / roughness interpolated from far away.
    invalid = too_wide | invalid
    if invalid.any():
        for key in ("slope", "roughness", "elevation_smooth",
                    "normal_vectors_x", "normal_vectors_y",
                    "normal_vectors_z"):
            if key in layers:
                layers[key][invalid] = np.nan

    return layers
