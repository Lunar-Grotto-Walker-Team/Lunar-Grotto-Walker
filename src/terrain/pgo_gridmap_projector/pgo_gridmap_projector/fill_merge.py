"""Pure fill-only merge for the projector.

The uLidar fill cloud is appended to the Livox map only in xy cells that have
no Livox point.  This is decided from the raw Livox cloud, never from the
previous elevation grid, so a filled cell cannot flip back to unknown and no
feedback oscillation is possible.
"""
from __future__ import annotations

import numpy as np


def _in_z_gate(xyz: np.ndarray, z_min: float, z_max: float) -> np.ndarray:
    return (xyz[:, 2] >= z_min) & (xyz[:, 2] <= z_max)


def _cell_keys(xyz: np.ndarray, cell_size: float) -> np.ndarray:
    ix = np.floor(xyz[:, 0] / cell_size).astype(np.int64)
    iy = np.floor(xyz[:, 1] / cell_size).astype(np.int64)
    offset = 1 << 20
    return ((ix + offset) | ((iy + offset) << 21))


def merge_fill_points(livox_xyz: np.ndarray, fill_xyz: np.ndarray,
                      fill_stamp_s: float | None, now_s: float,
                      max_age_s: float, cell_size: float,
                      z_min: float, z_max: float
                      ) -> tuple[np.ndarray, int, int]:
    """Return ``(merged, kept, dropped)``.

    ``fill_xyz`` is dropped entirely when it is empty/stale or when a fill
    point lands in a Livox-occupied xy cell or outside the z gate.  The
    Livox cloud itself is never modified.
    """
    if fill_xyz is None or fill_xyz.shape[0] == 0:
        return livox_xyz, 0, 0
    if fill_stamp_s is None or (now_s - float(fill_stamp_s)) > max_age_s:
        return livox_xyz, 0, 0
    if cell_size <= 0.0:
        raise ValueError("cell_size must be positive")

    livox_g = livox_xyz[_in_z_gate(livox_xyz, z_min, z_max)]
    fill_g = fill_xyz[_in_z_gate(fill_xyz, z_min, z_max)]
    if fill_g.shape[0] == 0:
        return livox_xyz, 0, int(fill_xyz.shape[0])

    livox_keys = np.unique(_cell_keys(livox_g, cell_size))
    fill_keys = _cell_keys(fill_g, cell_size)
    keep = ~np.isin(fill_keys, livox_keys)
    kept = int(np.count_nonzero(keep))
    dropped = int(fill_xyz.shape[0]) - kept
    if kept == 0:
        return livox_xyz, 0, dropped
    merged = np.vstack([livox_xyz, fill_g[keep]])
    return merged, kept, dropped
