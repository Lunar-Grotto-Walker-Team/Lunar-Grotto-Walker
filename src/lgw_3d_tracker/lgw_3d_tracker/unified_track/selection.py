"""Pure target selection (2026-11-17).

The tracker owns one mission target at a time.  Selection only runs when no
target is selected; once selected the tracker keeps it until the active TTL,
consume/ignore, or an explicit request.  This module is the "which one" rule:
the nearest *fresh* active track by XoY distance to the arm base, optionally
filtered by class and a distance band.

The distance is planar on purpose: navigation and arm reach both care about
the horizontal offset, and the arm base sits ~0.3 m above the floor.
"""
from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

from lgw_3d_tracker.unified_track.track import Track, TrackState


@dataclass(frozen=True)
class SelectionParams:
    class_filter: str = ""       # "" = allow every class
    min_distance_m: float = 0.20
    max_distance_m: float = 0.0  # <= 0 disables the ceiling
    fresh_ms: int = 1000


def select_target(
    tracks: Sequence[Track],
    now_ms: int,
    p_arm_base_odom: Optional[np.ndarray],
    params: SelectionParams,
) -> int:
    """Return the eligible track id with the smallest XoY distance, else 0.

    ``now_ms`` is accepted for symmetry with the tracker tick; freshness is
    read from each track's ``time_since_last_obs_ms`` so the rule stays pure.
    """
    if p_arm_base_odom is None:
        return 0

    base = np.asarray(p_arm_base_odom, dtype=float)
    best_id = 0
    best_distance = 0.0

    for t in tracks:
        if t.task_state != TrackState.ACTIVE:
            continue
        if params.class_filter:
            class_name = t.last_obs.class_name if t.last_obs is not None else ""
            if class_name != params.class_filter:
                continue
        if t.time_since_last_obs_ms > params.fresh_ms:
            continue

        distance = float(np.hypot(
            float(t.state.x[0]) - float(base[0]),
            float(t.state.x[1]) - float(base[1]),
        ))
        if distance < params.min_distance_m:
            continue
        if params.max_distance_m > 0.0 and distance > params.max_distance_m:
            continue
        if best_id == 0 or distance < best_distance or (
                distance == best_distance and t.id < best_id):
            best_id = t.id
            best_distance = distance

    return best_id
