"""Source selection (arm > head > odom) and live arm-base projection."""
from typing import List, Optional, Tuple

import numpy as np

from lgw_3d_tracker.unified_track.track import Track, TrackObservation


def short_camera_name(frame: str) -> str:
    lowered = str(frame).lower()
    if "head" in lowered:
        return "head"
    if "arm" in lowered:
        return "arm"
    return str(frame)


def choose_source(
    track: Track,
    now_ms: int,
    priority: List[str],
    max_age_ms: int,
) -> Tuple[str, Optional[TrackObservation]]:
    """Return the highest-priority fresh observation, else ("odom", None)."""
    for name in priority:
        for frame, obs in track.last_obs_by_camera.items():
            if short_camera_name(frame) != name:
                continue
            if now_ms - obs.stamp_ms <= max_age_ms:
                return name, obs
    return "odom", None


def project_odom_to_arm_base(
    p_odom: np.ndarray,
    T_base_odom: np.ndarray,
) -> np.ndarray:
    """Map an odom point into the current arm base frame.

    `T_base_odom` is lookupTransform(odom <- base_link), i.e. base -> odom;
    the inverse maps odom -> base.
    """
    p = np.append(np.asarray(p_odom, dtype=float), 1.0)
    return (np.linalg.inv(np.asarray(T_base_odom, dtype=float)) @ p)[:3]
