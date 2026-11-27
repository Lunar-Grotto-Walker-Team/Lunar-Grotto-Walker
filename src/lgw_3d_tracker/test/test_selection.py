"""Pure target selection: XoY distance to the arm base (2026-11-17)."""
import numpy as np

from lgw_3d_tracker.unified_track.kf import KalmanState3D
from lgw_3d_tracker.unified_track.selection import SelectionParams, select_target
from lgw_3d_tracker.unified_track.track import Track, TrackState


def make_track(tid, xyz, cls="wood_block", state=TrackState.ACTIVE, age_ms=0):
    t = Track(id=tid, created_ms=0, first_seen_ms=0, last_seen_ms=0,
              task_state=state, hits=3,
              state=KalmanState3D(x=np.array(xyz, dtype=float), P=np.eye(3) * 0.05))
    t.time_since_last_obs_ms = age_ms
    return t


BASE = np.array([0.0, 0.0, 0.3])
P = SelectionParams()


def test_picks_nearest_by_xoy_distance():
    tracks = [make_track(1, (1.0, 0.0, 0.0)), make_track(2, (0.0, 0.5, 2.0))]
    assert select_target(tracks, 0, BASE, P) == 2   # XoY 决定，z 不参与


def test_ignores_stale_tracks():
    tracks = [make_track(1, (0.5, 0.0, 0.0), age_ms=1500)]
    assert select_target(tracks, 2000, BASE, P) == 0


def test_respects_class_filter():
    tracks = [make_track(1, (0.5, 0.0, 0.0), cls="red_block")]
    assert select_target(tracks, 0, BASE, SelectionParams(class_filter="wood_block")) == 0


def test_respects_distance_band():
    tracks = [make_track(1, (0.10, 0.0, 0.0)),
              make_track(2, (2.0, 0.0, 0.0))]
    assert select_target(tracks, 0, BASE, SelectionParams(min_distance_m=0.20)) == 2
    assert select_target(tracks, 0, BASE,
                         SelectionParams(min_distance_m=0.20, max_distance_m=1.0)) == 0


def test_ignores_non_active_and_no_base_pose():
    tracks = [make_track(1, (0.5, 0.0, 0.0), state=TrackState.CANDIDATE)]
    assert select_target(tracks, 0, BASE, P) == 0
    assert select_target([make_track(2, (0.5, 0.0, 0.0))], 0, None, P) == 0
