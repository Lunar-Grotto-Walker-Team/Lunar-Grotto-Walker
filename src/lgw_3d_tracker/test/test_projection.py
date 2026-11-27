"""arm > head > odom source selection and live arm-base projection."""
import numpy as np

from lgw_3d_tracker.unified_track.geometry_projector import (
    choose_source, project_odom_to_arm_base)
from lgw_3d_tracker.unified_track.kf import KalmanState3D
from lgw_3d_tracker.unified_track.track import (
    Track, TrackObservation, TrackState)


def _track_with(cams):
    t = Track(
        id=1, created_ms=0, first_seen_ms=0,
        state=KalmanState3D(np.array([1.0, 2.0, 3.0]), np.eye(3)))
    t.task_state = TrackState.ACTIVE
    for cam, stamp in cams.items():
        t.last_obs_by_camera[cam] = TrackObservation(
            "sample", 0.8, cam, stamp, np.array([1.0, 2.0, 3.0]))
    return t


def test_arm_wins_when_both_fresh():
    t = _track_with({"camera_arm_color_optical_frame": 1000,
                     "camera_head_color_optical_frame": 1000})
    src, obs = choose_source(t, 1000, ["arm", "head"], 500)
    assert src == "arm"
    assert obs.source_camera == "camera_arm_color_optical_frame"


def test_head_used_when_arm_stale():
    t = _track_with({"camera_arm_color_optical_frame": 0,
                     "camera_head_color_optical_frame": 1000})
    src, obs = choose_source(t, 1200, ["arm", "head"], 500)
    assert src == "head"


def test_odom_when_nothing_fresh():
    t = _track_with({"camera_arm_color_optical_frame": 0})
    src, obs = choose_source(t, 2000, ["arm", "head"], 500)
    assert src == "odom"
    assert obs is None


def test_project_uses_current_base_pose():
    T = np.eye(4)
    T[:3, 3] = [1.0, 0.0, 0.0]       # base at (1,0,0) in odom
    p = project_odom_to_arm_base(np.array([2.0, 0.0, 0.0]), T)
    assert np.allclose(p, [1.0, 0.0, 0.0])
