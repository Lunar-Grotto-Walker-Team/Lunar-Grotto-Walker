"""Tracker lifecycle after lost retirement (2026-11-17)."""
import numpy as np

from lgw_3d_tracker.unified_track.track import TrackObservation, TrackState
from lgw_3d_tracker.unified_track.tracker import Tracker
from lgw_3d_tracker.unified_track.kf import KalmanFilter3D
from lgw_3d_tracker.unified_track.cost_matrix import TwoStageAssigner


def make_tracker(**kw):
    params = dict(candidate_min_hits=3, candidate_min_age_ms=0, candidate_ttl_ms=2000,
                  active_ttl_ms=15000, consumed_ttl_ms=60000, distance_gate_m=0.15,
                  duplicate_track_distance_m=0.04, max_registry_size=100,
                  consumed_suppress_enable=True, consumed_suppress_radius_m=0.15,
                  consumed_suppress_ttl_ms=10000, arm_window_ms=3000,
                  predict_freeze_after_ms=1000)
    params.update(kw)
    return Tracker(kf=KalmanFilter3D(sigma_p=0.05, sigma_obs=0.03),
                   assigner=TwoStageAssigner(0.45, 0.30, lambda s, z: True), **params)


def obs(x, cls="wood_block", cam="camera_head_color_optical_frame", conf=0.9):
    return TrackObservation(class_name=cls, confidence=conf, source_camera=cam,
                            stamp_ms=0, position_odom=np.array([x, 0.0, 0.0]))


def test_stale_active_match_keeps_id_and_resets_kf():
    tr = make_tracker()
    tr.set_now_ms(0)
    tr.associate_and_update([obs(0.0)])
    tr.associate_and_update([obs(0.0)])
    tr.associate_and_update([obs(0.0)])          # 3 hits -> active
    tr.tick_lifecycle(0)
    assert len(tr.active_tracks()) == 1
    track_id = tr.active_tracks()[0].id

    tr.set_now_ms(5000)
    tr.tick_lifecycle(5000)                        # 5 s 无观测，冻结
    report = tr.associate_and_update([obs(0.10)])  # 0.10 < 0.15
    assert [m["track_id"] for m in report.matches] == [track_id]
    t = tr.active_tracks()[0]
    assert np.allclose(t.state.x, [0.10, 0.0, 0.0])
    assert np.allclose(np.diag(t.state.P), [0.05, 0.05, 0.05])


def test_observation_beyond_gate_creates_candidate():
    tr = make_tracker()
    tr.set_now_ms(0)
    for _ in range(3):
        tr.associate_and_update([obs(0.0)])
    tr.tick_lifecycle(0)
    tr.set_now_ms(5000)
    report = tr.associate_and_update([obs(0.20)])  # > 0.15
    assert report.matches == []
    assert [c["obs"] for c in report.created] == [0]


def test_predict_freezes_when_stale():
    tr = make_tracker()
    tr.set_now_ms(0)
    for _ in range(3):
        tr.associate_and_update([obs(0.0)])
    tr.tick_lifecycle(0)
    t = tr.active_tracks()[0]
    p_before = t.state.P.copy()
    tr.set_now_ms(5000)
    tr.tick_lifecycle(5000)
    tr.predict_all()
    assert np.allclose(t.state.P, p_before)


def test_mark_track_consume_sets_consumed_since():
    tr = make_tracker()
    tr.set_now_ms(1000)
    for _ in range(3):
        tr.associate_and_update([obs(0.0)])
        tr.tick_lifecycle(0)
    t = tr.active_tracks()[0]
    assert tr.mark_track(t.id, 1)
    assert t.task_state == TrackState.CONSUMED
    assert t.consumed_since_ms == 1000
    assert tr.mark_track(t.id, 3) is False


def test_consumed_suppress_uses_consumed_since():
    tr = make_tracker()
    tr.set_now_ms(0)
    for _ in range(3):
        tr.associate_and_update([obs(0.0)])
        tr.tick_lifecycle(0)
    t = tr.active_tracks()[0]
    tr.set_now_ms(20000)                 # last_seen 仍是 0（20 s 前）
    assert tr.mark_track(t.id, 2)        # ignore 发生在 20000
    tr.set_now_ms(21000)                 # consumed_since 后仅 1 s
    report = tr.associate_and_update([obs(0.0)])
    assert report.created == []          # 仍被 consumed_suppress 压住
