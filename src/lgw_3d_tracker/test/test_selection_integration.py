"""Tracker-owned mission target: lock, request/release, consistency (2026-11-17)."""
import numpy as np
from test_tracker_lifecycle import make_tracker, obs


def seed_active(tr, x=0.5):
    tr.set_now_ms(0)
    for _ in range(3):
        tr.associate_and_update([obs(x)])
    tr.tick_lifecycle(0)
    return tr.active_tracks()[0]


def test_selects_when_none_and_base_pose_available():
    tr = make_tracker()
    t = seed_active(tr)
    tr.set_arm_base_position(np.zeros(3))
    tr.tick_lifecycle(0)
    assert tr.selected_track_id == t.id


def test_no_selection_without_base_pose():
    tr = make_tracker()
    seed_active(tr)
    tr.tick_lifecycle(0)
    assert tr.selected_track_id == 0


def test_lock_survives_closer_target():
    tr = make_tracker()
    t = seed_active(tr, 0.8)
    tr.set_arm_base_position(np.zeros(3))
    tr.tick_lifecycle(0)
    assert tr.selected_track_id == t.id
    for _ in range(3):
        tr.associate_and_update([obs(0.3)])
        tr.tick_lifecycle(0)
    assert tr.selected_track_id == t.id


def test_selection_cleared_on_consume():
    tr = make_tracker()
    t = seed_active(tr)
    tr.set_arm_base_position(np.zeros(3))
    tr.tick_lifecycle(0)
    assert tr.mark_track(t.id, 2)
    assert tr.selected_track_id == 0


def test_request_zero_releases_and_request_consumed_fails():
    tr = make_tracker()
    t = seed_active(tr)
    tr.set_arm_base_position(np.zeros(3))
    tr.tick_lifecycle(0)
    assert tr.mark_track(0, 4)
    assert tr.selected_track_id == 0
    tr.mark_track(t.id, 1)
    assert tr.mark_track(t.id, 4) is False


def test_selected_track_archived_clears_selection():
    """2026-11-19: 选中轨迹豁免 active TTL, 但仍受"长时间无观测"上限约束;
    一旦真的归档, selection 必须被清掉。"""
    tr = make_tracker()
    t = seed_active(tr)
    tr.set_arm_base_position(np.zeros(3))
    tr.tick_lifecycle(0)
    # active TTL 到点不再归档(选中豁免)
    tr.set_now_ms(15001)
    tr.tick_lifecycle(0)
    assert tr.selected_track_id == t.id
    # 超过"选中但仍无观测"的上限 -> 归档 -> 清 selection
    tr.set_now_ms(tr.selected_max_unseen_ms + 2)
    tr.tick_lifecycle(0)
    assert tr.selected_track_id == 0


def test_dedupe_keeps_selection_pointing_to_existing_track():
    from lgw_3d_tracker.unified_track.kf import KalmanState3D
    from lgw_3d_tracker.unified_track.track import Track, TrackState

    tr = make_tracker()
    t = seed_active(tr)
    tr.set_arm_base_position(np.zeros(3))
    tr.tick_lifecycle(0)
    # 同位置、同类的第二条轨道，迫使 dedupe 合并掉一条
    dup = Track(id=99, created_ms=-1000, first_seen_ms=-1000, last_seen_ms=0,
                task_state=TrackState.ACTIVE, hits=10,
                state=KalmanState3D(x=t.state.x.copy(), P=np.eye(3) * 0.05),
                last_obs=t.last_obs)
    tr._tracks.append(dup)
    removed = tr.dedupe_tracks()
    assert removed, "expected dedupe to remove one track"
    assert tr.selected_track_id in {x.id for x in tr.tracks}
    assert tr.selected_track_id in (t.id, 99)


def test_selection_survives_tf_loss():
    tr = make_tracker()
    t = seed_active(tr)
    tr.set_arm_base_position(np.zeros(3))
    tr.tick_lifecycle(0)
    assert tr.selected_track_id == t.id
    tr.set_arm_base_position(None)
    tr.tick_lifecycle(0)
    assert tr.selected_track_id == t.id


def test_capacity_never_evicts_selected_track():
    from lgw_3d_tracker.unified_track.kf import KalmanState3D
    from lgw_3d_tracker.unified_track.track import Track, TrackState

    tr = make_tracker(max_registry_size=1)
    t = seed_active(tr)
    tr.set_arm_base_position(np.zeros(3))
    tr.tick_lifecycle(0)
    for stale_id in (98, 99):
        stale = Track(id=stale_id, created_ms=0, first_seen_ms=0,
                      last_seen_ms=0, task_state=TrackState.ACTIVE, hits=1,
                      state=KalmanState3D(x=np.array([5.0, 0.0, 0.0]),
                                          P=np.eye(3) * 0.05))
        stale.time_since_last_obs_ms = 5000
        tr._tracks.append(stale)
    tr.tick_lifecycle(0)
    assert t.id in {x.id for x in tr.tracks}
    assert tr.selected_track_id == t.id


def test_request_valid_id_selects():
    tr = make_tracker()
    t = seed_active(tr)
    tr.set_arm_base_position(np.zeros(3))
    tr.tick_lifecycle(0)
    assert tr.mark_track(0, 4)
    assert tr.selected_track_id == 0
    assert tr.mark_track(t.id, 4)
    assert tr.selected_track_id == t.id
