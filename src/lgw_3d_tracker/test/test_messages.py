"""Message contract: selected target id + arm hit window (2026-11-17)."""
from yoloe_interfaces.msg import UnifiedTrack, UnifiedTrackArray


def test_new_fields_default_to_zero():
    arr = UnifiedTrackArray()
    track = UnifiedTrack()
    assert arr.selected_track_id == 0
    assert arr.arm_window_ms == 0
    assert track.arm_hits_in_window == 0


def test_new_fields_are_settable():
    arr = UnifiedTrackArray()
    arr.selected_track_id = 7
    arr.arm_window_ms = 3000
    track = UnifiedTrack()
    track.arm_hits_in_window = 3
    assert (arr.selected_track_id, arr.arm_window_ms, track.arm_hits_in_window) == (7, 3000, 3)
