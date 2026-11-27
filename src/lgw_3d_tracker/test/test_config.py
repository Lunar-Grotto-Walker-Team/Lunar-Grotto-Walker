"""Tracker configuration defaults (2026-11-17)."""
from lgw_3d_tracker.unified_track.config import UnifiedTrackConfig


def test_new_defaults():
    c = UnifiedTrackConfig()
    assert c.candidate_min_hits == 3
    assert c.candidate_ttl_ms == 2000
    assert c.active_ttl_ms == 20000        # 2026-11-19: 15 s -> 20 s
    assert c.selected_max_unseen_ms == 60000
    assert c.consumed_ttl_ms == 60000
    assert c.association_distance_gate_m == 0.15
    assert c.predict_freeze_after_ms == 1000
    assert c.selection_fresh_ms == 1000
    assert c.arm_window_ms == 3000
    assert c.select_min_distance_m == 0.20
    assert c.select_max_distance_m == 0.0
    assert c.select_class_filter == ""
    assert c.consumed_suppress_enable is True
    assert c.observation_max_age_ms == 500
    assert c.publish_heartbeat_hz == 25
    assert c.source_priority == ["arm", "head"]
    assert c.max_registry_size == 100


def test_legacy_keys_are_ignored(tmp_path):
    p = tmp_path / "runtime.yaml"
    p.write_text("unified_track:\n  lost_ttl_ms: 999\n  reid_distance_m: 0.3\n")
    c = UnifiedTrackConfig.from_yaml(str(p))
    assert c.active_ttl_ms == 20000


def test_yaml_overrides(tmp_path):
    p = tmp_path / "runtime.yaml"
    p.write_text(
        "unified_track:\n"
        "  active_ttl_ms: 45000\n"
        "  select_class_filter: wood_block\n"
        "  select_min_distance_m: 0.25\n")
    c = UnifiedTrackConfig.from_yaml(str(p))
    assert c.active_ttl_ms == 45000
    assert c.select_class_filter == "wood_block"
    assert c.select_min_distance_m == 0.25
