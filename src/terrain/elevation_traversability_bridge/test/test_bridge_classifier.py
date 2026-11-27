"""Tests for elevation_traversability_bridge.bridge_classifier."""
import math

import pytest

from elevation_traversability_bridge.bridge_classifier import (
    classify_cell,
    ThresholdConfig,
)


def _slope_rad(deg: float) -> float:
    return math.radians(deg)


def test_flat_ground_is_free():
    cfg = ThresholdConfig()
    assert classify_cell(slope_rad=0.0, roughness_m=0.0, cfg=cfg) == 0


def test_steep_slope_above_35_deg_is_lethal():
    cfg = ThresholdConfig()
    assert classify_cell(slope_rad=_slope_rad(35.001), roughness_m=0.0, cfg=cfg) == 254


def test_critical_slope_30_to_35_deg_is_soft():
    cfg = ThresholdConfig()
    assert classify_cell(slope_rad=_slope_rad(32.0), roughness_m=0.0, cfg=cfg) == 128


def test_smooth_ground_under_5cm_is_free():
    cfg = ThresholdConfig()
    assert classify_cell(slope_rad=0.0, roughness_m=0.05, cfg=cfg) == 0


def test_rough_ground_15cm_is_lethal():
    cfg = ThresholdConfig()
    assert classify_cell(slope_rad=0.0, roughness_m=0.1501, cfg=cfg) == 254


def test_moderate_roughness_10cm_is_soft():
    cfg = ThresholdConfig()
    assert classify_cell(slope_rad=0.0, roughness_m=0.10, cfg=cfg) == 128


def test_max_wins_between_dimensions():
    # 陡坡(soft)+ 平地 roughness = max(soft=128, free=0) = 128
    cfg = ThresholdConfig()
    assert classify_cell(slope_rad=_slope_rad(32.0), roughness_m=0.0, cfg=cfg) == 128
    # 平滑斜 + 粗糙 roughness = max(free=0, soft=128) = 128
    assert classify_cell(slope_rad=0.0, roughness_m=0.10, cfg=cfg) == 128
    # 陡坡 + 粗糙 = max(lethal=254, soft=128) = 254
    assert classify_cell(slope_rad=_slope_rad(40.0), roughness_m=0.20, cfg=cfg) == 254


def test_custom_thresholds_override_defaults():
    cfg = ThresholdConfig(
        slope_soft_upper_deg=20.0,
        slope_lethal_upper_deg=25.0,
        roughness_soft_upper_m=0.02,
        roughness_lethal_upper_m=0.05,
    )
    # 25° 自定义阈值 → LETHAL
    assert classify_cell(slope_rad=_slope_rad(25.001), roughness_m=0.0, cfg=cfg) == 254
    # 10° 自定义阈值 → FREE
    assert classify_cell(slope_rad=_slope_rad(10.0), roughness_m=0.0, cfg=cfg) == 0

def test_height_soft_must_not_exceed_lethal():
    with pytest.raises(ValueError):
        ThresholdConfig(height_soft_m=0.30, height_lethal_m=0.20)


def test_negative_height_soft_is_rejected():
    with pytest.raises(ValueError):
        ThresholdConfig(height_soft_m=-0.01)


def test_height_min_neighbors_must_be_positive():
    with pytest.raises(ValueError):
        ThresholdConfig(height_min_neighbors=0)
