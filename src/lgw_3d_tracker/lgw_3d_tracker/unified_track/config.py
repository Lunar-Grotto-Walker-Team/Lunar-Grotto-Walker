"""YAML configuration loader for the unified tracker.

Reads the `unified_track` section of `config/runtime.yaml`.
"""
import warnings
from dataclasses import dataclass, field
from typing import List

import yaml

DEFAULT_ARM_MOUNT_OFFSET = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


@dataclass
class TwoStageConfig:
    enabled: bool = True
    high_conf_threshold: float = 0.40
    low_conf_threshold: float = 0.15


@dataclass
class UnifiedTrackConfig:
    enabled: bool = True
    head_detections_topic: str = "/perception/two_stage/camera_head/result"
    arm_detections_topic: str = "/perception/two_stage/camera_arm/result"
    sport_pose_topic: str = "/utlidar/robot_pose"
    output_topic: str = "/perception/tracks/unified"
    output_frame_id: str = "odom"
    publish_heartbeat_hz: float = 25.0

    process_noise_position: float = 0.05
    observation_noise_position: float = 0.03

    # Task-state machine (all times wall-clock ms).
    candidate_min_hits: int = 3
    candidate_min_age_ms: int = 0
    candidate_ttl_ms: int = 2000
    active_ttl_ms: int = 20000
    consumed_ttl_ms: int = 60000
    predict_freeze_after_ms: int = 1000

    # Publish-time source selection.
    observation_max_age_ms: int = 500
    source_priority: List[str] = field(
        default_factory=lambda: ["arm", "head"])

    # Registry / selection.
    association_distance_gate_m: float = 0.15
    duplicate_track_distance_m: float = 0.04
    max_registry_size: int = 100
    # 2026-11-18 用户要求的两道"建轨门限"(只丢弃观测, 不改动已有轨迹):
    #   * max_track_distance_m: 与机械臂基座的三维距离超过该值的观测直接丢弃
    #     (0 = 不限; 之前那一版限制是 1.5 m, 现在放到 5 m)
    #   * base_exclusion_radius_m: 与"基地圆心"的 XoY 距离小于该值的观测不建
    #     立轨迹。基地圆心 = 行为树/本节点**启动那一刻**机器人(机械臂基座)在
    #     odom 系下的位置快照, 不是 odom 原点(上电点会漂, 基地是任务起点)。
    #     目的: 屏蔽基地里已经采集过、堆放在那里的目标, 防止被重复识别。
    max_track_distance_m: float = 5.0
    base_exclusion_radius_m: float = 0.4
    # 2026-11-19: 被选中的轨迹豁免 active TTL(否则抓取中途 id 滚动, BT 的
    # {track_id} 变悬空)。这个值是豁免的上限: 选中后多久完全没有观测就退役。
    selected_max_unseen_ms: int = 60000
    consumed_suppress_enable: bool = True
    consumed_suppress_radius_m: float = 0.15
    consumed_suppress_ttl_ms: int = 10000
    arm_window_ms: int = 3000

    # Mission target (tracker-owned; see selection.py).
    selection_fresh_ms: int = 1000
    select_class_filter: str = ""
    select_min_distance_m: float = 0.20
    select_max_distance_m: float = 0.0

    appearance_class_must_match: bool = True
    two_stage: TwoStageConfig = field(default_factory=TwoStageConfig)

    tf_cache_sec: float = 30.0
    robot_pose_stale_ms: int = 500

    head_calibration_file: str = ""
    arm_frame_prefix: str = "arm1/"
    arm_mount_offset: List[float] = field(
        default_factory=lambda: list(DEFAULT_ARM_MOUNT_OFFSET))

    @classmethod
    def from_yaml(cls, path: str) -> "UnifiedTrackConfig":
        try:
            with open(path) as f:
                raw = yaml.safe_load(f) or {}
        except FileNotFoundError:
            return cls()
        section = raw.get("unified_track", {}) or {}
        for legacy in ("coast_max_age_ms", "coast_enter_ms",
                       "tentative_min_hits", "output_min_period_ms",
                       "visible_timeout_ms", "lost_ttl_ms",
                       "reid_distance_m", "reid_reinit_distance_m"):
            if legacy in section:
                warnings.warn(
                    f"unified_track.{legacy} is deprecated and ignored; "
                    "use active_ttl_ms / publish_heartbeat_hz",
                    DeprecationWarning, stacklevel=2)
        ts = section.get("two_stage", {}) or {}
        return cls(
            enabled=section.get("enabled", True),
            head_detections_topic=section.get(
                "head_detections_topic", cls.head_detections_topic),
            arm_detections_topic=section.get(
                "arm_detections_topic", cls.arm_detections_topic),
            sport_pose_topic=section.get(
                "sport_pose_topic", cls.sport_pose_topic),
            output_topic=section.get("output_topic", cls.output_topic),
            output_frame_id=section.get(
                "output_frame_id", cls.output_frame_id),
            publish_heartbeat_hz=float(section.get(
                "publish_heartbeat_hz", cls.publish_heartbeat_hz)),
            process_noise_position=section.get(
                "process_noise_position", cls.process_noise_position),
            observation_noise_position=section.get(
                "observation_noise_position", cls.observation_noise_position),
            candidate_min_hits=int(section.get(
                "candidate_min_hits", cls.candidate_min_hits)),
            candidate_min_age_ms=int(section.get(
                "candidate_min_age_ms", cls.candidate_min_age_ms)),
            candidate_ttl_ms=int(section.get(
                "candidate_ttl_ms", cls.candidate_ttl_ms)),
            active_ttl_ms=int(section.get(
                "active_ttl_ms", cls.active_ttl_ms)),
            consumed_ttl_ms=int(section.get(
                "consumed_ttl_ms", cls.consumed_ttl_ms)),
            predict_freeze_after_ms=int(section.get(
                "predict_freeze_after_ms", cls.predict_freeze_after_ms)),
            observation_max_age_ms=int(section.get(
                "observation_max_age_ms", cls.observation_max_age_ms)),
            source_priority=list(section.get(
                "source_priority", ["arm", "head"])),
            association_distance_gate_m=section.get(
                "association_distance_gate_m", cls.association_distance_gate_m),
            duplicate_track_distance_m=section.get(
                "duplicate_track_distance_m",
                cls.duplicate_track_distance_m),
            max_registry_size=int(section.get(
                "max_registry_size", cls.max_registry_size)),
            max_track_distance_m=float(section.get(
                "max_track_distance_m", cls.max_track_distance_m)),
            base_exclusion_radius_m=float(section.get(
                "base_exclusion_radius_m",
                cls.base_exclusion_radius_m)),
            selected_max_unseen_ms=int(section.get(
                "selected_max_unseen_ms", cls.selected_max_unseen_ms)),
            consumed_suppress_enable=bool(section.get(
                "consumed_suppress_enable", cls.consumed_suppress_enable)),
            consumed_suppress_radius_m=float(section.get(
                "consumed_suppress_radius_m", cls.consumed_suppress_radius_m)),
            consumed_suppress_ttl_ms=int(section.get(
                "consumed_suppress_ttl_ms", cls.consumed_suppress_ttl_ms)),
            arm_window_ms=int(section.get(
                "arm_window_ms", cls.arm_window_ms)),
            selection_fresh_ms=int(section.get(
                "selection_fresh_ms", cls.selection_fresh_ms)),
            select_class_filter=str(section.get(
                "select_class_filter", cls.select_class_filter)),
            select_min_distance_m=float(section.get(
                "select_min_distance_m", cls.select_min_distance_m)),
            select_max_distance_m=float(section.get(
                "select_max_distance_m", cls.select_max_distance_m)),
            appearance_class_must_match=section.get(
                "appearance_class_must_match", cls.appearance_class_must_match),
            two_stage=TwoStageConfig(
                enabled=ts.get("enabled", True),
                high_conf_threshold=ts.get(
                    "high_conf_threshold", TwoStageConfig.high_conf_threshold),
                low_conf_threshold=ts.get(
                    "low_conf_threshold", TwoStageConfig.low_conf_threshold),
            ),
            tf_cache_sec=section.get("tf_cache_sec", cls.tf_cache_sec),
            robot_pose_stale_ms=section.get(
                "robot_pose_stale_ms", cls.robot_pose_stale_ms),
            head_calibration_file=section.get(
                "head_calibration_file", cls.head_calibration_file),
            arm_frame_prefix=section.get(
                "arm_frame_prefix", cls.arm_frame_prefix),
            arm_mount_offset=list(section.get(
                "arm_mount_offset", DEFAULT_ARM_MOUNT_OFFSET)),
        )
