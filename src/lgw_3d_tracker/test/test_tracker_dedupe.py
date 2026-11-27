"""3D 影子轨迹去重回归测试。

背景:YOLOE 偶尔在同一帧输出两条同类检测(实测同一木块,bbox 呈包含关系,
3D 中心相差 ~1.8cm)。关联是 Hungarian 一对一,第二条必然匹配失败 →
新建 tentative "影子轨迹"。真机实测影子轨迹存活后会抢走主轨迹的关联,
导致 track_id 切换(2026-10-23 run1:~155s 时 id 1 → 4)。

跑法(容器内):
  source /my_ws/l4t/l4t_setup.sh
  source /my_ws/l4t/lg_walker_ws/install/setup.bash
  python3 -m pytest src/lgw_3d_tracker/test/test_tracker_dedupe.py -q
"""
import numpy as np

from lgw_3d_tracker.unified_track.cost_matrix import TwoStageAssigner
from lgw_3d_tracker.unified_track.kf import KalmanFilter3D
from lgw_3d_tracker.unified_track.track import TrackObservation
from lgw_3d_tracker.unified_track.tracker import Tracker

HEAD = "camera_head_color_optical_frame"
ARM = "camera_arm_color_optical_frame"


def _tracker(dup_dist_m: float = 0.03) -> Tracker:
    kf = KalmanFilter3D(sigma_p=0.05, sigma_obs=0.03)
    assigner = TwoStageAssigner(
        high_conf_th=0.4, low_conf_th=0.15, gate_fn=lambda s, z: True)
    return Tracker(
        kf=kf, assigner=assigner,
        candidate_min_hits=3, candidate_min_age_ms=0,
        candidate_ttl_ms=2000, active_ttl_ms=15000,
        consumed_ttl_ms=60000, distance_gate_m=0.15,
        duplicate_track_distance_m=dup_dist_m,
        max_registry_size=100, consumed_suppress_enable=True,
        arm_window_ms=3000, predict_freeze_after_ms=1000)


def _obs(cam: str, x: float, y: float = 0.0, cls: str = "wood_block",
         conf: float = 0.8) -> TrackObservation:
    return TrackObservation(
        class_name=cls, confidence=conf, source_camera=cam,
        stamp_ms=0, position_odom=np.array([x, y, 0.0], dtype=float))


def _confirm(tr: Tracker, x: float = 0.0, cam: str = ARM):
    for _ in range(3):
        tr.predict_all()
        tr.associate_and_update([_obs(cam, x)])
        tr.tick_lifecycle(100)


def test_same_tick_duplicate_observation_is_suppressed_before_creation():
    """同帧两条极近的同类检测 → 第二条在**建轨迹前**就被丢弃。"""
    tr = _tracker()
    _confirm(tr)
    assert len(tr.tracks) == 1
    main_id = tr.tracks[0].id

    tr.predict_all()
    report = tr.associate_and_update([_obs(HEAD, 0.000), _obs(HEAD, 0.018)])
    tr.tick_lifecycle(100)

    assert [t.id for t in tr.tracks] == [main_id]
    assert report.created == []
    assert report.suppressed and report.suppressed[0]["track_id"] == main_id
    # 既没有新建,也就不需要 track 级去重
    assert tr.last_dedupe_removed == []


def test_dedupe_tracks_cleans_up_tracks_that_converged():
    """两条轨迹随时间靠拢到阈值内(track 级兜底路径)。

    建轨时用极小阈值(两轨迹可以并肩存在),之后把阈值放大模拟"两轨迹收敛",
    此时只能靠 dedupe_tracks() 清理。
    """
    tr = _tracker(dup_dist_m=0.005)
    tr.distance_gate_m = 0.002
    _confirm(tr, x=0.0)
    _confirm(tr, x=0.02)
    assert len(tr.tracks) == 2

    tr.duplicate_track_distance_m = 0.04   # 模拟两轨迹收敛到 2cm
    tr.predict_all()
    tr.tick_lifecycle(100)

    assert tr.last_dedupe_removed
    removed = tr.last_dedupe_removed[0]
    assert removed["dist"] < 0.04
    assert [t.id for t in tr.tracks] == [removed["kept_id"]]


def test_unmatched_observation_near_existing_track_is_suppressed():
    """同帧第二条近邻观测不应新建轨迹(它会在同一 tick 被去重删掉,纯抖动)。

    2026-10-26 真机:YOLOE 把一个木块拆成"顶面/正面"两个实例,3D 间距
    p50 = 3.03 cm。旧实现每 tick 新建一条 tentative 再被 dedupe 删除,
    轨迹 id 在 108→132 之间反复churn。
    """
    tr = _tracker(dup_dist_m=0.04)
    _confirm(tr)
    assert len(tr.tracks) == 1

    tr.predict_all()
    report = tr.associate_and_update([_obs(HEAD, 0.000), _obs(HEAD, 0.030)])
    tr.tick_lifecycle(100)

    assert len(tr.tracks) == 1
    assert report.created == []
    assert len(report.suppressed) == 1
    assert tr.last_dedupe_removed == []


def test_dedupe_respects_distance_threshold():
    """阈值 1cm 时,1.8cm 的影子轨迹不能被删(阈值必须真正生效)。

    注:两个同类轨迹在同一帧里只能以"同帧重复检测"的形式并肩存在
    (关联门控 15cm 会把更远的观测直接吸到已有轨迹上),所以这里用阈值
    而不是坐标来验证去重的收敛范围。
    """
    tr = _tracker(dup_dist_m=0.01)
    _confirm(tr)

    tr.predict_all()
    tr.associate_and_update([_obs(HEAD, 0.000), _obs(HEAD, 0.018)])
    tr.tick_lifecycle(100)

    assert len(tr.tracks) == 2
    assert tr.last_dedupe_removed == []


def test_same_position_different_class_not_merged():
    """重叠位置但类别不同 → 两条都保留。"""
    tr = _tracker()
    _confirm(tr, x=0.0)
    for _ in range(3):
        tr.predict_all()
        tr.associate_and_update([_obs(ARM, 0.0, y=0.01, cls="red_block")])
        tr.tick_lifecycle(100)

    assert len(tr.tracks) == 2
    assert tr.last_dedupe_removed == []


def test_config_reads_duplicate_track_distance(tmp_path):
    """影子轨迹距离阈值必须可配置,默认 3cm。"""
    from lgw_3d_tracker.unified_track.config import UnifiedTrackConfig

    assert UnifiedTrackConfig().duplicate_track_distance_m == 0.04

    path = tmp_path / "runtime.yaml"
    path.write_text("unified_track:\n  duplicate_track_distance_m: 0.05\n")
    assert (UnifiedTrackConfig.from_yaml(str(path)).duplicate_track_distance_m
            == 0.05)


def test_mature_track_survives_when_duplicate_pair_has_different_hits():
    """保留更成熟(命中更多/更老)的轨迹,删掉年轻的那条。"""
    tr = _tracker()
    _confirm(tr, x=0.0)          # 主轨迹 hits=3
    main_id = tr.tracks[0].id

    tr.predict_all()
    tr.associate_and_update([_obs(ARM, 0.0), _obs(HEAD, 0.02)])
    tr.tick_lifecycle(100)

    assert [t.id for t in tr.tracks] == [main_id]
