"""tracker 臂基座几何透传测试(servo takeover, Task 9)。

覆盖:
  - head 观测的 position_arm_base 只走 TF 安装桥,不受 odom TF 影响
  - arm 观测直接查 arm base ← camera_arm(桥已合成 tcp 链)
  - head / arm 链互相独立(查错链会失败)
  - position_arm_base 与 last_observation_stamp 来自**同一条**最新观测
    (contributor_* 保留每相机自己的 stamp)
  - position(odom KF)语义不变
  - TF 链缺失时 position_arm_base=None,formatter 跳过该相机的贡献项
  - 未知相机名在 contributor_cameras 里原样透传

跑法(容器内):
  source /my_ws/l4t/l4t_setup.sh
  source /my_ws/l4t/lg_walker_ws/install/setup.bash
  python3 -m pytest src/lgw_3d_tracker/test/test_arm_base_geometry.py -q
"""
import numpy as np
import pytest
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Point, TransformStamped, Quaternion, Vector3
from yoloe_interfaces.msg import ObservationResult, YoloeDetection3D

from lgw_3d_tracker.unified_track.calibration import (
    _matrix_to_quat, make_transform_matrix,
)
from lgw_3d_tracker.unified_track.input_adapter import (
    ARM_CAM_FRAME, HEAD_CAM_FRAME, InputAdapter,
)
from lgw_3d_tracker.unified_track.output_formatter import OutputFormatter
from lgw_3d_tracker.unified_track.track import Track, TrackObservation

IDENTITY_QUAT = (0.0, 0.0, 0.0, 1.0)
YAW90_QUAT = (0.0, 0.0, float(np.sqrt(0.5)), float(np.sqrt(0.5)))

ARM_BASE_FRAME = "arm1/base_link"
ARM_TCP_FRAME = "arm1/tcp_link"


# ----------------------------------------------------------------------
# 测试替身
# ----------------------------------------------------------------------
class FakeTfBuffer:
    """只按 (target, source) 返回预置矩阵的 tf buffer 替身。"""

    def __init__(self, transforms):
        self._transforms = dict(transforms)
        self.lookups = []

    def can_transform(self, target, source, time, timeout=None):
        return (target, source) in self._transforms

    def lookup_transform(self, target, source, time=None, timeout=None):
        self.lookups.append((target, source))
        M = self._transforms[(target, source)]
        msg = TransformStamped()
        msg.transform.translation = Vector3(
            x=float(M[0, 3]), y=float(M[1, 3]), z=float(M[2, 3]))
        q = _matrix_to_quat(M[:3, :3])
        msg.transform.rotation = Quaternion(
            x=q[0], y=q[1], z=q[2], w=q[3])
        return msg


class FakeLogger:
    def debug(self, *args, **kwargs):
        pass

    def warn(self, *args, **kwargs):
        pass

    def error(self, *args, **kwargs):
        pass


def _hom(p):
    return np.array([float(p[0]), float(p[1]), float(p[2]), 1.0])


def _make_msg(frame, center=(0.1, 0.2, 0.3), stamp=None, conf=0.9):
    msg = ObservationResult()
    msg.header.frame_id = frame
    msg.header.stamp = stamp if stamp is not None else TimeMsg(
        sec=1000, nanosec=0)
    det = YoloeDetection3D()
    det.position_valid = True
    det.class_name = "box"
    det.confidence = conf
    det.center_m = Point(x=center[0], y=center[1], z=center[2])
    msg.detections = [det]
    return msg


def _dummy_kf(state, z):
    """不更新 KF 的替身,便于单独验证 position 透传。"""
    return state


def _obs(cam, stamp_ms, arm_base, conf=0.8, odom=(0.0, 0.0, 0.0)):
    return TrackObservation(
        class_name="box", confidence=conf, source_camera=cam,
        stamp_ms=stamp_ms,
        position_odom=np.array(odom, dtype=float),
        position_arm_base=(None if arm_base is None
                           else np.array(arm_base, dtype=float)),
    )


def _stamp_ms(t):
    return int(t.sec) * 1000 + int(t.nanosec) // 1_000_000


# ----------------------------------------------------------------------
# 1. head:纯静态标定,不受 odom 影响
# ----------------------------------------------------------------------
def test_head_observation_in_arm_base_ignores_odom():
    p_cam = np.array([0.1, 0.2, 0.3])
    # 节点启动时按 head 标定发布的安装桥:arm1/base_link → head optical
    T_cam_armbase = make_transform_matrix((0.2, 0.0, 0.5), YAW90_QUAT)
    expected = (T_cam_armbase @ _hom(p_cam))[:3]

    def _adapt_with_odom(x_offset):
        # odom←base_link 的平移故意取极端值,若 head 的臂基座坐标被它污染,
        # 两个 offset 下会得到不同结果。
        T_base_odom = make_transform_matrix(
            (x_offset, 7.0, -3.0), YAW90_QUAT)
        tf_buffer = FakeTfBuffer({
            ("odom", "base_link"): T_base_odom,
            ("base_link", ARM_BASE_FRAME): make_transform_matrix(
                (0.05, 0.0, 0.1), IDENTITY_QUAT),
            (ARM_BASE_FRAME, HEAD_CAM_FRAME): T_cam_armbase,
        })
        adapter = InputAdapter(
            tf_buffer, FakeLogger(),
            arm_base_frame=ARM_BASE_FRAME,
            arm_tcp_frame=ARM_TCP_FRAME,
        )
        return adapter.adapt(_make_msg(HEAD_CAM_FRAME, center=p_cam))[0]

    obs_a = _adapt_with_odom(0.0)
    obs_b = _adapt_with_odom(100.0)

    np.testing.assert_allclose(obs_a.position_arm_base, expected, atol=1e-9)
    np.testing.assert_allclose(obs_b.position_arm_base, expected, atol=1e-9)
    # odom 位置确实随 TF 改变 → 证明"不受 odom 影响"不是因为没算 odom
    assert not np.allclose(obs_a.position_odom, obs_b.position_odom)


# ----------------------------------------------------------------------
# 2. arm:单条 TF lookup(桥已合成 T_tcp_armbase @ T_cam_tcp)
# ----------------------------------------------------------------------
def test_arm_observation_uses_tcp_chain():
    p_cam = np.array([0.1, 0.2, 0.3])
    T_cam_tcp = make_transform_matrix((-0.02, 0.01, 0.05), YAW90_QUAT)
    T_tcp_armbase = make_transform_matrix((0.3, 0.1, 0.4), IDENTITY_QUAT)
    # 节点发布的 arm 桥数值 = T_tcp_armbase @ T_cam_tcp
    T_cam_armbase = T_tcp_armbase @ T_cam_tcp
    expected = (T_cam_armbase @ _hom(p_cam))[:3]

    tf_buffer = FakeTfBuffer({
        ("odom", "base_link"): make_transform_matrix(
            (1.0, 2.0, 3.0), IDENTITY_QUAT),
        ("base_link", ARM_TCP_FRAME): make_transform_matrix(
            (0.1, 0.0, 0.2), IDENTITY_QUAT),
        (ARM_TCP_FRAME, ARM_CAM_FRAME): T_cam_tcp,
        (ARM_BASE_FRAME, ARM_CAM_FRAME): T_cam_armbase,
    })
    adapter = InputAdapter(
        tf_buffer, FakeLogger(),
        arm_base_frame=ARM_BASE_FRAME,
        arm_tcp_frame=ARM_TCP_FRAME,
    )

    obs = adapter.adapt(_make_msg(ARM_CAM_FRAME, center=p_cam))[0]

    np.testing.assert_allclose(obs.position_arm_base, expected, atol=1e-9)
    # 臂基座几何必须直接查 arm base ← camera_arm(而不是 tcp 段再乘矩阵)
    assert (ARM_BASE_FRAME, ARM_CAM_FRAME) in tf_buffer.lookups


def test_head_and_arm_chains_are_not_conflated():
    """head / arm 的臂基座链故意取不同数值 → 查错链会立刻失败。"""
    p_cam = np.array([0.1, 0.2, 0.3])
    T_head_chain = make_transform_matrix((0.2, 0.0, 0.5), IDENTITY_QUAT)
    T_arm_chain = make_transform_matrix((-0.4, 0.3, 0.1), IDENTITY_QUAT)

    tf_buffer = FakeTfBuffer({
        ("odom", "base_link"): np.eye(4),
        ("base_link", ARM_BASE_FRAME): np.eye(4),
        ("base_link", ARM_TCP_FRAME): np.eye(4),
        (ARM_BASE_FRAME, HEAD_CAM_FRAME): T_head_chain,
        (ARM_TCP_FRAME, ARM_CAM_FRAME): make_transform_matrix(
            (0.1, 0.2, 0.3), IDENTITY_QUAT),
        (ARM_BASE_FRAME, ARM_CAM_FRAME): T_arm_chain,
    })
    adapter = InputAdapter(
        tf_buffer, FakeLogger(),
        arm_base_frame=ARM_BASE_FRAME,
        arm_tcp_frame=ARM_TCP_FRAME,
    )

    head_obs = adapter.adapt(_make_msg(HEAD_CAM_FRAME, center=p_cam))[0]
    arm_obs = adapter.adapt(_make_msg(ARM_CAM_FRAME, center=p_cam))[0]

    np.testing.assert_allclose(
        head_obs.position_arm_base, (T_head_chain @ _hom(p_cam))[:3], atol=1e-9)
    np.testing.assert_allclose(
        arm_obs.position_arm_base, (T_arm_chain @ _hom(p_cam))[:3], atol=1e-9)
    assert not np.allclose(
        head_obs.position_arm_base, arm_obs.position_arm_base)


# ----------------------------------------------------------------------
# 3. contributor_* 保留每相机自己的 stamp
# ----------------------------------------------------------------------
def test_contributors_preserve_per_camera_stamps():
    track = Track(id=7, created_ms=0)
    track.apply_observation(
        _obs(HEAD_CAM_FRAME, 1000, (0.1, 0.2, 0.3), conf=0.9), _dummy_kf)
    track.apply_observation(
        _obs(ARM_CAM_FRAME, 1250, (0.4, 0.5, 0.6), conf=0.7), _dummy_kf)

    formatter = OutputFormatter(
        frame_id="odom", arm_base_frame=ARM_BASE_FRAME)
    msg = formatter.format(
        [track], sequence_id=1, frame_stamp_ms=1300,
        now=TimeMsg(sec=10, nanosec=0))
    m = msg.tracks[0]

    # 按短相机名排序,保证稳定
    assert list(m.contributor_cameras) == ["arm", "head"]
    assert [_stamp_ms(t) for t in m.contributor_stamps] == [1250, 1000]
    np.testing.assert_allclose(
        [[p.x, p.y, p.z] for p in m.contributor_positions_arm_base],
        [[0.4, 0.5, 0.6], [0.1, 0.2, 0.3]], atol=1e-9)
    np.testing.assert_allclose(
        list(m.contributor_confidences), [0.7, 0.9], atol=1e-6)

    assert m.geometry_frame == ARM_BASE_FRAME
    # 最新观测 = arm(1250),不是最后 apply 的那条之外的旧值
    assert _stamp_ms(m.last_observation_stamp) == 1250
    np.testing.assert_allclose(
        [m.position_arm_base.x, m.position_arm_base.y, m.position_arm_base.z],
        [0.4, 0.5, 0.6], atol=1e-9)

    # 既有字段语义不变:source_cameras 仍是 frame_id 字符串
    assert list(m.source_cameras) == [HEAD_CAM_FRAME, ARM_CAM_FRAME]


def test_latest_stamp_wins_even_when_applied_out_of_order():
    """arm 观测更旧但后 apply → 仍以 stamp 最新的 head 为准。"""
    track = Track(id=8, created_ms=0)
    track.apply_observation(
        _obs(ARM_CAM_FRAME, 900, (0.4, 0.5, 0.6)), _dummy_kf)
    track.apply_observation(
        _obs(HEAD_CAM_FRAME, 1100, (0.1, 0.2, 0.3)), _dummy_kf)

    m = OutputFormatter("odom", arm_base_frame=ARM_BASE_FRAME).format(
        [track], 1, 1200, TimeMsg(sec=10)).tracks[0]

    assert _stamp_ms(m.last_observation_stamp) == 1100
    np.testing.assert_allclose(
        [m.position_arm_base.x, m.position_arm_base.y, m.position_arm_base.z],
        [0.1, 0.2, 0.3], atol=1e-9)


# ----------------------------------------------------------------------
# 4. position(odom KF)语义不变
# ----------------------------------------------------------------------
def test_odom_position_still_published():
    track = Track(id=1, created_ms=0)
    track.state.x = np.array([1.0, 2.0, 3.0])
    track.apply_observation(
        _obs(HEAD_CAM_FRAME, 1000, (0.5, 0.0, 0.3),
             odom=(9.0, 9.0, 9.0)), _dummy_kf)

    m = OutputFormatter("odom", arm_base_frame=ARM_BASE_FRAME).format(
        [track], 1, 1000, TimeMsg(sec=10)).tracks[0]

    assert (m.position.x, m.position.y, m.position.z) == (1.0, 2.0, 3.0)
    np.testing.assert_allclose(
        [m.position_arm_base.x, m.position_arm_base.y, m.position_arm_base.z],
        [0.5, 0.0, 0.3], atol=1e-9)


# ----------------------------------------------------------------------
# 5. 标定/TF 缺失:None 不进入贡献项
# ----------------------------------------------------------------------
def test_none_arm_base_is_skipped_in_contributors():
    track = Track(id=3, created_ms=0)
    track.apply_observation(
        _obs(HEAD_CAM_FRAME, 1000, None), _dummy_kf)   # head 链缺失
    track.apply_observation(
        _obs(ARM_CAM_FRAME, 1100, (0.4, 0.5, 0.6)), _dummy_kf)

    m = OutputFormatter("odom", arm_base_frame=ARM_BASE_FRAME).format(
        [track], 1, 1200, TimeMsg(sec=10)).tracks[0]

    assert list(m.contributor_cameras) == ["arm"]
    assert len(m.contributor_stamps) == 1
    assert len(m.contributor_confidences) == 1
    np.testing.assert_allclose(
        [m.position_arm_base.x, m.position_arm_base.y, m.position_arm_base.z],
        [0.4, 0.5, 0.6], atol=1e-9)


def test_newest_observation_without_geometry_does_not_republish_older():
    """R-M4/newest-only:最新观测没有几何 → 主字段保持默认,不回退旧坐标。"""
    track = Track(id=9, created_ms=0)
    track.apply_observation(
        _obs(HEAD_CAM_FRAME, 900, (0.1, 0.2, 0.3)), _dummy_kf)
    track.apply_observation(
        _obs(ARM_CAM_FRAME, 1200, None), _dummy_kf)

    m = OutputFormatter("odom", arm_base_frame=ARM_BASE_FRAME).format(
        [track], 1, 1300, TimeMsg(sec=10)).tracks[0]

    # stamp 与几何必须描述同一条观测:最新那条没有几何 → 主几何不发布
    assert _stamp_ms(m.last_observation_stamp) == 1200
    assert (m.position_arm_base.x, m.position_arm_base.y,
            m.position_arm_base.z) == (0.0, 0.0, 0.0)
    # 每相机贡献仍按各自最新:head 有几何 → 保留;arm 无 → 跳过
    assert list(m.contributor_cameras) == ["head"]


def test_unknown_camera_frame_passes_through_unchanged():
    unknown = "camera_other_optical_frame"
    track = Track(id=4, created_ms=0)
    track.apply_observation(
        _obs(unknown, 1000, (0.1, 0.2, 0.3)), _dummy_kf)

    m = OutputFormatter("odom", arm_base_frame=ARM_BASE_FRAME).format(
        [track], 1, 1100, TimeMsg(sec=10)).tracks[0]

    assert list(m.contributor_cameras) == [unknown]


def test_newly_created_track_exposes_geometry_before_apply():
    """tracker 建新 track 时直接传 last_obs(不走 apply_observation)。"""
    obs = _obs(ARM_CAM_FRAME, 500, (0.2, 0.3, 0.4))
    track = Track(id=5, created_ms=0, last_obs=obs)

    m = OutputFormatter("odom", arm_base_frame=ARM_BASE_FRAME).format(
        [track], 1, 600, TimeMsg(sec=10)).tracks[0]

    assert list(m.contributor_cameras) == ["arm"]
    assert _stamp_ms(m.contributor_stamps[0]) == 500
    np.testing.assert_allclose(
        [m.position_arm_base.x, m.position_arm_base.y, m.position_arm_base.z],
        [0.2, 0.3, 0.4], atol=1e-9)


def test_missing_arm_base_tf_yields_none_position_arm_base():
    """odom 链可用、但臂基座链缺失 → position_arm_base=None。"""
    tf_buffer = FakeTfBuffer({
        ("odom", "base_link"): np.eye(4),
        ("base_link", ARM_TCP_FRAME): np.eye(4),
        (ARM_TCP_FRAME, ARM_CAM_FRAME): np.eye(4),
        # 故意不提供 (ARM_BASE_FRAME, ARM_CAM_FRAME)
    })
    adapter = InputAdapter(
        tf_buffer, FakeLogger(),
        arm_base_frame=ARM_BASE_FRAME,
        arm_tcp_frame=ARM_TCP_FRAME,
    )

    obs = adapter.adapt(_make_msg(ARM_CAM_FRAME))[0]

    assert obs.position_arm_base is None
    # odom 路径仍然可用,position_odom 照旧
    np.testing.assert_allclose(obs.position_odom, [0.1, 0.2, 0.3], atol=1e-9)
