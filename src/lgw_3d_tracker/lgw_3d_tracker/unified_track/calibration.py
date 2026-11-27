"""标定 JSON 加载 + 4x4 变换构造 + 机械臂挂载桥 TF。

标定语义(重要,别取逆):
  - head_calibration.json : T(camera_head_color_optical_frame -> **机械臂基座**)
        p_armbase = R_h2b · p_cam + t_h2b
        ⚠️ 这里的 "base_link" 是**机械臂 URDF 的 base_link**(= `<prefix>base_link`,
           与 `<prefix>world` 恒等),**不是**机器人本体的 base_link(odom→base_link)。
           佐证:head 标定是 eye-to-hand 产物,设计文档写明 "base_link → world =
           identity",而 world→base_link 的 fixed joint 属于机械臂 URDF。
  - arm_calibration.json  : T(camera_arm_color_optical_frame -> tcp_link)
        p_tcp  = R_a2t · p_cam + t_a2t
        (含 12cm tcp_offset,xyzw 四元数)

两者的方向都是"相机 → 机器人侧",与 tf2 的 edge 语义
(parent=P, child=C 表示 p_P = R·p_C + t)同构,所以**直接用,不取逆**。

Fix Round 6 (2026-10-22 真机验证):改用"分段查 + 矩阵乘"替代整链 tf2 lookup。
原因:机械臂 URDF 的 world→base_link fixed joint 与机器人本体的
odom→base_link 抢同一个 child(base_link)的 parent 位置,tf2 的 parent 解析
不确定 → "Tf has two or more unconnected trees" 随机失败(实测 25%~35%)。
现方案把链拆成:
    T_cam_odom = T_base_odom  (x)  T_cam_base          [head 相机]
    T_cam_odom = T_base_odom  (x)  T_tcp_base  (x)  T_cam_tcp   [arm 相机]
其中 T_tcp_base 的 lookup 上界是 base_link,不会越过它的 parent → 无歧义。
⚠️ Ruling R6 (2026-10-27):tracker 的 InputAdapter 已改为全部走 TF
(相机段直接查下面这些安装桥),不再在 adapter 里乘标定矩阵;本模块的
`calibration_matrix` 现在只用于 **构造安装桥**。桥与标定同源,数值一致。

Fix Round 7 (2026-10-22):机械臂侧启用 robot_state_publisher 的 frame_prefix
(arm1/),机械臂链的 frame 变成 arm1/base_link、arm1/tcp_link ... 与机器人本体
的 base_link 不再重名。本模块负责补那条把两棵树连起来的挂载桥:
    base_link -> {prefix}base_link   (= 机械臂在 Go2 躯干上的安装偏移)
"""
import json
import math
import os
from typing import Optional, Sequence, Tuple

import numpy as np
import rclpy
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
from tf2_ros.buffer import Buffer
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def _default_calib_path(filename: str) -> str:
    """从 lgw_bt_planner share 目录解析标定 JSON 路径。"""
    share = get_package_share_directory("lgw_bt_planner")
    return os.path.join(share, "calibration", filename)


def _load_calibration(path: str) -> Tuple[Tuple[float, float, float],
                                          Tuple[float, float, float, float]]:
    """读标定 JSON,返回 ((tx,ty,tz), (qx,qy,qz,qw))。

    orientation 字段是 xyzw(ROS geometry_msgs 约定,见
    handeye_calibration_ros/handeye_calibration.py:42-51)。
    """
    with open(path) as f:
        data = json.load(f)
    pos = data["position"]
    q = data["orientation"]
    return (float(pos[0]), float(pos[1]), float(pos[2])), (
        float(q[0]), float(q[1]), float(q[2]), float(q[3]),
    )


def load_head_calibration(path: Optional[str] = None) -> Tuple:
    """读 head_calibration.json → T(camera_head_color_optical_frame -> base_link)。"""
    if not path:
        path = _default_calib_path("head_calibration.json")
    return _load_calibration(path)


def load_arm_calibration(path: Optional[str] = None) -> Tuple:
    """读 arm_calibration.json(_tcp 变体) → T(camera_arm_color_optical_frame -> tcp_link)。

    注意:必须是 _tcp 变体(已叠加 12cm tcp_offset)。项目内 ground truth 是
    lgw_bt_planner/calibration/arm_calibration.json。
    """
    if not path:
        path = _default_calib_path("arm_calibration.json")
    return _load_calibration(path)


def quat_to_matrix(q: Sequence[float]) -> np.ndarray:
    """xyzw 四元数 → 3x3 旋转矩阵。"""
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        raise ValueError("quaternion norm is ~0")
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rpy_to_quat(rpy: Sequence[float]) -> Tuple[float, float, float, float]:
    """RPY(roll,pitch,yaw,弧度) → xyzw 四元数。"""
    r, p, y = (float(v) for v in rpy)
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def make_transform_matrix(translation: Sequence[float],
                          quat_xyzw: Sequence[float]) -> np.ndarray:
    """构造 4x4 齐次矩阵,把 source 系下的点变到 target 系。

    p_target = M · p_source,对应 tf2 edge 语义(parent=target, child=source)。
    """
    m = np.eye(4)
    m[:3, :3] = quat_to_matrix(quat_xyzw)
    m[:3, 3] = [float(v) for v in translation]
    return m


def calibration_matrix(kind: str,
                       path: Optional[str] = None) -> np.ndarray:
    """按名字取标定矩阵。

    Args:
      kind: "head" 或 "arm"
      path: 自定义 JSON 路径(仅 head 支持,arm 走默认 _tcp 变体)

    Returns:
      4x4 齐次矩阵
    """
    if kind == "head":
        t, q = load_head_calibration(path)
    elif kind == "arm":
        t, q = load_arm_calibration(path)
    else:
        raise ValueError(f"unknown calibration kind: {kind}")
    return make_transform_matrix(t, q)


def transform_to_matrix(tf_msg) -> np.ndarray:
    """tf2_msgs/geometry_msgs TransformStamped → 4x4 齐次矩阵。

    语义:edge (parent=P, child=C) 表示 p_P = R·p_C + t,
    所以矩阵把 **child 系下的点**变到 **parent 系**。
    """
    m = np.eye(4)
    q = tf_msg.transform.rotation
    m[:3, :3] = quat_to_matrix((q.x, q.y, q.z, q.w))
    tr = tf_msg.transform.translation
    m[:3, 3] = [tr.x, tr.y, tr.z]
    return m


def broadcast_arm_mount_tf(node: rclpy.node.Node,
                           frame_prefix: str,
                           offset_xyz: Sequence[float] = (0.0, 0.0, 0.0),
                           offset_rpy: Sequence[float] = (0.0, 0.0, 0.0)):
    """广播 base_link → {frame_prefix}world 的机械臂挂载偏移 TF。

    机械臂启用 frame_prefix(arm1/)后,它的 TF 链是
    (arm1/world → arm1/base_link → arm1/link1 → ... → arm1/tcp_link),
    与机器人本体的 odom → base_link 是两棵独立的树。必须由本节点补一条桥
    把两棵树连起来,否则 tracker 无法把 arm 相机的观测换算到 odom 系。

    **桥必须接在机械臂链的 root({prefix}world)上,不能接 {prefix}base_link**:
    URDF 里有 world_to_base_link 这个 fixed joint,{prefix}base_link 已经有
    parent {prefix}world;再补一条 base_link → {prefix}base_link 会让它有两个
    parent → tf2 的 parent 解析重新变成不确定 → "Tf has two or more unconnected
    trees" 随机失败(这正是本次要消灭的问题)。接在 root 上则每个 frame 恰好
    一个 parent。

    这条边的数值 = **机械臂在 Go2 躯干上的安装偏移**(URDF 里 world 与
    base_link 恒等,所以 world 就是机械臂的安装座)。当前取单位阵(未实测),
    等价于"机械臂安装座 ≡ Go2 base_link";实测后填真实值即可,链路结构不变。

    用 TransformBroadcaster(dynamic /tf) + 100ms timer 周期性重发,而不是
    StaticTransformBroadcaster:实测在 cyclonedds+eth10 setup 下,一次性的
    /tf_static 消息对 late-joining subscriber 不可靠(收不到 latched message)。

    Args:
      node: ROS 节点
      frame_prefix: 机械臂 TF frame 前缀,如 "arm1/"(空 = 不加前缀)
      offset_xyz: 安装偏移平移(米)
      offset_rpy: 安装偏移 RPY(弧度)

    Returns:
      (broadcaster, timer) 元组,需由调用方持有防 GC
    """
    if not frame_prefix:
        raise ValueError(
            "broadcast_arm_mount_tf 需要非空 frame_prefix:"
            "无前缀时机械臂 base_link 与机器人本体 base_link 同名,会直接冲突"
        )
    qx, qy, qz, qw = rpy_to_quat(offset_rpy)
    # 固定变换 → /tf_static,只 publish 一次。
    # TRANSIENT_LOCAL 的 latch 对**晚启动**的订阅者有效(实测),前提是本
    # publisher 常驻 —— 因此 broadcaster 引用必须由调用方持有(防 GC),
    # 否则 node 被回收 → publisher 析构 → latched 数据随之消失。
    broadcaster = StaticTransformBroadcaster(node)

    t = TransformStamped()
    t.header.stamp = node.get_clock().now().to_msg()
    t.header.frame_id = "base_link"
    t.child_frame_id = f"{frame_prefix}world"
    t.transform.translation = Vector3(
        x=float(offset_xyz[0]), y=float(offset_xyz[1]), z=float(offset_xyz[2]))
    t.transform.rotation = Quaternion(x=qx, y=qy, z=qz, w=qw)
    broadcaster.sendTransform(t)

    node.get_logger().info(
        f"Arm mount TF published (static): base_link → {frame_prefix}world "
        f"(t=({offset_xyz[0]:.3f}, {offset_xyz[1]:.3f}, {offset_xyz[2]:.3f}))"
    )
    return broadcaster


def broadcast_camera_mount_tf(node: rclpy.node.Node,
                              tf_buffer: Buffer,
                              cam_to_parent: np.ndarray,
                              parent_frame: str,
                              link_frame: str,
                              optical_frame: str,
                              lookup_timeout_sec: float = 0.05):
    """广播 parent_frame → link_frame 的相机安装 TF(一次性 /tf_static)。

    realsense2_camera_node 只 publish 相机**内部**链
    (camera_X_link → camera_X_color_frame → ..._optical_frame),
    camera_X_link 是那棵树的 root,不与机器人本体相连。要让相机在 TF 里可见
    (rviz 等),需要补一条把相机树接到机器人侧。

    **必须接 camera_X_link(相机体 link),不能接 ..._optical_frame**:
    optical 帧已经有 realsense 的 parent(camera_X_color_frame),再补一条
    就成了双 parent → tf2 parent 解析不确定。

    数值合成(标定方向:相机光学系 → 机器人侧,**原值不取逆**):
        p_parent = CALIB · p_optical
        p_optical = inv(M1) · p_link,  M1 = lookup(link, optical)
        ⇒ p_parent = (CALIB ⊗ inv(M1)) · p_link
    所以 M = CALIB @ inv(lookup(link, optical))。

    lookup(link, optical) 的 tf 语义是 p_link = R·p_optical + t
    (parent=link, child=optical),取逆后才是 p_optical → p_link。

    realsense 节点可能晚于本节点启动,所以用一个 timer 轮询解析;
    **解析成功即 publish 一次并停掉 timer**(内容是常量,不需要周期重发)。

    Args:
      node: ROS 节点
      tf_buffer: tf2 buffer(用于查 realsense 的内部变换)
      cam_to_parent: 4x4 标定矩阵,相机光学系 → 机器人侧
        (head: → **机械臂基座** `<prefix>base_link`;arm: → `<prefix>tcp_link`。
         注意 head 的不是机器人本体 base_link,详见模块 docstring)
      parent_frame: 机器人侧的 parent 帧
        (head: `<prefix>base_link`;arm: `<prefix>tcp_link`)
      link_frame: 相机体 link 名(camera_head_link / camera_arm_link)
      optical_frame: 相机光学帧名
      lookup_timeout_sec: 单次 lookup 超时

    Returns:
      (broadcaster, timer) 元组,需由调用方持有防 GC
    """
    broadcaster = StaticTransformBroadcaster(node)
    timeout = rclpy.duration.Duration(seconds=lookup_timeout_sec)
    # 注意:rclpy(Humble)的 timer callback **不接受参数**
    # (executors 里是 `await_or_execute(tmr.callback)`,无参调用),
    # 所以 timer 句柄要自己放 state 里,不能靠回调参数拿。
    state = {"published": False, "timer": None}

    def _try_publish() -> None:
        if state["published"]:
            return
        try:
            M1 = transform_to_matrix(
                tf_buffer.lookup_transform(
                    link_frame, optical_frame,
                    time=rclpy.time.Time(), timeout=timeout))
        except Exception:
            return  # realsense 还没起来,下次再试
        # p_parent = (CALIB ⊗ inv(M1)) · p_link
        m = cam_to_parent @ np.linalg.inv(M1)
        t = TransformStamped()
        t.header.stamp = node.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = link_frame
        t.transform.translation = Vector3(x=m[0, 3], y=m[1, 3], z=m[2, 3])
        q = _matrix_to_quat(m[:3, :3])
        t.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        broadcaster.sendTransform(t)
        state["published"] = True
        if state["timer"] is not None:
            state["timer"].cancel()  # 内容常量,一次即可
        node.get_logger().info(
            f"Camera mount TF published (static): {parent_frame} → {link_frame} "
            f"t=({m[0,3]:.3f}, {m[1,3]:.3f}, {m[2,3]:.3f})"
        )

    state["timer"] = node.create_timer(0.2, _try_publish)
    _try_publish()
    return broadcaster, state["timer"]


def _matrix_to_quat(R: np.ndarray) -> Tuple[float, float, float, float]:
    """3x3 旋转矩阵 → xyzw 四元数。"""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return (x, y, z, w)
