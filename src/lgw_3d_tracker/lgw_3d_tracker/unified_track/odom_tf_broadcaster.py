"""/utlidar/robot_pose → 动态 odom→base_link TF。

**时间轴校正(Plan A,2026-10-23)**:Go2 内置主机没有对时,其 stamp 与 Jetson
本地时钟相差一个**固定偏移**(真机实测 659.509 s,10 s 内漂移 0.000 s)。旧做法是
检测到 drift > 1 s 就直接用 `now()` 重新打戳 —— 这会把位姿"贴"到**回调时刻**,
而它实际是在更早的测量时刻采样的,于是 TF 时间轴整体滞后(实测最新样本落后
wallclock p50 ~100 ms),按检测 stamp 插值出来的位姿也就系统性偏旧。

现在改为**估计固定偏移并校正**:

    offset  = min over 窗口 (now_local - remote_stamp)
    corrected_stamp = remote_stamp + offset

取最小值是因为传输延迟恒为正,最小值最接近真实时钟差(实测到达抖动 p50 0.7 ms)。
校正后的 stamp 表示**测量时刻(换算到本地时基)**,与检测 stamp 同源,可直接插值。

与旧行为的关键差别:

| | 旧(fallback to now) | 新(offset 校正) |
|---|---|---|
| stamp 含义 | 回调处理时刻 | 测量时刻(本地时基) |
| 位姿新鲜度 | 被"贴晚"一个处理延迟 | 只差传输+抖动 |
| 检测 stamp 插值 | 能,但结果偏旧 | 能,且时间对齐 |

兜底:校正后的 stamp 若落在未来或过旧,退回 `now()`;远端时钟跳变
(机器人重启/对时)时清空窗口重新收敛。
"""
import collections

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion, Vector3
from tf2_ros.transform_broadcaster import TransformBroadcaster


class OdomTFBroadcaster:
    """订阅 /utlidar/robot_pose 后调用 update() 即可广播动态 TF。

    偏移估计用滑动窗口的最小值,能自动跟踪慢漂移;远端时钟跳变时重新收敛。
    """

    # 偏移估计窗口:100 帧 ≈ 5.3 s @19 Hz。
    # 2026-10-23 实测:100 vs 200 帧的估计值完全相同(差值 0.000 ms)——
    # 该链路的最小延迟每 <50 帧就重复出现,长窗口不带来额外精度。
    # 但窗口越短,「旧样本滑出」越快 → 时钟跳变/坏样本的恢复时间从 10.7 s 减半到 5.3 s。
    WINDOW_SIZE = 100
    # 冷启动保护:样本数不足时直接采用当前样本的延迟(退化为旧行为,不会突跳)
    MIN_SAMPLES = 20
    # 偏移突变阈值(5 s):超过就认为远端时钟跳变(重启/对时)
    JUMP_THRESHOLD_NS = 5 * 10**9
    # 校正后 stamp 的合法区间:不早于 now-2s、不晚于 now+100ms
    MAX_AGE_NS = 2 * 10**9
    MAX_AHEAD_NS = 100 * 10**6

    def __init__(self, node: rclpy.node.Node):
        self._node = node
        self._tf_broadcaster = TransformBroadcaster(node)
        self._deltas = collections.deque(maxlen=self.WINDOW_SIZE)
        self._offset_ns = None
        self._msg_count = 0

    def update(self, msg: PoseStamped) -> None:
        """把 PoseStamped 转成 odom → base_link TF 发出。

        Args:
          msg: PoseStamped,header.stamp 为 SLAM 输出时刻(远端时基)
        """
        now_ns = self._node.get_clock().now().nanoseconds
        msg_stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        corrected_ns = msg_stamp_ns + self._estimate_offset(now_ns, msg_stamp_ns)

        # 兜底:估计异常时不要把 stamp 打到会被 buffer 立刻淘汰的位置
        if not (now_ns - self.MAX_AGE_NS < corrected_ns <= now_ns + self.MAX_AHEAD_NS):
            self._node.get_logger().warn(
                f"odom TF stamp 校正结果不合理(corrected-now="
                f"{(corrected_ns - now_ns) / 1e9:.3f} s),退回 now()",
                throttle_duration_sec=5.0,
            )
            corrected_ns = now_ns

        t = TransformStamped()
        t.header.stamp = rclpy.time.Time(nanoseconds=corrected_ns).to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation = Vector3(
            x=float(msg.pose.position.x),
            y=float(msg.pose.position.y),
            z=float(msg.pose.position.z),
        )
        t.transform.rotation = Quaternion(
            x=float(msg.pose.orientation.x),
            y=float(msg.pose.orientation.y),
            z=float(msg.pose.orientation.z),
            w=float(msg.pose.orientation.w),
        )
        self._tf_broadcaster.sendTransform(t)

    def _estimate_offset(self, now_ns: int, msg_stamp_ns: int) -> int:
        """估计 remote→local 的时钟偏移(纳秒),返回可用于校正的 offset。

        offset = min(now - remote_stamp):延迟只可能为正,最小值最接近真值。
        """
        d = now_ns - msg_stamp_ns
        self._deltas.append(d)
        self._msg_count += 1

        if len(self._deltas) < self.MIN_SAMPLES or self._offset_ns is None:
            # 冷启动:样本不足,先用当前样本(等价于旧的 now() 行为,不会突跳)
            current = d
        else:
            current = min(self._deltas)
            if abs(current - self._offset_ns) > self.JUMP_THRESHOLD_NS:
                # 远端时钟跳变(机器人重启/对时)→ 清窗重新收敛
                self._offset_ns = d
                self._deltas.clear()
                self._deltas.append(d)
                self._node.get_logger().warn(
                    f"检测到远端时钟跳变(jump={current / 1e9:+.3f} s),"
                    f"重置偏移估计窗口"
                )
                return d

        self._offset_ns = current

        # 每秒打印一次当前估计,便于现场核对
        if self._msg_count % 100 == 0:
            self._node.get_logger().info(
                f"clock offset (remote→local) = {current / 1e9:.3f} s "
                f"(窗口 {len(self._deltas)} 帧)"
            )
        return current
