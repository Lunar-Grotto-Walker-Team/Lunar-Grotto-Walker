#!/usr/bin/env python3
"""沿当前朝向逐段前进（每段 1 m，最多 N 段），用局部代价地图避障，
收到目标识别信号就提前停。

用法：
  ros2 run go2_transit forward_explore.py               # 走 5 段，每段 1 m
  ros2 run go2_transit forward_explore.py --steps 3 --step-m 0.8
  ros2 run go2_transit forward_explore.py --dry-run     # 只算点，不动狗

每一段的选点逻辑：
  1. 以当前位姿为基准，在前方 step_m 处取候选点；
  2. 横向依次尝试 0 / ±0.2 / ±0.4 m（左正右负），选第一个
     "段内采样点全部空闲"的候选；
  3. 一个都不可行 -> 停止（不原地乱转），返回 reason=blocked；
  4. 否则发 go2_transit 的 TransitToPose（align_yaw=False，机头不变）。

停止条件（任一）：
  - 走满 steps 段                         -> reason=done
  - 前方无可行点（局部代价地图）           -> reason=blocked
  - 收到目标识别信号（std_msgs/Bool true） -> reason=detected
  - 单段 transit 失败                     -> reason=transit_failed

依赖：/odom（go2_driver）、/local_costmap/costmap（Nav2）、
      动作 /go2_transit/transit_to_pose。

注意：局部代价地图的 unknown(-1) 当作"不可走"，宁停不闯。
"""

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool

from go2_transit.action import TransitToPose

LATERAL_OFFSETS = (0.0, 0.2, -0.2, 0.4, -0.4)


def yaw_of(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ForwardExplore(Node):
    def __init__(self, args):
        super().__init__('forward_explore')
        self.args = args
        self.detected = False
        self.odom = None
        self.costmap = None
        self._transit = ActionClient(self, TransitToPose,
                                     '/go2_transit/transit_to_pose')
        self.create_subscription(Odometry, args.odom_topic, self._on_odom, 10)
        self.create_subscription(OccupancyGrid, args.costmap_topic,
                                 self._on_costmap, 1)
        self.create_subscription(Bool, args.detect_topic, self._on_detect, 10)

    # ---------- callbacks ----------
    def _on_odom(self, msg):
        self.odom = msg

    def _on_costmap(self, msg):
        self.costmap = msg

    def _on_detect(self, msg):
        if msg.data:
            self.detected = True
            self.get_logger().info('[explore] 收到目标识别信号，准备停止')

    # ---------- costmap helpers ----------
    def _cell_cost(self, x, y):
        cm = self.costmap
        info = cm.info
        gx = int((x - info.origin.position.x) / info.resolution)
        gy = int((y - info.origin.position.y) / info.resolution)
        if gx < 0 or gy < 0 or gx >= info.width or gy >= info.height:
            return 100  # 出图 = 不可走
        c = cm.data[gy * info.width + gx]
        return 100 if c < 0 else c  # unknown 当不可走

    def _segment_free(self, x0, y0, x1, y1):
        """沿线段按 0.1 m 采样，并检查左右 corridor_half_width。"""
        dist = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(dist / 0.1))
        nx, ny = (-(y1 - y0) / dist, (x1 - x0) / dist) if dist > 1e-6 else (0, 0)
        for i in range(n + 1):
            t = i / n
            px = x0 + (x1 - x0) * t
            py = y0 + (y1 - y0) * t
            for off in (0.0, self.args.corridor_half_width,
                        -self.args.corridor_half_width):
                if self._cell_cost(px + nx * off, py + ny * off) >= self.args.cost_threshold:
                    return False
        return True

    def _pick_target(self):
        o = self.odom.pose.pose
        yaw = yaw_of(o.orientation)
        x0, y0 = o.position.x, o.position.y
        for lat in LATERAL_OFFSETS:
            tx = x0 + self.args.step_m * math.cos(yaw) - lat * math.sin(yaw)
            ty = y0 + self.args.step_m * math.sin(yaw) + lat * math.cos(yaw)
            if self._segment_free(x0, y0, tx, ty):
                return tx, ty, lat
        return None

    # ---------- action ----------
    def _send_transit(self, tx, ty):
        if not self._transit.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('[explore] transit action 不可用')
            return False
        goal = TransitToPose.Goal()
        goal.target = PoseStamped()
        goal.target.header.frame_id = 'odom'
        goal.target.pose.position.x = tx
        goal.target.pose.position.y = ty
        goal.target.pose.orientation.w = 1.0
        goal.align_yaw = False
        fut = self._transit.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        handle = fut.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('[explore] transit goal 被拒绝')
            return False
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=self.args.timeout_s)
        return res.done() and res.result() is not None

    # ---------- main loop ----------
    def run(self):
        self.get_logger().info(
            f'[explore] steps={self.args.steps} step_m={self.args.step_m} '
            f'cost<{self.args.cost_threshold} detect={self.args.detect_topic}')
        while self.odom is None or self.costmap is None:
            if not rclpy.ok():
                return 'interrupted'
            rclpy.spin_once(self, timeout_sec=0.2)

        for i in range(self.args.steps):
            if self.detected:
                self.get_logger().info('[explore] 目标已识别，停止')
                return 'detected'
            rclpy.spin_once(self, timeout_sec=0.1)

            pick = self._pick_target()
            if pick is None:
                self.get_logger().warn(
                    f'[explore] 第 {i + 1} 段前方 {self.args.step_m} m 内无可行点，停止')
                return 'blocked'
            tx, ty, lat = pick
            self.get_logger().info(
                f'[explore] 第 {i + 1}/{self.args.steps} 段 -> '
                f'({tx:+.2f}, {ty:+.2f}) lat={lat:+.1f}')
            if self.args.dry_run:
                continue
            if not self._send_transit(tx, ty):
                self.get_logger().error('[explore] transit 失败，停止')
                return 'transit_failed'
        return 'done'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--steps', type=int, default=5)
    ap.add_argument('--step-m', type=float, default=1.0)
    ap.add_argument('--corridor-half-width', type=float, default=0.15)
    ap.add_argument('--cost-threshold', type=int, default=50)
    ap.add_argument('--timeout-s', type=float, default=60.0)
    ap.add_argument('--odom-topic', default='/odom')
    ap.add_argument('--costmap-topic', default='/local_costmap/costmap')
    ap.add_argument('--detect-topic', default='/target_detected')
    ap.add_argument('--dry-run', action='store_true')
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args)
    node = ForwardExplore(args)
    try:
        reason = node.run()
        node.get_logger().info(f'[explore] 结束: {reason}')
        rc = 0 if reason in ('done', 'detected') else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
