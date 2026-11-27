#!/usr/bin/env python3
"""按"相对当前位置"发一次摆渡指令 —— 真机联调用的临时替身。

将来这些会由行为树发；现在用来手动验证 go2_transit 的接口。

用法：
  ros2 run go2_transit transit_relative.py forward 0.3
  ros2 run go2_transit transit_relative.py polar 3.0 30
  ros2 run go2_transit transit_relative.py rotate 20
  ros2 run go2_transit transit_relative.py polar 3.0 30 --dry-run

  forward <m>              沿当前朝向走 <m> 米
  polar   <m> <bearing>    沿"当前朝向 + bearing 度"走 <m> 米（左正右负）
  rotate  <deg>            原地左转 <deg> 度（左正右负）

  --dry-run     只打印算出来的目标，不发动作
  --odom-topic  取位姿的话题，默认 /odom

位姿取 /odom（go2_driver 发布，frame odom / child base_footprint），
和 go2_transit 的默认 planning_frame 一致。
"""

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node

from go2_transit.action import RotateToYaw, TransitToPose


def yaw_of(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RelativeTransit(Node):
    def __init__(self, odom_topic):
        super().__init__('transit_relative_cli')
        self._pose = None
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self._transit = ActionClient(self, TransitToPose, '/go2_transit/transit_to_pose')
        self._rotate = ActionClient(self, RotateToYaw, '/go2_transit/rotate_to_yaw')

    def _on_odom(self, msg):
        self._pose = msg

    def wait_pose(self, timeout_s=10.0):
        deadline = self.get_clock().now().nanoseconds + int(timeout_s * 1e9)
        while self._pose is None:
            if self.get_clock().now().nanoseconds > deadline:
                return None
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._pose

    def send_and_wait(self, client, goal, timeout_s=120.0):
        if not client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('动作服务器没起来，检查 nav2 的 controller/behavior server')
            return None
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('目标被拒绝')
            return None
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_s)
        if not result_future.done():
            self.get_logger().error(f'{timeout_s:.0f}s 内没收到结果，尝试取消')
            handle.cancel_goal_async()
            return None
        return result_future.result().result


def main(argv=None):
    parser = argparse.ArgumentParser(description='按相对当前位置发一次摆渡指令')
    parser.add_argument('command', choices=['forward', 'polar', 'rotate'])
    parser.add_argument('value', type=float, help='forward/polar: 米；rotate: 度（左正右负）')
    parser.add_argument(
        'bearing', type=float, nargs='?', default=None,
        help='polar 必填：相对当前朝向的角度（度，左正右负）')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--odom-topic', default='/odom')
    args = parser.parse_args(argv)

    rclpy.init()
    node = RelativeTransit(args.odom_topic)
    try:
        odom = node.wait_pose()
        if odom is None:
            node.get_logger().error(f'{args.odom_topic} 10 s 内没有消息，go2_driver 起了吗？')
            return 2

        p = odom.pose.pose.position
        yaw = yaw_of(odom.pose.pose.orientation)
        node.get_logger().info(
            f'当前位姿 odom 系: x={p.x:.3f} y={p.y:.3f} yaw={math.degrees(yaw):+.1f}°')

        if args.command in ('forward', 'polar'):
            bearing_deg = args.bearing if args.command == 'polar' else 0.0
            if bearing_deg is None:
                node.get_logger().error('polar 需要第二个参数：相对角度（度）')
                return 2
            travel_yaw = yaw + math.radians(bearing_deg)
            tx = p.x + args.value * math.cos(travel_yaw)
            ty = p.y + args.value * math.sin(travel_yaw)
            node.get_logger().info(
                f'目标: 沿 {math.degrees(travel_yaw):+.1f}° 走 {args.value:.2f} m '
                f'(相对当前朝向 {bearing_deg:+.1f}°) → x={tx:.3f} y={ty:.3f}')

            if args.dry_run:
                return 0

            goal = TransitToPose.Goal()
            goal.target = PoseStamped()
            goal.target.header.frame_id = 'odom'
            goal.target.pose.position.x = tx
            goal.target.pose.position.y = ty
            goal.target.pose.orientation.w = 1.0
            goal.align_yaw = False
            result = node.send_and_wait(node._transit, goal)
            if result is None:
                return 3
            node.get_logger().info(
                f'结果: success={result.success} reason={result.reason} '
                f'xy_err={result.final_xy_error:.2f}')
            return 0 if result.success else 1

        target_yaw = yaw + math.radians(args.value)
        node.get_logger().info(
            f'目标: 转 {args.value:+.1f}° → yaw={math.degrees(target_yaw):+.1f}°')

        if args.dry_run:
            return 0

        goal = RotateToYaw.Goal()
        goal.target_yaw = float(target_yaw)
        result = node.send_and_wait(node._rotate, goal, timeout_s=60.0)
        if result is None:
            return 3
        node.get_logger().info(
            f'结果: success={result.success} reason={result.reason} '
            f'yaw_err={result.final_yaw_error:.2f}')
        return 0 if result.success else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
