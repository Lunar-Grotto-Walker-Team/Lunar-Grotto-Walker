"""Launch body_pose_bridge_node.

Starts the persistent bridge that holds a Go2 Euler rpy target by
republishing Euler(1007) at 50Hz (single-shot Euler+BalanceStand on
each SetBodyPose call to latch entry into balance mode).

2026-10-13 v7:BodyHeight api(1013)bh_off 在 Go2 固件是 dead control
(真机验证),bridge 3-call (Euler+BH+BalStand) 退化为 2-call
(Euler+BalStand),50Hz refresh 退化为 Euler-only。

See docs/superpowers/specs/2026-10-10-body-pose-bridge-design.md.

Usage:
  ros2 launch body_pose_bridge body_pose_bridge.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='body_pose_bridge',
            executable='body_pose_bridge_node',
            name='body_pose_bridge_node',
            output='screen',
        ),
    ])