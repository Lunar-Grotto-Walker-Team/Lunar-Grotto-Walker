#!/usr/bin/env python3
"""Publish odom -> base_link from /utlidar/robot_pose, without the tracker.

The 3D tracker's OdomTFBroadcaster normally provides this frame, but starting
the whole tracker (cameras + YOLOE) just to get a TF is wasteful and would also
race with a fake track publisher on /perception/tracks/unified.

This node reuses the broadcaster's own clock-offset correction, so the stamps
stay consistent with what the tracker would produce.

Usage (inside self_l4t):
    python3 scripts/odom_tf_bridge.py
"""
import sys

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from lgw_3d_tracker.unified_track.odom_tf_broadcaster import OdomTFBroadcaster


class OdomTfBridge(Node):
    def __init__(self):
        super().__init__("odom_tf_bridge")
        self._broadcaster = OdomTFBroadcaster(self)
        self._count = 0
        self.create_subscription(
            PoseStamped, "/utlidar/robot_pose", self._on_pose, 10)
        # Connect the two trees. The arm chain roots at arm1/world and the
        # tracker normally publishes base_link -> arm1/world (its arm mount
        # bridge, identity offset). Without it, odom <- arm1/base_link has no
        # path and the servo node's odom fallback tier is unusable.
        self._static = StaticTransformBroadcaster(self)
        st = TransformStamped()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = "base_link"
        st.child_frame_id = "arm1/world"
        st.transform.rotation.w = 1.0
        self._static.sendTransform(st)
        self.get_logger().info(
            "relaying /utlidar/robot_pose -> TF odom -> base_link; "
            "static base_link -> arm1/world")

    def _on_pose(self, msg: PoseStamped) -> None:
        self._broadcaster.update(msg)
        self._count += 1
        if self._count == 1:
            self.get_logger().info("first robot_pose received; odom frame is up")


def main() -> int:
    rclpy.init()
    node = OdomTfBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
