"""Scenario-level test: bring the servo node up on mock hardware and assert it
reports a sane state without any real arm.

Task 2 (2026-11-07 node-split plan) started this; Task 8 extends it to the
lifecycle services. Runs entirely in the container:

    launch_test src/lgw_bt_planner/test/session_launch_test.py

It deliberately does NOT drive the control loop (the per-tick deterministic
replay is built once `TrackingInput` exists — see the plan ledger).
"""
import os
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions

import rclpy
from rclpy.node import Node

from lgw_bt_planner.msg import ServoState
from std_srvs.srv import Trigger

STACK_ENV = {
    "RCUTILS_LOGGING_BUFFERED_STREAM": "1",
}


def generate_test_description():
    launch_file = os.path.join(
        os.environ.get("LGW_WS", "/my_ws/l4t/lg_walker_ws"),
        "src/lgw_bt_planner/launch/lgw_full.launch.py",
    )
    stack = launch_ros.actions.Node(
        package="lgw_bt_planner",
        executable="servo_node",
        name="lgw_servo_node",
        output="screen",
        parameters=[{"frame_prefix": "arm1/"}],
    )
    return launch.LaunchDescription([
        # Only the servo node: the launch file needs move_group and
        # ros2_control, which is what the bench scripts already cover. This
        # test's job is to prove the node starts and publishes a state.
        stack,
        launch_testing.actions.ReadyToTest(),
    ]), {"stack": stack}


class TestServoNodeComesUp(unittest.TestCase):
    def test_state_topic_is_published(self, proc_output):
        rclpy.init()
        node = Node("session_launch_probe")
        seen = []
        node.create_subscription(
            ServoState, "/lgw/servo/state", lambda m: seen.append(m), 10)
        deadline = time.time() + 20.0
        while time.time() < deadline and not seen:
            rclpy.spin_once(node, timeout_sec=0.2)
        node.destroy_node()
        rclpy.shutdown()
        self.assertTrue(seen, "no /lgw/servo/state within 20 s")


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)


class TestLifecycleServices(unittest.TestCase):
    """Task 8: the services must be reachable and must refuse out of context.

    The node is IDLE here -- no pose stream, no move_group, so Servo is never
    started. That is exactly the state in which release must be a no-op and
    pause / resume must refuse, and it exercises the real rclcpp service path
    rather than a fake.
    """

    def test_release_is_idempotent_in_idle(self):
        rclpy.init()
        node = Node("session_launch_release")
        client = node.create_client(Trigger, "/lgw/servo/release")
        first = _call_with(node, client)
        second = _call_with(node, client)
        node.destroy_node()
        rclpy.shutdown()

        # IDLE/RELEASING accept RELEASE as a no-op: no Servo call at all, so
        # this must succeed twice even with no move_group running.
        self.assertTrue(first.success, first.message)
        self.assertTrue(second.success, second.message)
        self.assertEqual(second.message, "already released")

    def test_pause_is_refused_in_idle(self):
        rclpy.init()
        node = Node("session_launch_pause")
        client = node.create_client(Trigger, "/lgw/servo/pause")
        result = _call_with(node, client)
        node.destroy_node()
        rclpy.shutdown()

        self.assertFalse(result.success)
        self.assertIn("pause refused", result.message)

    def test_resume_is_refused_in_idle(self):
        rclpy.init()
        node = Node("session_launch_resume")
        client = node.create_client(Trigger, "/lgw/servo/resume")
        result = _call_with(node, client)
        node.destroy_node()
        rclpy.shutdown()

        self.assertFalse(result.success)
        self.assertIn("resume refused", result.message)


def _call_with(node, client):
    """Call a Trigger service and spin until the response arrives."""
    assert client.wait_for_service(timeout_sec=10.0), "service never appeared"
    future = client.call_async(Trigger.Request())
    deadline = time.time() + 10.0
    while time.time() < deadline and not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
    assert future.done(), "service call did not complete"
    return future.result()
