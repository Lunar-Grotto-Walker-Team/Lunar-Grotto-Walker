"""Launch for the Go2 posture BT demo.

Brings up bt_runner + the body_pose_bridge service node + the
vui_bridge service node. Same pattern as bt_with_lgw_moveit.launch.py:74
using IncludeLaunchDescription. We keep this as a separate top-level
launch so the demo tree can be run without the pick-and-place moveit
chain.

Trigger the tree after launch with:
  ros2 action send_goal /lgw_bt_execution \
    btcpp_ros2_interfaces/action/ExecuteTree \
    "{target_tree: Go2PostureDemo}"

Order matters:
  1. bt_runner first so its RosServiceNode clients (SetBodyPose,
     SetLedBrightness) are created before StandUp starts. Their
     internal wait_for_service() will block until the bridge nodes
     publish their services in steps 2-3, so any lag is absorbed
     transparently.
  2. body_pose_bridge_node next — /set_body_pose is a critical-path
     dependency of every SetBodyPose node in the tree.
  3. vui_bridge_node last — /set_vui_brightness is needed for every
     SetLedBrightness node. Spawned as a long-running process so the
     SDK subprocess (go2_vui_set_brightness_helper) is one fork away
     on every tick instead of paying full SDK init overhead.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    body_pose_bridge_launch = os.path.join(
        get_package_share_directory('body_pose_bridge'),
        'launch', 'body_pose_bridge.launch.py')
    vui_bridge_launch = os.path.join(
        get_package_share_directory('lgw_vui_bridge'),
        'launch', 'vui_bridge.launch.py')

    return LaunchDescription([
        Node(
            package='lgw_bt_planner',
            executable='bt_runner',
            name='bt_runner',
            parameters=[{
                'plugins': ['lgw_bt_planner/bt_plugins'],
                'behavior_trees': ['lgw_bt_planner/bt_trees'],
                'action_name': 'lgw_bt_execution',
                'tick_frequency': 10,
            }],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(body_pose_bridge_launch)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(vui_bridge_launch)),
    ])
