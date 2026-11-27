"""One-shot bring-up for the current scanning chain (2026-11-15).

What this launch starts:
  * lgw_full.launch.py   — agx_arm_moveit demo (RSP + move_group + JTC) +
                           moveit_servo + lgw_servo_node + (optional) BT
  * unified_track_node   — the 3D tracker that owns the target registry

What it deliberately does NOT start:
  * the arm driver / CAN bus — `scripts/start_lgw_arm.sh --driver-only` on the
    HOST (docker exec is not available inside the container);
  * the two realsense cameras — they live in the yoloe_humble_orchestration
    install space (see scripts/start_lgw_head_to_arm_lookat.sh Phase Y2, or
    `ros2 launch yoloe_humble_orchestration two_realsense.launch.py` with the
    camera serials and ROS_DOMAIN_ID=0).

Arguments are forwarded to lgw_full.launch.py; tracker-only arguments are
listed at the bottom.

Usage (inside the container, after sourcing the env):
    ros2 launch lgw_bt_planner lgw_chain.launch.py follow:=true use_bt:=false
On the host: scripts/start_lgw_chain.sh [args...]
"""
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    lgw_share = get_package_share_directory('lgw_bt_planner')
    tracker_share = get_package_share_directory('lgw_3d_tracker')
    full_launch = os.path.join(lgw_share, 'launch', 'lgw_full.launch.py')
    tracker_launch = os.path.join(tracker_share, 'launch',
                                  'unified_track_node.launch.py')

    return LaunchDescription([
        # ---- forwarded to lgw_full.launch.py -------------------------------
        DeclareLaunchArgument(
            'follow', default_value='true', choices=['true', 'false'],
            description='true: real arm (/feedback), false: bench (/control)'),
        DeclareLaunchArgument(
            'use_bt', default_value='false', choices=['true', 'false'],
            description='start the behaviour tree as well'),
        DeclareLaunchArgument(
            'use_rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument(
            'servo_v_max', default_value='0.20',
            description='first-motion hardware cap (m/s)'),
        DeclareLaunchArgument(
            'servo_w_max', default_value='0.5',
            description='first-motion hardware cap (rad/s)'),
        DeclareLaunchArgument(
            'servo_velocity_scaling', default_value='0.5'),
        DeclareLaunchArgument(
            'frame_prefix', default_value='arm1/'),
        DeclareLaunchArgument(
            'hold_recovery_enable', default_value='true'),
        # ---- forwarded implicitly to lgw_bt_planner.launch.py ----------------
        # (nested includes inherit launch_configurations, no explicit plumbing
        # needed). transit_node 是 TransitToLocalPoint 的动作服务器; V2 系列
        # 树(TransitMission/Patrol/Return)必须有它, 否则 3×10 s 后整树 FAILURE。
        # 只有 use_bt:=true 时才会真的起。
        DeclareLaunchArgument(
            'use_transit', default_value='true', choices=['true', 'false'],
            description='start go2_transit/transit_node with the BT runner; '
                        'set false if another launch already runs one'),
        # ---- tracker ------------------------------------------------------
        DeclareLaunchArgument(
            'use_tracker', default_value='true', choices=['true', 'false'],
            description='start the 3D tracker (target registry owner)'),
        DeclareLaunchArgument(
            'trace_log_dir', default_value='',
            description='tracker per-tick JSONL log directory; empty = off'),
        DeclareLaunchArgument(
            'trace_tag', default_value=''),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(full_launch),
            launch_arguments={
                'follow': LaunchConfiguration('follow'),
                'use_bt': LaunchConfiguration('use_bt'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'servo_v_max': LaunchConfiguration('servo_v_max'),
                'servo_w_max': LaunchConfiguration('servo_w_max'),
                'servo_velocity_scaling':
                    LaunchConfiguration('servo_velocity_scaling'),
                'frame_prefix': LaunchConfiguration('frame_prefix'),
                'hold_recovery_enable':
                    LaunchConfiguration('hold_recovery_enable'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(tracker_launch),
            condition=IfCondition(LaunchConfiguration('use_tracker')),
            launch_arguments={
                'trace_log_dir': LaunchConfiguration('trace_log_dir'),
                'trace_tag': LaunchConfiguration('trace_tag'),
            }.items(),
        ),
    ])
