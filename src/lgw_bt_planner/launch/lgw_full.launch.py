"""Deprecated alias — use lgw_servo_stack.launch.py (2026-11-15).

"full" no longer describes this file: it brings up the ARM SERVO STACK
(MoveIt demo + moveit_servo + lgw servo + optional BT).  The full chain is
now lgw_chain.launch.py (servo stack + 3D tracker).

Kept so existing commands and muscle memory keep working; arguments are
forwarded verbatim.  Remove once nothing references it any more.
"""
import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Arguments lgw_servo_stack.launch.py declares; forwarded only when given.
_FORWARD = (
    'arm_type', 'effector_type', 'revo2_type', 'tcp_offset',
    'hardware_backend', 'frame_prefix', 'follow', 'use_rviz',
    'servo_singularity_lower', 'servo_singularity_hard_stop',
    'servo_v_max', 'servo_w_max', 'hold_recovery_enable',
    'recovery_allow_target_lost', 'recovery_deadline_ms', 'posture_trace',
    'servo_joint_domain', 'servo_velocity_scaling', 'use_bt',
)


def _forward(context):
    target = os.path.join(
        get_package_share_directory('lgw_bt_planner'), 'launch',
        'lgw_servo_stack.launch.py')
    forwarded = {
        name: context.launch_configurations[name]
        for name in _FORWARD
        if name in context.launch_configurations
    }
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(target),
        launch_arguments=forwarded.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg=(
            '[deprecated] lgw_full.launch.py -> lgw_servo_stack.launch.py '
            '(full chain: lgw_chain.launch.py)')),
        OpaqueFunction(function=_forward),
    ])
