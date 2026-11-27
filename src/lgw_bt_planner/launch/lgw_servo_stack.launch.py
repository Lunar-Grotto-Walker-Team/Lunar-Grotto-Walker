"""Single-stack bringup: arm MoveIt + moveit_servo + lgw servo node + BT.

One launch for the production servo stack (spec 2026-10-26 §5.2):

  1. agx_arm_moveit/launch/demo.launch.py  — rsp + move_group + JTC + broadcaster
     (the ONLY controller_manager / RSP in this stack — never start a second)
  2. moveit_servo/servo_node_main          — official Servo node, YAML + URDF/SRDF
  3. lgw_bt_planner/servo_node             — Task 7 takeover service node
  4. apply_head_camera_collision_node      — head camera collision box (self-exits)
  5. lgw_bt_planner/launch/lgw_bt_planner.launch.py — bt_runner (gated by use_bt)

Bench vs robot differ by `follow` only: it is forwarded to demo.launch.py (rsp
joint_states source) and used here to derive Servo's `joint_topic` and the lgw
node's `pose_watch_topic` from the same value, so the closed-loop pose
source cannot diverge (spec R11):
    follow:=true  -> /feedback/joint_states  (real arm)
    follow:=false -> /control/joint_states   (bench / mock)

Bench runs should also pass use_bt:=false: bt_runner is not needed to exercise
Servo and its trees expect MoveIt-visible state.

Prerequisites (NOT started here, same convention as start_lgw_arm.sh):
  - can0 up + agx_arm_ctrl driver running (single driver process per §5.4).
    On the robot that is `bash scripts/start_lgw_arm.sh --driver-only`: it does
    CAN + driver and SKIPS MoveIt, precisely because this launch includes
    demo.launch.py itself and a second controller_manager must never appear.
  - CAN/driver mode chosen before launch (`move_j` / `move_js`); not switchable
    at runtime.

`ros2 launch lgw_bt_planner lgw_servo_stack.launch.py --show-args` only lists the
arguments and starts nothing.
"""
import copy
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Reuse the exact model builder demo.launch.py uses (URDF/SRDF/kinematics) so
# Servo and move_group can never disagree about the robot model. The module is
# a normal installed .py file next to demo.launch.py.
_AGX_ARM_MOVEIT_LAUNCH_DIR = os.path.join(
    get_package_share_directory('agx_arm_moveit'), 'launch')
if _AGX_ARM_MOVEIT_LAUNCH_DIR not in sys.path:
    sys.path.insert(0, _AGX_ARM_MOVEIT_LAUNCH_DIR)

from _moveit_config_builder import (  # noqa: E402  (sys.path set above)
    ALL_ARM_TYPES,
    ALL_EFFECTOR_TYPES,
    ALL_REVO2_TYPES,
    build_moveit_config,
)

FEEDBACK_JOINT_STATES = '/feedback/joint_states'
CONTROL_JOINT_STATES = '/control/joint_states'

# Matches agx_arm_ctrl's tcp_offset in scripts/start_lgw_arm.sh (12 cm z).
DEFAULT_TCP_OFFSET = '[0.0, 0.0, 0.12, 0.0, 0.0, 0.0]'


def _launch_servo_stack(context):
    follow = LaunchConfiguration('follow').perform(context)
    joint_states_topic = (
        FEEDBACK_JOINT_STATES if follow == 'true' else CONTROL_JOINT_STATES
    )
    # Servo's state source (`moveit_servo.joint_topic`). Servo re-seeds its
    # internal state from this topic every cycle and publishes
    # (that state + delta_theta), so the choice decides what the command is
    # anchored to.
    #   control  (default) JTC/mock domain — decision A'. The real arm is a
    #            follower, so the command can build a lead over it. Chosen
    #            2026-10-28 because anchoring on the real arm made the
    #            per-cycle increment (3.4e-4 rad) vanish into the CAN lag.
    #   feedback real arm — MoveIt's own default wiring. Kept as a switch
    #            because it is the cleanest way to test whether the state
    #            source is responsible for the ~0.2x speed attenuation
    #            (docs/servo/2026-10-30-servo-speed-attenuation.md).
    posture_trace = LaunchConfiguration('posture_trace')
    servo_domain = LaunchConfiguration('servo_joint_domain').perform(context)
    servo_joint_states_topic = (
        FEEDBACK_JOINT_STATES if servo_domain == 'feedback'
        else CONTROL_JOINT_STATES
    )
    frame_prefix = LaunchConfiguration('frame_prefix').perform(context)
    velocity_scaling = float(
        LaunchConfiguration('servo_velocity_scaling').perform(context))
    v_max = float(LaunchConfiguration('servo_v_max').perform(context))
    w_max = float(LaunchConfiguration('servo_w_max').perform(context))
    # HOLD recovery toggles (spec 2026-11-03). They are launch arguments, not
    # runtime parameters: the node copies them into HoldRecoveryParams once at
    # startup (there is no dynamic parameter callback).
    hold_recovery_enable = (
        LaunchConfiguration('hold_recovery_enable').perform(context) == 'true')
    recovery_allow_target_lost = (
        LaunchConfiguration('recovery_allow_target_lost').perform(context) == 'true')
    recovery_deadline_ms = int(
        LaunchConfiguration('recovery_deadline_ms').perform(context))
    singularity_lower = float(
        LaunchConfiguration('servo_singularity_lower').perform(context))
    singularity_hard = float(
        LaunchConfiguration('servo_singularity_hard_stop').perform(context))

    # Same model demo.launch.py builds for move_group. Must be called with the
    # real launch context: it reads arm_type/effector_type/revo2_type/tcp_offset
    # from it.
    moveit_config = build_moveit_config(context)

    share = get_package_share_directory('lgw_bt_planner')
    servo_params_yaml = os.path.join(share, 'config', 'servo_params.yaml')
    servo_node_yaml = os.path.join(share, 'config', 'servo_node.yaml')
    bt_launch = os.path.join(share, 'launch', 'lgw_bt_planner.launch.py')

    # Official Servo node. Servo declares its tunables with the
    # `moveit_servo.` prefix (ServoParameters ns), hence the dotted key.
    #
    # `joint_topic` is deliberately NOT in servo_params.yaml: that file lives
    # under the node-specific `servo_node:` section, and rcl merges wildcard
    # (`/**`) rules BEFORE node-specific ones regardless of --params-file
    # order — so a dict override here would silently lose to it. Measured
    # 2026-10-28: with the URDF/SRDF param files loaded first, `follow:=false`
    # still left Servo subscribed to /feedback/joint_states. Keeping the key out
    # of the YAML makes this override the only source, which works.
    # Servo's kinematics: same model, but with the arm's IK solver REMOVED.
    #
    # Servo has two ways to turn a twist into joint deltas (servo_calcs.cpp):
    #   * IK path  — used when the group has a solver. It calls
    #     searchPositionIK with a HARD-CODED timeout of publish_period/2
    #     (5 ms) and opts.return_approximate_solution = true, then accepts
    #     whatever comes back. Measured 2026-10-30: KDL returned a partial
    #     solution every cycle, so every joint advanced a uniform x0.20 of the
    #     commanded step (independent of twist magnitude), capping the arm at
    #     ~0.27 rad/s while the twist asked for 2.6 rad/s. Note this timeout
    #     is NOT the kinematics.yaml one, and MoveIt 2.5.9's KDL plugin has no
    #     solve_type parameter either — kinematics.yaml cannot fix it.
    #   * Jacobian path — used when the group has NO solver:
    #     delta_theta = pseudo_inverse * delta_x, exact and non-iterative.
    #     Singularity scaling and joint velocity limits are applied after it,
    #     so the safety behaviour is unchanged.
    # move_group keeps the real kinematics file: joint-space planning never
    # calls IK, and pose goals / Cartesian paths keep working as before.
    servo_kinematics = copy.deepcopy(moveit_config.robot_description_kinematics)
    servo_kinematics.get('robot_description_kinematics', {}).pop('arm', None)

    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        output='screen',
        # Long-lived and safety-relevant: if it dies, the whole stack must
        # go down instead of silently holding position.
        on_exit=[Shutdown(reason='moveit_servo (servo_node) exited')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            servo_kinematics,
            servo_params_yaml,
            {
                'moveit_servo.joint_topic': servo_joint_states_topic,
                # Caps joint velocity as a fraction of the SRDF limits
                # (joint_limits.yaml: 5.0 rad/s), applied in _both_ command
                # modes. < 0.01 means "no override" (enforce_limits.cpp:80).
                'moveit_servo.override_velocity_scaling_factor': velocity_scaling,
                # Condition-number thresholds. The 17/30 defaults halt the arm at
                # poses it can physically still reach (measured 2026-10-28: a
                # wrist-heavy lookat pose hit 43 and stopped mid-approach).
                # Raised so ordinary wrist-close poses run at full speed; the
                # joint velocity limits and the control law's w_max still bound it.
                'moveit_servo.lower_singularity_threshold': singularity_lower,
                'moveit_servo.hard_stop_singularity_threshold': singularity_hard,
            },
        ],
    )

    # Task 7 takeover node. `pose_watch_topic` gets the same value as Servo's
    # `joint_topic`; `frame_prefix` follows the launch argument so the TF
    # lookups match the prefixed frames demo.launch.py publishes.
    lgw_servo_node = Node(
        package='lgw_bt_planner',
        executable='servo_node',
        name='lgw_servo_node',
        output='screen',
        # Same rule as moveit_servo: this node owns takeover/scanning, so its
        # death must not leave the rest of the stack running.
        on_exit=[Shutdown(reason='lgw_servo_node exited')],
        parameters=[
            servo_node_yaml,
            # Pinned-posture lookat (spec 2026-10-30 rev2) needs the same
            # URDF move_group uses, so its FK and the TF tcp_link cannot
            # disagree about arm_type / tcp_offset.
            moveit_config.robot_description,
            {
                'pose_watch_topic': joint_states_topic,
                'frame_prefix': frame_prefix,
                # Twist caps for the control law. These are the ONLY effective
                # speed limit in speed_units mode (see servo_velocity_scaling
                # note above); lower them for first-motion hardware tests.
                'v_max': v_max,
                'w_max': w_max,
                'posture_trace': posture_trace,
                'hold_recovery_enable': hold_recovery_enable,
                'recovery_allow_target_lost': recovery_allow_target_lost,
                'recovery_deadline_ms': recovery_deadline_ms,
            },
        ],
    )

    # Reads calibration/head_calibration.json, applies the head camera BOX to
    # /apply_planning_scene and exits. Must start after move_group (ordering
    # only — the node's synchronous service call does the real waiting).
    head_collision_node = Node(
        package='lgw_bt_planner',
        executable='apply_head_camera_collision_node',
        name='head_camera_collision',
        output='screen',
    )

    bt_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bt_launch),
        condition=IfCondition(LaunchConfiguration('use_bt')),
    )

    return [servo_node, lgw_servo_node, head_collision_node, bt_include]


def generate_launch_description():
    demo_launch = os.path.join(
        get_package_share_directory('agx_arm_moveit'), 'launch', 'demo.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'arm_type',
            default_value='piper',
            choices=ALL_ARM_TYPES,
            description='Arm type.',
        ),
        DeclareLaunchArgument(
            'effector_type',
            default_value='agx_gripper',
            choices=ALL_EFFECTOR_TYPES,
            description='End effector type (production stack uses agx_gripper).',
        ),
        DeclareLaunchArgument(
            'revo2_type',
            default_value='left',
            choices=ALL_REVO2_TYPES,
            description='Revo2 hand side (unused with agx_gripper).',
        ),
        DeclareLaunchArgument(
            'tcp_offset',
            default_value=DEFAULT_TCP_OFFSET,
            description=(
                'TCP offset [x, y, z, rx, ry, rz] in m/rad. Must equal the '
                'value passed to agx_arm_ctrl (start_lgw_arm.sh).'
            ),
        ),
        DeclareLaunchArgument(
            'hardware_backend',
            default_value='fake',
            choices=['fake', 'topic'],
            description=(
                'ros2_control backend: fake (mock_components, default) or '
                'topic (agx_topic_hardware: state from real feedback).'
            ),
        ),
        DeclareLaunchArgument(
            'frame_prefix',
            default_value='arm1/',
            description='TF frame prefix published by robot_state_publisher.',
        ),
        DeclareLaunchArgument(
            'follow',
            default_value='true',
            choices=['true', 'false'],
            description=(
                'true: real arm (/feedback/joint_states); false: bench/mock '
                '(/control/joint_states). Drives rsp, Servo joint_topic and '
                'the lgw node pose_watch_topic together.'
            ),
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            choices=['true', 'false'],
            description='Start MoveIt RViz.',
        ),
        DeclareLaunchArgument(
            'servo_singularity_lower',
            default_value='60.0',
            description=(
                'Jacobian condition number above which Servo starts decelerating. '
                'moveit_servo default is 17.0; raised because ordinary lookat poses '
                'sit above it and would crawl. 60.0 since 2026-11-09: the solver '
                'accepts up to posture_cond_max 50, and with both at 50 an accepted '
                'pose could land exactly on the deceleration boundary.'
            ),
        ),
        DeclareLaunchArgument(
            'servo_singularity_hard_stop',
            default_value='100.0',
            description=(
                'Condition number above which Servo halts (moveit_servo default 30). '
                'Raised so wrist-close poses are still reachable; joint velocity '
                'limits and w_max remain the real safety bound.'
            ),
        ),
        DeclareLaunchArgument(
            'servo_v_max',
            default_value='0.90',
            description=(
                'Control-law linear speed cap (m/s). This is what actually limits '
                'how fast Servo drives the arm in speed_units mode. Servo '
                'attenuates the commanded speed by ~0.5 at publish_period=0.034 '
                '(measured 35 ms internal lag), so the EFFECTIVE ceiling is '
                'v_max/2: 0.90 => ~0.45 m/s, which is Go2 walking speed. The old '
                '0.45 gave only ~0.22 m/s and could never hold a world-fixed '
                'target while walking. Keep in sync with servo_node.yaml.'
            ),
        ),
        DeclareLaunchArgument(
            'servo_w_max',
            default_value='1.0',
            description=(
                'Control-law angular speed cap (rad/s). Keep in sync with '
                'servo_node.yaml.'
            ),
        ),
        DeclareLaunchArgument(
            'hold_recovery_enable',
            default_value='true',
            choices=['true', 'false'],
            description=(
                'Bounded MoveGroup recovery from HOLD (spec 2026-11-03). '
                'false restores the pre-recovery behaviour exactly.'
            ),
        ),
        DeclareLaunchArgument(
            'recovery_allow_target_lost',
            default_value='false',
            choices=['true', 'false'],
            description=(
                'Allow a recovery move while the target is lost, using the '
                'last known geometry (<= recovery_geometry_max_age_ms).'
            ),
        ),
        DeclareLaunchArgument(
            'recovery_deadline_ms',
            default_value='8000',
            description=(
                'Overall HOLD recovery budget. Lower it to exercise the '
                'deadline-cancel path in a bench.'
            ),
        ),
        DeclareLaunchArgument(
            'posture_trace',
            default_value='false',
            description=(
                'Log why each tick picked its wrist branch (5 Hz). '
                'Diagnostics only; leave false in normal operation.'
            ),
        ),
        DeclareLaunchArgument(
            'servo_joint_domain',
            default_value='control',
            choices=['control', 'feedback'],
            description=(
                'What moveit_servo.joint_topic points at. control = '
                '/control/joint_states (JTC/mock, decision A\'): the command is '
                'anchored to the mock so it can lead the real arm. feedback = '
                '/feedback/joint_states (real arm, MoveIt default): the command '
                'is re-anchored to the lagging real arm every cycle. Switch to '
                'feedback to A/B the state source when diagnosing speed.'
            ),
        ),
        DeclareLaunchArgument(
            'servo_velocity_scaling',
            default_value='0.8',
            description=(
                'Sets moveit_servo.override_velocity_scaling_factor. Any value '
                '>= 0.01 BYPASSES Servo internal getVelocityScalingFactor() and '
                'caps joint velocity at this fraction of the joint limits '
                '(5.0 rad/s); 0.0 = no override, i.e. Servo applies its own '
                'scaling. Measured 2026-10-30: the internal scaling was cutting '
                'the commanded motion to ~0.66 (see '
                'docs/servo/2026-10-30-servo-speed-attenuation.md), so 0.8 both '
                'restores most of the speed and keeps an 80% joint-limit guard. '
                'Do NOT use 1.0: at full scaling the arm slams audibly when the '
                'stack brings it back to home. '
                'NOTE: moveit_servo.scale.linear / .scale.rotational are IGNORED '
                'with command_in_type:=speed_units (servo_calcs.cpp) — they only '
                'apply in unitless mode.'
            ),
        ),
        DeclareLaunchArgument(
            'use_bt',
            default_value='true',
            choices=['true', 'false'],
            description=(
                'Include lgw_bt_planner.launch.py (bt_runner). Bench servo '
                'runs pass use_bt:=false.'
            ),
        ),

        # demo.launch.py owns rsp + move_group + JTC + broadcaster — do not add
        # a controller_manager or a second RSP anywhere below.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(demo_launch),
            launch_arguments={
                'arm_type': LaunchConfiguration('arm_type'),
                'effector_type': LaunchConfiguration('effector_type'),
                'revo2_type': LaunchConfiguration('revo2_type'),
                'tcp_offset': LaunchConfiguration('tcp_offset'),
                'hardware_backend': LaunchConfiguration('hardware_backend'),
                'frame_prefix': LaunchConfiguration('frame_prefix'),
                'follow': LaunchConfiguration('follow'),
                'use_rviz': LaunchConfiguration('use_rviz'),
            }.items(),
        ),

        OpaqueFunction(function=_launch_servo_stack),
    ])
