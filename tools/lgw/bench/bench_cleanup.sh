#!/usr/bin/env bash
# Tear down a bench run without `pkill -f ros` (banned: it also kills the
# tracker, the Go2 bridge and anything else that mentions ros).
#
# Lives in its own file on purpose: the PIDs are collected by pgrep, and a
# killer whose own command line contains the patterns would match itself and
# die on the first kill (exit 143, most of the stack left running). Running
# this file keeps the shell's command line to a path.
set -uo pipefail

PATTERNS=(
  'lgw_chain.launch.py'
  'lgw_full.launch.py'   # deprecated alias
  'lgw_servo_stack.launch.py'
  'unified_track_node'
  'lgw_bt_planner/lib/lgw_bt_planner/servo_node'
  'moveit_servo/servo_node_main'
  'moveit_ros_move_group/move_group'
  'controller_manager/ros2_control_node'
  'robot_state_publisher'
  'apply_head_camera_collision_node'
  'tf2_ros/static_transform_publisher'
  'fake_track_publisher.py'
  # Several stack restarts in a row leak odom TF broadcasters: more than one
  # publisher of odom -> base_link makes the target's base-frame position
  # jitter, which the look-at solver turns into visible arm motion.
  'odom_tf_bridge.py'
)

for pattern in "${PATTERNS[@]}"; do
  for pid in $(pgrep -f "${pattern}" 2>/dev/null); do
    kill -15 "${pid}" 2>/dev/null || true
  done
done

sleep 3

# Anything still alive after SIGTERM is stuck in a C call; SIGKILL by PID.
for pattern in "${PATTERNS[@]}"; do
  for pid in $(pgrep -f "${pattern}" 2>/dev/null); do
    kill -9 "${pid}" 2>/dev/null || true
  done
done

sleep 1
remaining=$(ps aux | grep -E '[s]ervo_node|[m]ove_group|[r]os2_control_node|[s]tatic_transform_publisher|[f]ake_track_publisher' | wc -l)
echo "[cleanup] remaining bench processes: ${remaining}"
