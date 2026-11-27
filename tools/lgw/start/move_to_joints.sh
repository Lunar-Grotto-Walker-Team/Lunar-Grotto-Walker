#!/usr/bin/env bash

# shellcheck source=../_common.sh
source "$(dirname "${BASH_SOURCE[0]}")/../_common.sh"
# Move the arm to an explicit joint configuration via MoveGroup.
#
# Usage (inside self_l4t):
#   bash scripts/move_to_joints.sh 0.0 1.2 -1.2 0.0 0.95 0.0
#
# Velocity scaling defaults to 15% for bench work on real hardware. Pass a
# 7th argument to override it (1.0 = the 5 rad/s MoveIt joint limit) — used by
# the arm speed-limit probe.
set -euo pipefail

if [ "$#" -ne 6 ] && [ "$#" -ne 7 ]; then
  echo "usage: $0 j1 j2 j3 j4 j5 j6 [velocity_scaling]" >&2
  exit 2
fi

read -r J1 J2 J3 J4 J5 J6 <<< "$1 $2 $3 $4 $5 $6"
SCALE="${7:-0.15}"

read -r -d '' GOAL <<EOF || true
{
  request: {
    group_name: 'arm',
    num_planning_attempts: 10,
    allowed_planning_time: 5.0,
    max_velocity_scaling_factor: ${SCALE},
    max_acceleration_scaling_factor: ${SCALE},
    goal_constraints: [{
      joint_constraints: [
        {joint_name: 'joint1', position: ${J1}, tolerance_above: 0.02, tolerance_below: 0.02, weight: 1.0},
        {joint_name: 'joint2', position: ${J2}, tolerance_above: 0.02, tolerance_below: 0.02, weight: 1.0},
        {joint_name: 'joint3', position: ${J3}, tolerance_above: 0.02, tolerance_below: 0.02, weight: 1.0},
        {joint_name: 'joint4', position: ${J4}, tolerance_above: 0.02, tolerance_below: 0.02, weight: 1.0},
        {joint_name: 'joint5', position: ${J5}, tolerance_above: 0.02, tolerance_below: 0.02, weight: 1.0},
        {joint_name: 'joint6', position: ${J6}, tolerance_above: 0.02, tolerance_below: 0.02, weight: 1.0}
      ]
    }]
  }
}
EOF

ros2 action send_goal /move_action moveit_msgs/action/MoveGroup "${GOAL}" | tail -2
