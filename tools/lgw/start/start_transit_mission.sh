#!/usr/bin/env bash

# shellcheck source=../_common.sh
source "$(dirname "${BASH_SOURCE[0]}")/../_common.sh"
# start_transit_mission.sh — 起 TransitMission 需要的两个进程（transit_node + bt_runner）。
#
# 为什么需要包装脚本：bt_runner 要 dlopen go2_transit 的 BT 插件 .so，而
# go2_transit 装在 nav_ws/go2-nav2-terrain-pointlio，不在 .l4t_env.sh 里。
# .l4t_nav_env.sh 已经把这些工作区全叠好了（9 ms 加载）。
#
# 前置：A+B+C 已起（bash /my_ws/l4t/nav_ws/scripts/realstart/start_chain.sh），
#       狗已用遥控器站起来。
#
# 用法（容器内）：
#   bash /my_ws/l4t/lg_walker_ws/scripts/start_transit_mission.sh
#   bash /my_ws/l4t/lg_walker_ws/scripts/start_transit_mission.sh dry_run:=true
#
# 动作名默认 /transit_bt_execution（2026-11-27 起，与机械臂链的
# /lgw_bt_execution 错开，两套 bt_runner 可同时在线）：
#   ros2 action send_goal /transit_bt_execution \
#     btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: TransitMission}"

set -euo pipefail

NAV_ENV="${NAV_ENV:-/my_ws/l4t/.l4t_nav_env.sh}"
if [ ! -f "$NAV_ENV" ]; then
  echo "ERROR: 找不到 $NAV_ENV" >&2
  echo "       先生成：bash /my_ws/l4t/nav_ws/scripts/lgw_nav_env.sh --refresh" >&2
  exit 1
fi

source "$NAV_ENV"
exec ros2 launch lgw_bt_planner transit_mission.launch.py "$@"
