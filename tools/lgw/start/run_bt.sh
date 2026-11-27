#!/usr/bin/env bash
# scripts/run_bt.sh — 触发一棵行为树。
#
# 两种调用方式都能用:
#   宿主机:  bash scripts/run_bt.sh [TreeID]      (自动 docker exec 转进容器)
#   容器内:  /my_ws/l4t/lg_walker_ws/scripts/run_bt.sh [TreeID]
#
# 为什么要有这个脚本(2026-11-18):直接写
#   docker exec -it self_l4t bash -c "source ... && ros2 action send_goal ..."
# 时,只要终端/编辑器把这条长命令折成两行,双引号里的换行会原样传给容器内的
# bash,被当成命令分隔符 —— 现象是 `usage: ros2 action send_goal ...` 加上
# `bash: line 2: btcpp_ros2_interfaces/action/ExecuteTree: No such file` 两条
# 风马牛不相及的报错。封装成脚本后调用侧只有一行,没有嵌套引号。
# 2026-11-18 补:在宿主机误跑时脚本会自己转进容器,不再报"缺少 .l4t_env.sh"。
#
# 参数:行为树 ID(= XML 里的 BehaviorTree ID,不是文件名),默认 TrackCollectReactive。
#
# 终止:Ctrl-C 取消 goal。多数树是 Repeat(-1) 常驻,不会自己结束;日志里会看到
#   "Received request to cancel goal" -> "canceling and halting Behavior Tree"。
#
# 前置:栈已通过 start_lgw_chain.sh 起来(/lgw_bt_execution action server 在线)。
set -euo pipefail

TREE="${1:-TrackCollectReactive}"

# shellcheck source=../_common.sh
source "$(dirname "${BASH_SOURCE[0]}")/../_common.sh"

# ---- 宿主机调用:转发到容器内执行 ----
if [ ! -f /my_ws/l4t/.l4t_env.sh ]; then
  self_path="$(readlink -f "$0")"
  case "$self_path" in
    "$HOST_MOUNT"/*) ct_path="${CT_WS}${self_path#"$HOST_MOUNT"}" ;;
    *) echo "[run_bt] 既不在容器内、也不在 ${HOST_WS} 下,无法定位容器路径: $self_path" >&2
       exit 1 ;;
  esac
  tty_flags="-i"
  if [ -t 0 ] && [ -t 1 ]; then tty_flags="-it"; fi
  echo "[run_bt] 宿主机调用 -> docker exec ${CONTAINER} ${ct_path}"
  exec docker exec ${tty_flags} "$CONTAINER" "$ct_path" "$TREE"
fi

# ---- 容器内 ----
if [ ! -f /my_ws/l4t/.l4t_env.sh ]; then
  echo "[run_bt] 缺少 /my_ws/l4t/.l4t_env.sh;先跑 l4t/lg_walker_ws/scripts/lgw_env.sh --refresh" >&2
  exit 1
fi
# 4 ms 的环境缓存;只有装了新包才需要 --refresh 重建。
source /my_ws/l4t/.l4t_env.sh

echo "[run_bt] target_tree=${TREE} (Ctrl-C 取消)"
exec ros2 action send_goal /lgw_bt_execution \
  btcpp_ros2_interfaces/action/ExecuteTree \
  "{target_tree: \"${TREE}\"}" --feedback
