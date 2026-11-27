#!/bin/bash
# scripts/start_lgw_go2_posture_demo.sh
#
# 一键启动 Go2 起身/平衡/趴下 demo —— 验证 StandUp / BalanceStand / StandDown
# 三个 BT 节点在真机上的行为(单 Sequence:起 → 站 → 睡 2s → 趴)。
#
# 这是 scripts/start_lgw_lookat_pose_isolated_test.sh 的极简变种:
#   - 不带 CAN / agx_arm_ctrl / MoveIt / cameras / YOLOE(本脚本只跑 BT runner)
#   - Go2 driver 由用户在另一终端单独起(走 cyclonedds + eth10)
#   - BT_XML 默认指向 fork install 的 go2_posture_demo.xml,TreeID = Go2PostureDemo
#   - LGW_INSTALL 在 source l4t_setup.sh 之前 export 指向 fork,避免
#     l4t_setup.sh 默认覆盖回主 ws (l4t_setup.sh:20 LGW_INSTALL 默认值)
#   - 流程与 reference 对齐:docker ps → 残留清理 → launch → action server wait
#     → 提示 send_goal → sleep infinity → Ctrl-C trap 清理
#
# 启动后本脚本:
#   a. 从 ${BT_XML} 的 <BehaviorTree ID="..."> 属性提取 TreeID
#   b. 等 /lgw_bt_execution action server 就绪 (≤30s)
#   c. 前台 sleep infinity 等用户 Ctrl-C → trap 自动清理 launch 树
#
# 用法 (host 上跑):
#   ./scripts/start_lgw_go2_posture_demo.sh
#   # 或覆盖默认 BT_XML: BT_XML=/path/to/other.xml ./scripts/start_lgw_go2_posture_demo.sh
#
# 退出: Ctrl-C (前台 sleep 等用户,trap 自动按 PID 阶梯清理 launch 进程)
#
# send_goal 在另一终端:
#   docker exec -it self_l4t bash -lc 'source /my_ws/l4t/l4t_setup.sh \
#     && ros2 action send_goal /lgw_bt_execution \
#         btcpp_ros2_interfaces/action/ExecuteTree \
#         "{\"target_tree\": \"Go2PostureDemo\"}" --feedback'
#
# ⚠ 硬件安全(真机必读):
#   - Go2 已上电,driver 已在另一终端起来(本脚本不启 Go2 driver —— 用户自行起)
#   - 周围 1.5 m 范围内无障碍 / 无关人员(Go2 起/趴要净空,起身动作腿部伸展)
#   - E-stop 按钮随手可触
#   - BT 树会真实起/趴 Go2(StandUp / StandDown),运动前需确保 Go2 不在充电座上
#
# 本脚本只管 BT runner stack(bt_runner + plugin + BT XML)。
# Go2 driver / 硬件 bringup 不在本脚本范围 —— 真机部署前请按 runbook 在
# T1/T2 终端分别启动 (e.g. ros2 launch go2_ros2_sdk go2_ros2.launch.py 等)。

set -u
CONTAINER="self_l4t"

# Fork 路径(host / container view)
# shellcheck source=../_common.sh
source "$(dirname "${BASH_SOURCE[0]}")/../_common.sh"

# 关键:export LGW_INSTALL 指向 fork,保证容器内 source l4t_setup.sh 后,
# ros2 pkg list / launch 路径 / action 类型都解析到 fork install 而不是
# 默认主 ws (l4t_setup.sh:20 LGW_INSTALL 默认 = /my_ws/l4t/lg_walker_ws/install/setup.bash)。
export LGW_INSTALL="${WS_FORK_CT}/install/setup.bash"
echo "[bringup] LGW_INSTALL=${LGW_INSTALL}"

# BT runner 进程 + launch 进程统一 grep pattern (cleanup_on_signal / auto_cleanup_stale 共用)。
# go2 posture demo 不启 MoveIt/arm/can,只跑 bt_runner 一个 ROS 节点 +
# 上层的 ros2 launch 进程。严格按 PID 阶梯清理 (禁用 pkill / killall,CLAUDE.md 反模式)。
readonly LG_STACK_PIDS_PATTERN='go2_posture_demo\.launch\.py|bt_runner'

# ---- Ctrl-C 自动清理 ----
# 收到 SIGINT/SIGTERM 时,按 PID 阶梯清理容器内 BT runner launch 树:
# TERM → 3s → KILL,一次一个 PID,禁用 pkill。
cleanup_on_signal() {
  if [ "${CLEANUP_DONE:-0}" = "1" ]; then
    return
  fi
  CLEANUP_DONE=1

  echo ""
  echo "[cleanup] Ctrl-C / 信号触发,清理 launch 进程..."

  local pids
  pids=$(docker exec "${CONTAINER}" bash -lc "
    ps -ef | grep -E '${LG_STACK_PIDS_PATTERN}' | grep -v grep | awk '{print \$2}'
  " 2>/dev/null) || pids=""

  if [ -z "${pids}" ]; then
    echo "  无残留进程"
    exit 130
  fi

  local count
  count=$(echo "${pids}" | wc -w)
  echo "  找到 ${count} 个进程,先 TERM:"

  for pid in ${pids}; do
    echo "    TERM ${pid}"
    docker exec "${CONTAINER}" kill -TERM "${pid}" 2>/dev/null || true
  done

  sleep 3

  echo "  3s 后仍活的 KILL:"
  for pid in ${pids}; do
    if docker exec "${CONTAINER}" kill -0 "${pid}" 2>/dev/null; then
      echo "    KILL ${pid}"
      docker exec "${CONTAINER}" kill -KILL "${pid}" 2>/dev/null || true
    fi
  done

  echo "  清理完成"
  exit 130
}

# 自动清理 BT runner 残留进程 (复用 cleanup_on_signal 同一份 LG_STACK_PIDS_PATTERN)。
# 列 PID → 3s panic window (用户 Ctrl-C 可中止,bash trap 会转 cleanup_on_signal
# 重复 TERM 后 exit 130) → TERM → 3s grace → KILL → 二次 grep 验证。
auto_cleanup_stale() {
  local pids
  pids=$(docker exec "${CONTAINER}" bash -lc "
    ps -ef | grep -E '${LG_STACK_PIDS_PATTERN}' | grep -v grep | awk '{print \$2}'
  " 2>/dev/null) || pids=""

  if [ -z "${pids}" ]; then
    return 0
  fi

  local count
  count=$(echo "${pids}" | wc -w)
  echo "  找到 ${count} 个残留进程 (上次 session 留下的):"
  docker exec "${CONTAINER}" bash -lc "
    ps -ef | grep -E '${LG_STACK_PIDS_PATTERN}' | grep -v grep | sed 's/^/    /'
  "
  echo ""
  echo "  3s 内 Ctrl-C 可中止 (走 cleanup_on_signal);否则自动按 PID 阶梯清理。"
  echo -n "  等 3s"
  for _ in 1 2 3; do
    sleep 1
    echo -n "."
  done
  echo ""

  echo "  开始 TERM:"
  for pid in ${pids}; do
    echo "    TERM ${pid}"
    docker exec "${CONTAINER}" kill -TERM "${pid}" 2>/dev/null || true
  done

  sleep 3

  echo "  3s 后仍活的 KILL:"
  for pid in ${pids}; do
    if docker exec "${CONTAINER}" kill -0 "${pid}" 2>/dev/null; then
      echo "    KILL ${pid}"
      docker exec "${CONTAINER}" kill -KILL "${pid}" 2>/dev/null || true
    fi
  done

  sleep 1
  local remaining
  remaining=$(docker exec "${CONTAINER}" bash -lc "
    ps -ef | grep -E '${LG_STACK_PIDS_PATTERN}' | grep -v grep | awk '{print \$2}'
  " 2>/dev/null) || remaining=""

  if [ -n "${remaining}" ]; then
    echo "  ERROR: 自动清理后仍存活 (PID: ${remaining}),请手动检查。"
    return 1
  fi

  echo "  清理完成 (${count} 个)"
  return 0
}

trap cleanup_on_signal INT TERM

# 默认 BT_XML = fork install 下的 go2_posture_demo 树
# (TreeID = Go2PostureDemo)
BT_XML="${BT_XML:-${WS_FORK_CT}/install/lgw_bt_planner/share/lgw_bt_planner/bt_trees/go2_posture_demo.xml}"

# ---- [1/4] 前置检查:容器必须在跑 ----
echo "[1/4] 检查容器 ${CONTAINER} ..."
if ! docker ps --format '{{.Names}}' 2>/dev/null | grep -q "^${CONTAINER}$"; then
  echo "  容器未运行,尝试 docker start ${CONTAINER} ..."
  if ! docker start "${CONTAINER}" >/dev/null 2>&1; then
    echo "  ERROR: docker start ${CONTAINER} 失败"
    exit 1
  fi
  sleep 2
  if ! docker ps --format '{{.Names}}' 2>/dev/null | grep -q "^${CONTAINER}$"; then
    echo "  ERROR: ${CONTAINER} 启动后仍不在运行列表"
    exit 1
  fi
fi
echo "  OK"

# ---- [2/4] 前置清理:残留 bt_runner ----
echo "[2/4] 检查残留进程 (有就自动按 PID 阶梯清,3s panic window)..."
if ! auto_cleanup_stale; then
  exit 1
fi
echo "  OK (无残留)"

# ---- [3/4] 提取 TreeID + 启动 launch (后台) ----
echo "[3/4] 启动 launch (后台) 并准备 send_goal..."
TARGET_TREE=$(docker exec "${CONTAINER}" bash -lc "
  grep -oP '<BehaviorTree ID=\"\K[^\"]+' '${BT_XML}' | head -1
")
if [ -z "${TARGET_TREE}" ]; then
  echo "  ERROR: 无法从 ${BT_XML} 提取 <BehaviorTree ID>"
  exit 1
fi
echo "  BT_XML      = ${BT_XML}"
echo "  target_tree = ${TARGET_TREE}"

LAUNCH_LOG="${WS_FORK_CT}/.run/lgw_go2_posture_demo_launch_$(date +%Y%m%d_%H%M%S).log"
LAUNCH_LOG_HOST="${WS_FORK_HOST}/.run/lgw_go2_posture_demo_launch_$(date +%Y%m%d_%H%M%S).log"
docker exec "${CONTAINER}" mkdir -p "$(dirname "${LAUNCH_LOG}")"
echo "  launch_log  = ${LAUNCH_LOG_HOST}"

# detached launch —— 注意 -e LGW_INSTALL + source l4t_setup.sh (否则走 fastrtps + 无 CYCLONEDDS_URI)
docker exec -e LGW_INSTALL="${LGW_INSTALL}" -e BT_PLANNER_DEBUG=1 -d "${CONTAINER}" bash -lc "
  source /my_ws/l4t/l4t_setup.sh
  ros2 launch lgw_bt_planner go2_posture_demo.launch.py > '${LAUNCH_LOG}' 2>&1
"

# 等 /lgw_bt_execution action server up (bt_runner 加载 .so/.xml 都需要时间)
WAITED=0
while [ ${WAITED} -lt 30 ]; do
  if docker exec "${CONTAINER}" bash -lc "
    source /my_ws/l4t/l4t_setup.sh
    ros2 action list 2>/dev/null | grep -q '^/lgw_bt_execution\$'
  " >/dev/null 2>&1; then
    echo "  /lgw_bt_execution action server 就绪 (等了 ${WAITED}s)"
    break
  fi
  sleep 1
  WAITED=$((WAITED + 1))
done
if [ ${WAITED} -eq 30 ]; then
  echo "  WARN: 30s 内 action server 未起来,后续 send_goal 会 FAIL"
fi

# ---- [4/4] 提示用户手动 send_goal ----
echo "[4/4] Launch 已就绪 —— 请手动 send_goal。Ctrl-C 终止本脚本 → 自动清理。"
echo ""
echo "  ✅ 硬件安全自检 (真机必读):"
echo "      1) Go2 已上电,driver 已在另一终端起来(走 cyclonedds + eth10)"
echo "      2) 周围 1.5 m 范围内无障碍物 / 无关人员(Go2 起/趴腿部要净空)"
echo "      3) E-stop 按钮随手可触"
echo "      4) BT 树会真实起/趴 Go2(StandUp / StandDown),中途 Ctrl-C 会在"
echo "         /api/sport/request 上发 1003(StopMove)"
echo ""
echo "  在另一终端(或本终端新开窗口)执行:"
echo ""
echo "    docker exec -e LGW_INSTALL=\"${LGW_INSTALL}\" -it ${CONTAINER} bash -lc '"
echo "      export LGW_INSTALL=\"${LGW_INSTALL}\""
echo "      source /my_ws/l4t/l4t_setup.sh"
echo "      ros2 action send_goal /lgw_bt_execution \\"
echo "        btcpp_ros2_interfaces/action/ExecuteTree \\"
echo "        \"{\\\"target_tree\\\": \\\"${TARGET_TREE}\\\"}\" --feedback"
echo "    '"
echo ""
echo "  当前默认 BT_XML: ${BT_XML}"
echo "  (脚本开头可用 BT_XML=... 覆盖;或直接在 send_goal 里改 target_tree)"
echo ""
echo "  install 下所有可用 BT xml + TreeID:"
docker exec "${CONTAINER}" bash -lc '
  for f in '"${WS_FORK_CT}"'/install/lgw_bt_planner/share/lgw_bt_planner/bt_trees/*.xml; do
    tid=$(grep -oP "<BehaviorTree ID=\"\K[^\"]+" "$f" | head -1)
    printf "    %-32s TreeID=%s\n" "$(basename $f)" "$tid"
  done
' 2>/dev/null || echo "    (failed to list BT trees)"
echo ""
echo "  多次 send_goal 复用同一组 launch 进程,可换 TreeID。"
echo "  Ctrl-C 终止本脚本 → trap 自动按 PID 清理 launch 树。"
echo ""

# 前台等 Ctrl-C
echo "=== 等 Ctrl-C ==="
sleep infinity