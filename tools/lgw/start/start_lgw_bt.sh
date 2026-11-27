#!/bin/bash
# scripts/start_lgw_bt.sh
#
# 行为树 (BT) 初始化:启动 lgw_bt_planner.launch.py(bind-mount 里也跑
# select_arm_observation_pose_service_node),等 /lgw_bt_execution action
# server 和 /select_arm_observation_pose service 都 UP,印出 send_goal
# 指令,保持前台。
#
# 前置依赖(必须先在另外 2 个终端跑起来,本脚本只做 precheck,不会自己起):
#   - start_lgw_arm.sh   → /move_action action server 必须就绪
#   - start_lgw_yoloe.sh → /perception/two_stage/camera_{head,arm}/result
#                          必须有连续推理流(看 .run/lgw_yoloe_current.log)
#
# 启动项(对应原 start_lgw_head_arm_independent_lookat.sh 的 Step 3b +
# /select_arm_observation_pose wait + Phase Y3.5):
#   precheck  验 /move_action + YOLOE CONTINUOUS_RESULT 流
#   Y3.5      校验 YOLOE 日志近 30 行 ≥3 次 CONTINUOUS_RESULT (head + arm)
#   Step 3b   启动 BT runner (lgw_bt_planner.launch.py)
#   wait 1    /lgw_bt_execution action server
#   wait 2    /select_arm_observation_pose service
#
# Env override(可选):
#   BT_XML=...  默认 head_arm_independent_lookat.xml
#               (即原脚本 DEFAULT_BT_XML,TreeID=HeadArmIndependentLookat)
#
# 终止:Ctrl-C → cleanup_bt kill 容器内 BT 相关进程(bg_runner +
# select_arm_observation_pose_service_node),不影响 arm / YOLOE。
# 重启本脚本会 preflight 清掉上次 BT 残留,可以在迭代 BT 时反复跑而
# 不动 arm / YOLOE。

set -u
CONTAINER="${CONTAINER:-self_l4t}"

# shellcheck source=../_common.sh
source "$(dirname "${BASH_SOURCE[0]}")/../_common.sh"

# 仅 BT 相关 PID pattern
readonly LG_BT_PIDS_PATTERN='lgw_bt_planner\.launch\.py|bt_with_lgw_moveit\.launch\.py|bt_runner|select_arm_observation_pose_service_node'

# 固定 YOLOE 日志路径(由 start_lgw_yoloe.sh 写入)
readonly YOLOE_LOG_HOST="${WS_FORK_HOST}/.run/lgw_yoloe_current.log"

# BT_XML 默认指向原脚本同款
readonly DEFAULT_BT_XML="${WS_FORK_CT}/install/lgw_bt_planner/share/lgw_bt_planner/bt_trees/head_arm_independent_lookat.xml"
BT_XML="${BT_XML:-${DEFAULT_BT_XML}}"
readonly TREE_ID="HeadArmIndependentLookat"

# ---- Ctrl-C 清理 ----
cleanup_on_signal() {
  if [ "${CLEANUP_DONE:-0}" = "1" ]; then return; fi
  CLEANUP_DONE=1
  echo ""
  echo "[bt-cleanup] Ctrl-C / 信号触发,清理 BT runner + service node..."

  local pids
  pids=$(docker exec "${CONTAINER}" bash -c "
    ps -eo pid,args 2>/dev/null | \
    grep -E '${LG_BT_PIDS_PATTERN}' | \
    grep -v grep | \
    awk '{print \$1}'
  " 2>/dev/null) || pids=""
  if [ -n "${pids}" ]; then
    echo "[bt-cleanup] kill candidates (container pids): ${pids}"
    for pid in ${pids}; do
      docker exec "${CONTAINER}" kill -TERM "${pid}" 2>/dev/null || true
    done
    sleep 3
    for pid in ${pids}; do
      if docker exec "${CONTAINER}" kill -0 "${pid}" 2>/dev/null; then
        docker exec "${CONTAINER}" kill -KILL "${pid}" 2>/dev/null || true
      fi
    done
  fi
  echo "[bt-cleanup] done"
}

# ---- preflight ----
preflight_cleanup() {
  echo "[bt-preflight] 清理容器内 BT 残留进程..."
  local pids
  pids=$(docker exec "${CONTAINER}" bash -c "
    ps -eo pid,args 2>/dev/null | \
    grep -E '${LG_BT_PIDS_PATTERN}' | \
    grep -v grep | \
    awk '{print \$1}'
  " 2>/dev/null) || pids=""
  if [ -n "${pids}" ]; then
    echo "[bt-preflight] candidates: ${pids}"
    for pid in ${pids}; do
      docker exec "${CONTAINER}" kill -TERM "${pid}" 2>/dev/null || true
    done
    sleep 3
    for pid in ${pids}; do
      if docker exec "${CONTAINER}" kill -0 "${pid}" 2>/dev/null; then
        echo "[bt-preflight]   KILL container pid=${pid}"
        docker exec "${CONTAINER}" kill -KILL "${pid}" 2>/dev/null || true
      fi
    done
  fi
  sleep 1
  echo "[bt-preflight] done"
}

trap cleanup_on_signal INT TERM
preflight_cleanup

docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$" \
  || docker start "${CONTAINER}" >/dev/null

export LGW_INSTALL="${WS_FORK_CT}/install/setup.bash"
echo "[bt] LGW_INSTALL=${LGW_INSTALL}"

LOG_TS="$(date +%Y%m%d_%H%M%S)"
docker exec "${CONTAINER}" mkdir -p "${WS_FORK_CT}/.run"
docker exec "${CONTAINER}" chmod 777 "${WS_FORK_CT}/.run" 2>/dev/null || true

# ====================================================================
# precheck:arm + YOLOE 必须先就绪
# ====================================================================
echo "[bt] precheck: 校验 arm + YOLOE 前置依赖..."

# arm: /move_action
WAITED=0
arm_ok=0
while [ ${WAITED} -lt 5 ]; do
  if docker exec "${CONTAINER}" bash -lc "
    source /my_ws/l4t/l4t_setup.sh
    source ${LGW_INSTALL:-${WS_FORK_CT}/install/setup.bash}
    ros2 action list 2>/dev/null | grep -q '^/move_action\$'
  " >/dev/null 2>&1; then
    arm_ok=1
    break
  fi
  sleep 1
  WAITED=$((WAITED + 1))
done
if [ "${arm_ok}" != "1" ]; then
  echo "[bt]   ERROR: /move_action 不在 —— 请先在另一终端跑 start_lgw_arm.sh"
  echo "[bt]           并等到 /move_action action server 就绪"
  exit 1
fi
echo "[bt]   /move_action 已就绪 (waited ${WAITED}s)"

# YOLOE: CONTINUOUS_RESULT 流
[ -f "${YOLOE_LOG_HOST}" ] || {
  echo "[bt]   ERROR: YOLOE 日志不存在 (${YOLOE_LOG_HOST})"
  echo "[bt]           请先在另一终端跑 start_lgw_yoloe.sh"
  exit 1
}

# ====================================================================
# Phase Y3.5: 校验 CONTINUOUS_RESULT 流(head + arm 各 ≥3 次,近 30 行)
# 替代原 ros2 topic info polling 的不可靠实现(详见原脚本注释)。
# ====================================================================
echo "[bt] Phase Y3.5: verify CONTINUOUS_RESULT 流 (head + arm 各 ≥3 次,日志近 30 行)..."
for cam in head arm; do
  pub_ok=0
  for _ in $(seq 1 30); do
    sleep 1
    # tail -n 30 ≈ 3-6s 运行窗口 (稳态 ~5-10Hz);3+ 次 = 真的在连续
    # 推理,排除冷启动单条 + 立即 hang 的情况。
    recent=$(tail -n 30 "${YOLOE_LOG_HOST}" 2>/dev/null | \
             grep -c "CONTINUOUS_RESULT camera=${cam}" || true)
    if [ "${recent}" -ge 3 ]; then
      pub_ok=1
      break
    fi
  done
  if [ "${pub_ok}" != "1" ]; then
    echo "[bt]   ERROR: ${cam} 相机 30s 内未连续出现 ≥3 次 CONTINUOUS_RESULT"
    echo "[bt]          末 20 行 log 摘录:"
    tail -n 20 "${YOLOE_LOG_HOST}" | sed 's/^/             /'
    exit 1
  fi
  echo "[bt]   ${cam}: CONTINUOUS_RESULT 连续出现 (近 30 行 ≥3 次)"
done

# ====================================================================
# Step 3b: BT runner
# ====================================================================
echo "[bt] Step 3b: 启动 BT runner (lgw_bt_planner.launch.py,detached)..."
LGW_LAUNCH_LOG="${WS_FORK_CT}/.run/lgw_bt.log"
LGW_LAUNCH_LOG_HOST="${WS_FORK_HOST}/.run/lgw_bt.log"
: > "${LGW_LAUNCH_LOG_HOST}" || true
docker exec -e LGW_INSTALL="${LGW_INSTALL}" -e BT_PLANNER_DEBUG=1 -d "${CONTAINER}" bash -lc "
  source /my_ws/l4t/l4t_setup.sh
  # 2026-11-20: bt_runner 的 plugins 里现在有 go2_transit/bt_plugins
  # (TrackCollectReactiveV2 的 init 末尾 <SubTree ID=\"TransitMission\"/>)。
  # go2_transit 在 nav_ws 里, l4t_setup.sh 不带它; 不 source 的话
  # RegisterPlugins 会静默跳过该插件 -> transit_mission.xml 解析失败
  # ('Node not recognized: TransitToLocalPoint') -> V2 创建时就报
  # 'Can't find tree TransitMission'。nav env 放在 lgw install 之前 source,
  # 让 lgw_walker_ws 的前缀仍然最后生效。
  if [ -f /my_ws/l4t/.l4t_nav_env.sh ]; then
    source /my_ws/l4t/.l4t_nav_env.sh
  else
    echo '[bt] WARN: /my_ws/l4t/.l4t_nav_env.sh 缺失, go2_transit 插件不可用'
  fi
    source ${LGW_INSTALL:-${WS_FORK_CT}/install/setup.bash}
  ros2 launch lgw_bt_planner lgw_bt_planner.launch.py \
    >> '${LGW_LAUNCH_LOG}' 2>&1
"
echo "[bt]   bt log → ${LGW_LAUNCH_LOG_HOST}"

WAITED=0
while [ ${WAITED} -lt 30 ]; do
  if docker exec "${CONTAINER}" bash -lc "
    source /my_ws/l4t/l4t_setup.sh
    source ${LGW_INSTALL:-${WS_FORK_CT}/install/setup.bash}
    ros2 action list 2>/dev/null | grep -q '^/lgw_bt_execution\$'
  " >/dev/null 2>&1; then
    echo "[bt]   /lgw_bt_execution action server 就绪 (等了 ${WAITED}s)"
    break
  fi
  sleep 1
  WAITED=$((WAITED + 1))
done
if [ ${WAITED} -eq 30 ]; then
  echo "[bt]   WARN: 30s 内 action server 未起来"
fi

# /select_arm_observation_pose service 由 lgw_bt_planner.launch.py 自动起
WAITED=0
while [ ${WAITED} -lt 10 ]; do
  if docker exec "${CONTAINER}" bash -lc "
    source /my_ws/l4t/l4t_setup.sh
    source ${LGW_INSTALL:-${WS_FORK_CT}/install/setup.bash}
    ros2 service list 2>/dev/null | grep -q '^/select_arm_observation_pose\$'
  " >/dev/null 2>&1; then
    echo "[bt]   /select_arm_observation_pose service 就绪 (等了 ${WAITED}s)"
    break
  fi
  sleep 1
  WAITED=$((WAITED + 1))
done
if [ ${WAITED} -eq 10 ]; then
  echo "[bt]   WARN: 10s 内 /select_arm_observation_pose service 未起来"
  echo "[bt]          BT 走到 SelectArmObservationPose 会 FAILURE"
fi

# ---- 提示 ----
echo ""
echo "[bt] Launch 已就绪 —— /lgw_bt_execution action server 已 UP。"
echo "     保持本脚本运行(不要 Ctrl-C);另起终端 send_goal 启动循环。"
echo ""
echo "  ✅ 硬件安全自检 (真机必读):"
echo "      1) 机械臂 + 夹爪 + CAN 总线已上电(由 start_lgw_arm.sh 保证)"
echo "      2) 周围 0.5 m 范围内无障碍物 / 无关人员"
echo "      3) E-stop 按钮随手可触"
echo "      4) BT 树会真实移动机械臂 (每 cycle head→arm→home)"
echo "      5) YOLOE 推理流已持续(由 start_lgw_yoloe.sh 保证)"
echo ""
echo "  日志:"
echo "    BT (bt_runner + select_arm service + Compute* + MoveArmTo*):"
echo "      ${LGW_LAUNCH_LOG_HOST}"
echo ""
echo "  当前默认 BT_XML: ${BT_XML}"
echo "  (脚本开头可用 BT_XML=... 覆盖)"
echo ""
echo "  install 下所有可用 BT xml + TreeID:"
docker exec "${CONTAINER}" bash -lc '
  for f in ${WS_FORK_CT}/install/lgw_bt_planner/share/lgw_bt_planner/bt_trees/*.xml; do
    tid=$(grep -oP "<BehaviorTree ID=\"\K[^\"]+" "$f" | head -1)
    printf "    %-32s TreeID=%s\n" "$(basename $f)" "$tid"
  done
' 2>/dev/null || echo "    (failed to list BT trees)"
echo ""
echo "  启动 BT(在另一终端执行,容器内 source 好 l4t_setup.sh):"
echo ""
echo "    docker exec -it ${CONTAINER} bash -lc '"
echo "      source /my_ws/l4t/l4t_setup.sh
    source ${LGW_INSTALL:-${WS_FORK_CT}/install/setup.bash}"
echo "      ros2 action send_goal /lgw_bt_execution \\"
echo "        btcpp_ros2_interfaces/action/ExecuteTree \\"
echo "        \"{\\\"target_tree\\\": \\\"${TREE_ID}\\\"}\" --feedback"
echo "    '"
echo ""
echo "  取消 goal(在 send_goal 那个终端按 Ctrl-C 即可),BT 停止循环"
echo "  → arm 停在当前 step 完成的 pose(不会自动回 home)。"
echo ""
echo "  本脚本 Ctrl-C 仍然会清理整个 BT launch 树(且会顺带 cancel action)。"
echo ""
echo ""
echo "=== 等 Ctrl-C ==="
sleep infinity
