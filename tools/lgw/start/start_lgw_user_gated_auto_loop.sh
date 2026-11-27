#!/bin/bash
# scripts/start_lgw_user_gated_auto_loop.sh
#
# 一键启动 UserGatedAutoPickAndPlace BT(ReactiveFallback + Repeat num_cycles="-1")。
# 用户不主动发 /bt_user_input 时,机械臂循环 7 步 pick-and-place;用户发
# std_msgs/Bool data:true → ReactiveFallback 整树 SUCCESS,RosActionNode
# 基类对 MoveArmToPose/ToNamed 已做 async_cancel_goal,arm 在当前帧停。
#
# 这是 scripts/start_lgw_pose_replay.sh 的 fork 专用变种(只换 BT_XML/
# log 路径,其他保持对齐 —— 同一份 cleanup / bringup 约定)。
#
# 跟 start_lgw_pose_replay.sh 的关键差异:
#   - WS_FORK(本 ws)硬编码,而不是默认 lg_walker_ws(主 ws)
#   - BT_XML 默认 user_gated_auto_pick_and_place.xml (TreeID=UserGatedAutoPickAndPlace)
#   - .run 日志目录指向 ${WS_FORK}/.run/
#   - LGW_INSTALL 在 source l4t_setup.sh 之前 export 到 fork,避免被默认
#     覆盖回主 ws(本脚本不用求用户记得 export)
#
# 流程(对齐 start_lgw_pose_replay.sh):
#   1. docker ps / docker start self_l4t
#   2. can_activate.sh can0 1000000       # CAN 总线 up (幂等)
#   3. agx_arm_ctrl/start_single_agx_arm.launch.py  # 硬件 driver
#   4. agx_arm_moveit/launch/demo.launch.py        # rsp + move_group + ros2_control_node
#   5. lgw_bt_planner/launch/bt_with_lgw_moveit.launch.py  # bt_runner
#                                                  起 action server @ /lgw_bt_execution
#
# 启动后:
#   a. 从 ${BT_XML} 的 <BehaviorTree ID="..."> 属性提取 TreeID
#   b. 等 /lgw_bt_execution action server 就绪 (≤30s)
#   c. 前台 sleep infinity 等用户 Ctrl-C → trap 自动按 PID 阶梯清理
#      (用户可在另一终端多次 send_goal / 反复发 user msg halt+re-trigger)
#
# Launch stdout/stderr 重定向到
#   /my_ws/l4t/lg_walker_ws/.run/lgw_launch_<ts>.log
#
# 硬件前置(真机)—— 比 start_lgw_pose_replay.sh **更严**:
#   - 机械臂 + 夹爪 + CAN 总线已上电
#   - 周围 1.0 m 范围内无障碍物(无限循环 → 每次 cycle 都会到 pick/place 目标)
#   - E-stop 按钮随手可触;**真机调试时不要离开控制台**
#   - Go2 driver + 硬件 bringup 已在 T1/T2 终端启动
#   - 确认 MoveArmToPose IK seed 接 /feedback/joint_states(agx_arm IK stability
#     三件套:ompl jiggle=0, kinematics_solver_timeout=0.05+,
#     /feedback/joint_states 作 seed)
#
# 用法(host 上跑):
#   ./scripts/start_lgw_user_gated_auto_loop.sh
#   # 或覆盖默认 BT_XML:
#   BT_XML=/my_ws/l4t/lg_walker_ws/install/lgw_bt_planner/share/lgw_bt_planner/bt_trees/pick_and_place_demo.xml \
#     ./scripts/start_lgw_user_gated_auto_loop.sh
# 退出: Ctrl-C(trap 自动按 PID 阶梯清理 launch 进程,禁 pkill)
#
# 停止机械臂循环(不终止 launch):
#   在另一终端:
#     docker exec -it ${CONTAINER} bash -lc '
#       ros2 topic pub --once /bt_user_input std_msgs/Bool "data: true"
#     '
#   ReactiveFallback Child 1 SUCCESS → 整树 SUCCESS → Repeat halt → arm 当前帧停。

set -u
CONTAINER="self_l4t"

# 本 ws 路径(host 写 /home/unitree/...,容器写 /my_ws/...;本脚本主要在 host 跑,
# 但 docker exec -e ... / docker exec bash -lc "..." 传给容器内 bash 时用容器路径)
# shellcheck source=../_common.sh
source "$(dirname "${BASH_SOURCE[0]}")/../_common.sh"

# LG stack 全部相关进程的统一 grep pattern (cleanup_on_signal / STRAY 检查 / 自动
# 清理共用)。沿用 start_lgw_pose_replay.sh 的 LG_STACK_PIDS_PATTERN,严格按进
# 程名 / 路径字串匹配,禁用 pkill / killall (CLAUDE.md 反模式)。
readonly LG_STACK_PIDS_PATTERN='bt_with_lgw_moveit\.launch\.py|demo\.launch\.py|start_single_agx_arm|agx_arm_ctrl_single|bt_runner|move_group|robot_state_publisher|ros2_control_node|controller_manager|spawner_joint_state_broadcaster|spawner_arm_controller|spawner_gripper_controller'

# ---- Ctrl-C 自动清理 ----
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

# 自动清理 LG stack 残留进程(同上,沿用 cleanup_on_signal 同一份 pattern)。
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
  echo "  3s 内 Ctrl-C 可中止(走 cleanup_on_signal);否则自动按 PID 阶梯清理。"
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

# 覆盖默认 BT_XML(可选)。默认指向本 fork 的 user_gated_auto_pick_and_place.xml。
BT_XML="${BT_XML:-${WS_FORK_CT}/install/lgw_bt_planner/share/lgw_bt_planner/bt_trees/user_gated_auto_pick_and_place.xml}"

# 关键:export LGW_INSTALL 指向本 fork,保证容器内 source l4t_setup.sh 后,
# ros2 pkg list / launch 路径都解析到 fork install 而不是主 ws。
# 这一行也覆盖用户提前 export 的 LGW_INSTALL(本脚本就是要固化到 fork)
export LGW_INSTALL="${WS_FORK_CT}/install/setup.bash"

# ---- 前置检查: 容器必须在跑 ----
echo "[1/6] 检查容器 ${CONTAINER} ..."
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

# ---- 激活 CAN 总线 ----
echo "[2/6] 激活 CAN 总线 can0 @ 1 Mbps..."
if ! docker exec "${CONTAINER}" bash /my_ws/l4t/agx_arm_ws/src/agx_arm_ros/scripts/can_activate.sh can0 1000000 2>&1 | tail -3; then
  echo "  ERROR: can_activate.sh 失败。检查:"
  echo "    - 机械臂 + CAN 模块已上电"
  echo "    - 容器内 unitree 有 passwordless sudo (NOPASSWD)"
  echo "    - can_activate.sh 路径: /my_ws/l4t/agx_arm_ws/src/agx_arm_ros/scripts/can_activate.sh"
  exit 1
fi
echo "  OK"

# ---- 前置清理: 检查残留 driver / moveit / bt_runner ----
echo "[3/6] 检查残留进程 (有就自动按 PID 阶梯清,3s panic window)..."
if ! auto_cleanup_stale; then
  exit 1
fi
echo "  OK (无残留)"

# ---- 启动 agx_arm_ctrl hardware driver (L2 session-level) ----
echo "[4/6] 启动 agx_arm_ctrl hardware driver..."
DRIVER_LOG="${WS_FORK_CT}/.run/lgw_driver_$(date +%Y%m%d_%H%M%S).log"
docker exec "${CONTAINER}" mkdir -p "$(dirname "${DRIVER_LOG}")"
docker exec -e LGW_INSTALL="${LGW_INSTALL}" -d "${CONTAINER}" bash -lc "
  source /my_ws/l4t/l4t_setup.sh
  ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
    effector_type:=agx_gripper tcp_offset:='[0.0, 0.0, 0.12, 0.0, 0.0, 0.0]' \
    > '${DRIVER_LOG}' 2>&1
"

WAITED=0
while [ ${WAITED} -lt 15 ]; do
  if docker exec -e LGW_INSTALL="${LGW_INSTALL}" "${CONTAINER}" bash -lc "
    source /my_ws/l4t/l4t_setup.sh
    ros2 node list 2>/dev/null | grep -q '/agx_arm_ctrl_single_node\$'
  " >/dev/null 2>&1; then
    echo "  agx_arm_ctrl_single_node 就绪 (等了 ${WAITED}s)"
    break
  fi
  sleep 1
  WAITED=$((WAITED + 1))
done
if [ ${WAITED} -eq 15 ]; then
  echo "  WARN: 15s 内 driver 未就绪,继续 (后续 launch 启动会失败)"
fi
echo "  driver_log     = ${DRIVER_LOG}"
echo "  OK"

# ---- 提取 TreeID + 启动 launch (后台) ----
echo "[5/6] 启动 launch (后台) 并准备 send_goal..."
TARGET_TREE=$(docker exec "${CONTAINER}" bash -lc "
  grep -oP '<BehaviorTree ID=\"\K[^\"]+' '${BT_XML}' | head -1
")
if [ -z "${TARGET_TREE}" ]; then
  echo "  ERROR: 无法从 ${BT_XML} 提取 <BehaviorTree ID>"
  exit 1
fi
echo "  BT_XML         = ${BT_XML}"
echo "  target_tree    = ${TARGET_TREE}"
echo "  LGW_INSTALL    = ${LGW_INSTALL}"

LAUNCH_LOG="${WS_FORK_CT}/.run/lgw_launch_$(date +%Y%m%d_%H%M%S).log"
docker exec "${CONTAINER}" mkdir -p "$(dirname "${LAUNCH_LOG}")"
echo "  launch_log     = ${LAUNCH_LOG}"

docker exec -e LGW_INSTALL="${LGW_INSTALL}" -d "${CONTAINER}" bash -lc "
  source /my_ws/l4t/l4t_setup.sh
  ros2 launch lgw_bt_planner bt_with_lgw_moveit.launch.py > '${LAUNCH_LOG}' 2>&1
"

# 等 /lgw_bt_execution action server up
WAITED=0
while [ ${WAITED} -lt 30 ]; do
  if docker exec -e LGW_INSTALL="${LGW_INSTALL}" "${CONTAINER}" bash -lc "
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
  echo "  WARN: 30s 内 /lgw_bt_execution 未起来,继续 send_goal (会 FAIL)"
fi

# ---- 提示用户手动 send_goal + user msg halt ----
echo "[6/6] Launch 已就绪 —— 请手动 send_goal。Ctrl-C 终止本脚本 → 自动清理。"
echo ""
echo "  ⚠️  硬件安全自检 (真机必读) — UserGatedAutoPickAndPlace 行为:"
echo "      1) 机械臂 + 夹爪 + CAN 总线已上电"
echo "      2) 周围 1.0 m 范围内无障碍物(每次 cycle 都会到 pick/place 目标)"
echo "      3) E-stop 按钮随手可触;**真机调试时不要离开控制台**"
echo "      4) BT 树会**无限循环** 7 步 pick-and-place (Repeat num_cycles=\"-1\")"
echo "      5) Go2 driver + 硬件 bringup 已在 T1/T2 终端启动"
echo ""
echo "  在另一终端(或本终端新开窗口)执行:"
echo ""
echo "    # 1) 触发 (执行 → 自动评估 BT 树;BehaviorTree 名=\"${TARGET_TREE}\")"
echo "    docker exec -it ${CONTAINER} bash -lc '"
echo "        ros2 action send_goal /lgw_bt_execution \\\\"
echo "          btcpp_ros2_interfaces/action/ExecuteTree \\\\"
echo "          \"{target_tree: ${TARGET_TREE}}\" --feedback"
echo "      '"
echo ""
echo "    # 2) **停止循环**(不终止 launch):发 user msg → ReactiveFallback SUCCESS"
echo "    docker exec -it ${CONTAINER} bash -lc '"
echo "        ros2 topic pub --once /bt_user_input std_msgs/Bool \"data: true\""
echo "      '"
echo "    期望:WaitForUserInput SUCCESS → 整树 SUCCESS → Repeat halt → MoveArmTo{ToPose,ToNamed} halt"
echo "          (RosActionNode 基类已 async_cancel_goal,arm 在当前帧停)"
echo ""
echo "  当前默认 BT_XML: ${BT_XML}"
echo "  (脚本开头可用 BT_XML=... 覆盖;或直接在 send_goal 里改 target_tree)"
echo ""
echo "  install 下所有可用 BT xml + TreeID:"
docker exec "${CONTAINER}" bash -lc '
  for f in '${WS_FORK_CT}'/install/lgw_bt_planner/share/lgw_bt_planner/bt_trees/*.xml; do
    tid=$(grep -oP "<BehaviorTree ID=\"\K[^\"]+" "$f" | head -1)
    printf "    %-32s TreeID=%s\n" "$(basename $f)" "$tid"
  done
'
echo ""
echo "  多次 send_goal 复用同一组 launch 进程,可换 TreeID。"
echo "  Ctrl-C 终止本脚本 → trap 自动按 PID 清理 launch 树。"
echo ""

echo "=== 等 Ctrl-C ==="
sleep infinity