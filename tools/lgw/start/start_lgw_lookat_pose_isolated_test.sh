#!/bin/bash
# scripts/start_lgw_lookat_pose_isolated_test.sh
#
# 一键启动 LG_ws 隔离 test (LookatPoseIsolatedTest) —— 验证
# ComputeLookatPose + MoveArmToPose 在 hardcoded head-frame target 下
# 能否解 IK / ompl。不引入 YOLOE / cameras / head detection 链路。
#
# 这是 scripts/start_lgw_pose_replay.sh 的隔离变种:
#   - 不带 YOLOE / cameras / preflight_cleanup / host PID(本脚本只跑容器内 LG stack)
#   - BT_XML 默认指向 fork install 的 lookat_pose_isolated_test.xml
#   - LGW_INSTALL 在 source l4t_setup.sh 之前 export 指向 fork,避免
#     l4t_setup.sh 默认覆盖回主 ws (l4t_setup.sh:20 LGW_INSTALL 默认值)
#   - 流程与 reference 对齐:docker ps → CAN → driver → combined launch →
#     action server wait → 提示 send_goal → sleep infinity → Ctrl-C trap 清理
#
# 启动后本脚本:
#   a. 从 ${BT_XML} 的 <BehaviorTree ID="..."> 属性提取 TreeID
#   b. 等 /lgw_bt_execution action server 就绪 (≤30s)
#   c. 前台 sleep infinity 等用户 Ctrl-C → trap 自动清理 launch 树
#
# 用法 (host 上跑):
#   ./scripts/start_lgw_lookat_pose_isolated_test.sh
#   # 或覆盖默认 BT_XML: BT_XML=/path/to/other.xml ./scripts/start_lgw_lookat_pose_isolated_test.sh
#
# 退出: Ctrl-C (前台 sleep 等用户,trap 自动按 PID 阶梯清理 launch 进程)
#
# send_goal 在另一终端:
#   docker exec -it self_l4t bash -lc 'source /my_ws/l4t/l4t_setup.sh \
#     && ros2 action send_goal /lgw_bt_execution \
#         btcpp_ros2_interfaces/action/ExecuteTree \
#         "{\"target_tree\": \"LookatPoseIsolatedTest\"}" --feedback'
#
# ⚠ 硬件安全(真机必读):
#   - 机械臂 + 夹爪 + CAN 总线已上电
#   - 周围 0.5 m 范围内无障碍 / 无关人员
#   - E-stop 按钮随手可触
#   - BT 树会真实移动机械臂 (MoveArmToPose + MoveArmToNamed)
#
# 本脚本只管 LG stack (机械臂 + MoveIt + BT runner)。
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

# LG stack 全部相关进程的统一 grep pattern (cleanup_on_signal / 自动清理共用)。
# 严格按 PID 阶梯清理 (禁用 pkill / killall,CLAUDE.md 反模式)。
readonly LG_STACK_PIDS_PATTERN='bt_with_lgw_moveit\.launch\.py|demo\.launch\.py|start_single_agx_arm|agx_arm_ctrl_single|bt_runner|move_group|robot_state_publisher|ros2_control_node|controller_manager|spawner_joint_state_broadcaster|spawner_arm_controller|spawner_gripper_controller'

# ---- Ctrl-C 自动清理 ----
# 收到 SIGINT/SIGTERM 时,按 PID 阶梯清理容器内 LG launch 树:
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

# 自动清理 LG stack 残留进程 (复用 cleanup_on_signal 同一份 LG_STACK_PIDS_PATTERN)。
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

# 默认 BT_XML = fork install 下的 isolation test 树
# (TreeID = LookatPoseIsolatedTest)
BT_XML="${BT_XML:-${WS_FORK_CT}/install/lgw_bt_planner/share/lgw_bt_planner/bt_trees/lookat_pose_isolated_test.xml}"

# ---- [1/6] 前置检查:容器必须在跑 ----
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

# ---- [2/6] 激活 CAN 总线 ----
# start_single_agx_arm.launch.py 假设 can0 已 up(对齐 agx_arm_ros README 约定)。
echo "[2/6] 激活 CAN 总线 can0 @ 1 Mbps..."
if ! docker exec "${CONTAINER}" bash \
    /my_ws/l4t/agx_arm_ws/src/agx_arm_ros/scripts/can_activate.sh can0 1000000 2>&1 | tail -3; then
  echo "  ERROR: can_activate.sh 失败。检查:"
  echo "    - 机械臂 + CAN 模块已上电"
  echo "    - 容器内 unitree 有 passwordless sudo (NOPASSWD)"
  echo "    - can_activate.sh 路径: /my_ws/l4t/agx_arm_ws/src/agx_arm_ros/scripts/can_activate.sh"
  exit 1
fi
echo "  OK"

# ---- [3/6] 前置清理:残留 driver / moveit / bt_runner ----
echo "[3/6] 检查残留进程 (有就自动按 PID 阶梯清,3s panic window)..."
if ! auto_cleanup_stale; then
  exit 1
fi
echo "  OK (无残留)"

# ---- [4/6] 启动 agx_arm_ctrl hardware driver ----
echo "[4/6] 启动 agx_arm_ctrl hardware driver..."
DRIVER_LOG="${WS_FORK_CT}/.run/lgw_lookat_isolated_test_driver_$(date +%Y%m%d_%H%M%S).log"
DRIVER_LOG_HOST="${WS_FORK_HOST}/.run/lgw_lookat_isolated_test_driver_$(date +%Y%m%d_%H%M%S).log"
docker exec "${CONTAINER}" mkdir -p "$(dirname "${DRIVER_LOG}")"
# 关键:docker exec -e 传 LGW_INSTALL,让容器内 source l4t_setup.sh 走 fork install
docker exec -e LGW_INSTALL="${LGW_INSTALL}" -d "${CONTAINER}" bash -lc "
  source /my_ws/l4t/l4t_setup.sh
  ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
    effector_type:=agx_gripper tcp_offset:='[0.0, 0.0, 0.12, 0.0, 0.0, 0.0]' \
    > '${DRIVER_LOG}' 2>&1
"

# 等 driver up (≤15s)
WAITED=0
while [ ${WAITED} -lt 15 ]; do
  if docker exec "${CONTAINER}" bash -lc "
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
echo "  driver_log = ${DRIVER_LOG_HOST}"
echo "  OK"

# ---- [5/6] 提取 TreeID + 启动 launch (后台) ----
echo "[5/6] 启动 launch (后台) 并准备 send_goal..."
TARGET_TREE=$(docker exec "${CONTAINER}" bash -lc "
  grep -oP '<BehaviorTree ID=\"\K[^\"]+' '${BT_XML}' | head -1
")
if [ -z "${TARGET_TREE}" ]; then
  echo "  ERROR: 无法从 ${BT_XML} 提取 <BehaviorTree ID>"
  exit 1
fi
echo "  BT_XML      = ${BT_XML}"
echo "  target_tree = ${TARGET_TREE}"

LAUNCH_LOG="${WS_FORK_CT}/.run/lgw_lookat_isolated_test_launch_$(date +%Y%m%d_%H%M%S).log"
LAUNCH_LOG_HOST="${WS_FORK_HOST}/.run/lgw_lookat_isolated_test_launch_$(date +%Y%m%d_%H%M%S).log"
docker exec "${CONTAINER}" mkdir -p "$(dirname "${LAUNCH_LOG}")"
echo "  launch_log  = ${LAUNCH_LOG_HOST}"

# detached launch —— 注意 -e LGW_INSTALL + source l4t_setup.sh (否则走 fastrtps + 无 CYCLONEDDS_URI)
docker exec -e LGW_INSTALL="${LGW_INSTALL}" -e BT_PLANNER_DEBUG=1 -d "${CONTAINER}" bash -lc "
  source /my_ws/l4t/l4t_setup.sh
  ros2 launch lgw_bt_planner bt_with_lgw_moveit.launch.py > '${LAUNCH_LOG}' 2>&1
"

# 等 /lgw_bt_execution action server up (move_group + bt_runner 加载 .so/.xml 都需要时间)
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

# ---- [6/6] 提示用户手动 send_goal ----
echo "[6/6] Launch 已就绪 —— 请手动 send_goal。Ctrl-C 终止本脚本 → 自动清理。"
echo ""
echo "  ✅ 硬件安全自检 (真机必读):"
echo "      1) 机械臂 + 夹爪 + CAN 总线已上电"
echo "      2) 周围 0.5 m 范围内无障碍物 / 无关人员"
echo "      3) E-stop 按钮随手可触"
echo "      4) BT 树会真实移动机械臂 (MoveArmToPose + MoveArmToNamed)"
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
