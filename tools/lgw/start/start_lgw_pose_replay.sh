#!/bin/bash

# shellcheck source=../_common.sh
source "$(dirname "${BASH_SOURCE[0]}")/../_common.sh"
# scripts/start_lgw_pose_replay.sh
#
# 一键启动 LG_ws standalone pose replay (canonical BT.ROS2 TreeExecutionServer)
#
# 这是 agx_arm_ws/scripts/start_standalone_pose_replay.sh 的 LG_ws 版本
# (用 lgw_bt_planner 取代 piper_bt_planner,MoveIt 配置直接复用
#  agx_arm_moveit 的 demo.launch.py + SRDF)。两个脚本结构对齐,
# 改的是 BT_XML / launch / action name / stray 进程正则。
#
# 流程 (本脚本 step 2 显式调 can_activate.sh;launch 文件不再内嵌 activate_can,
#       对齐 agx_arm_ros README 约定):
#   1. docker ps / docker start self_l4t
#   2. can_activate.sh can0 1000000       # CAN 总线 up (幂等)
#   3. agx_arm_ctrl/start_single_agx_arm.launch.py  # 硬件 driver (顶层 launch include)
#   4. agx_arm_moveit/launch/demo.launch.py        # rsp + move_group + ros2_control_node
#                                              (effector_type:=agx_gripper 让 xacro
#                                               <xacro:if> 块原样展开,不再 patch SRDF)
#   5. lgw_bt_planner/launch/lgw_bt_planner.launch.py  # bt_runner
#                                              起 action server @ /lgw_bt_execution
#
# 启动后本脚本:
#   a. 从 ${BT_XML} 的 <BehaviorTree ID="..."> 属性提取 TreeID
#   b. 等 /lgw_bt_execution action server 就绪 (≤30s)
#   c. foreground `ros2 action send_goal` 触发 BT,tree 跑完 / FAIL / Ctrl-C 才返回
#
# Launch 自身 stdout/stderr 重定向到
#   /my_ws/l4t/lg_walker_ws/.run/lgw_launch_<ts>.log
# (bind-mount 路径,host 直接可 tail)。launch 启动失败时第一时间看那个文件。
#
# 默认 BT_XML = pick_and_place_demo.xml (TreeID = PickAndPlaceDemo)
# bt_runner 通过 `behavior_trees` ROS param 加载
#   install/lgw_bt_planner/share/lgw_bt_planner/bt_trees/*.xml,
#   全部 3 个 BT 树 (pick_and_place_demo / go_to_target_and_back /
#   pose_via_set_pose_stamped) 都能作为 goal 触发。
#
# tcp_offset 默认 [0.0, 0.0, 0.12, 0.0, 0.0, 0.0] (沿 flange 工具 z 轴 +0.12 m)
#
# WITH_PERCEPTION: 本 launch 不接 perception 节点 (perception 由 T1/T2 终端单独跑),
#                  保留这个 env var 仅是为了和 agx_arm_ws 版本对齐;传入值会被忽略。
#
# 本脚本只管 LG stack (机械臂 + MoveIt + BT runner)。
# Go2 driver / 硬件 bringup 不在本脚本范围内 —— 真机部署前请按 runbook 在
# T1/T2 终端分别启动 (e.g. ros2 launch go2_ros2_sdk go2_ros2.launch.py 等)。
#
# 硬件前置 (真机):
#   - 机械臂 + 夹爪 + CAN 总线已通电
#   - 周围 0.5 m 范围内无障碍 (MoveArmToPose / MoveArmToNamed 真实移动机械臂)
#   - E-stop 按钮随手可触,无人在碰撞危险区
#   - 首次运行前确认 MoveArmToPose IK seed 已接 /feedback/joint_states
#     (agx_arm IK stability 三件套: ompl jiggle=0, kinematics_solver_timeout=0.05+,
#      用 /feedback/joint_states 作 seed)
#
# 用法 (host 上跑):
#   ./scripts/start_lgw_pose_replay.sh
#   # 或覆盖默认 BT_XML: BT_XML=/path/to/other_tree.xml ./scripts/start_lgw_pose_replay.sh
# 退出: Ctrl-C (前台 sleep 等用户,trap 自动按 PID 阶梯清理 launch 进程)
# send_goal 不再由本脚本触发 —— 用户在另一终端重复 send_goal / 换 TreeID
# 手动清理(自动 trap 失败时的 backup):
#   docker exec self_l4t bash -c '
#     ps -ef | grep -E "lgw_bt_planner|bt_with_lgw|moveit_lgw|bt_runner|start_single_agx_arm|agx_arm_ctrl_single|demo.launch.py" | grep -v grep
#   '
#   列出 PID 后逐个 kill -TERM,3s 后还活的就 kill -KILL (禁用 pkill)

set -u
CONTAINER="self_l4t"

# LG stack 全部相关进程的统一 grep pattern (cleanup_on_signal / STRAY 检查 / 自动清理共用)。
# 列启动 wrapper (bt_with_lgw_moveit / demo / start_single_agx_arm) + 子节点
# (move_group, bt_runner, agx_arm_ctrl_single, ros2_control_node, controller_manager,
# robot_state_publisher, spawner_*)。故意只列精确名字,不写路径前缀
# (lgw_bt_planner / moveit_lgw),避免路径字串误命中其他无关进程。
#
# 严格按 PID 阶梯清理 (禁用 pkill / killall,CLAUDE.md 反模式)。
readonly LG_STACK_PIDS_PATTERN='bt_with_lgw_moveit\.launch\.py|demo\.launch\.py|start_single_agx_arm|agx_arm_ctrl_single|bt_runner|move_group|robot_state_publisher|ros2_control_node|controller_manager|spawner_joint_state_broadcaster|spawner_arm_controller|spawner_gripper_controller'

# ---- Ctrl-C 自动清理 ----
# 收到 SIGINT/SIGTERM 时,按 PID 阶梯清理容器内 LG launch 树
# (per memory `feedback_no_blankslate_kill` + `feedback_yoloe_orphan`):
# TERM → 3s → KILL,一次一个 PID,禁用 pkill。
cleanup_on_signal() {
  if [ "${CLEANUP_DONE:-0}" = "1" ]; then
    return
  fi
  CLEANUP_DONE=1

  echo ""
  echo "[cleanup] Ctrl-C / 信号触发,清理 launch 进程..."

  # 容器内 grep LG launch 树:复用 LG_STACK_PIDS_PATTERN (与 STRAY 检查 /
  # auto_cleanup_stale 共用),严格按进程名 / 路径字串匹配,禁用 pkill /
  # killall 等宽匹配 (CLAUDE.md 反模式)。
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
# 流程: 列 PID → 3s panic window (用户 Ctrl-C 可中止,bash trap 会转 cleanup_on_signal
# 重复 TERM 后 exit 130) → TERM → 3s grace → KILL → 二次 grep 验证。
#
# 返回: 0 全部清干净 / 1 还有残留 (调用方自行 exit 1)。
#
# 真机安全约束:
#   - 严格按 PID 阶梯 (禁用 pkill / killall,kill -9 -1)
#   - 3s panic window 让用户能中止
#   - TERM 让节点有 graceful shutdown 机会 (move_group cancel goal /
#     ros2_control disable controllers),3s 后 KILL 兜底
auto_cleanup_stale() {
  local pids
  pids=$(docker exec "${CONTAINER}" bash -lc "
    ps -ef | grep -E '${LG_STACK_PIDS_PATTERN}' | grep -v grep | awk '{print \$2}'
  " 2>/dev/null) || pids=""

  if [ -z "${pids}" ]; then
    return 0  # 干净,无需清理
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

  # 二次验证:还有活的就报失败
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

# 覆盖默认 BT_XML / with_perception (可选,通常不传):
# BT_XML 指向 bt_trees/ 下的某个 XML 文件,TreeID 从 <BehaviorTree ID=...> 提取
# 默认 BT_XML = pick_and_place_demo.xml (TreeID = PickAndPlaceDemo)
BT_XML="${BT_XML:-${WS_FORK_CT}/install/lgw_bt_planner/share/lgw_bt_planner/bt_trees/pick_and_place_demo.xml}"
WITH_PERCEPTION="${WITH_PERCEPTION:-false}"   # LG launch 不接 perception,值会被忽略

# ---- 前置检查: 容器必须在跑 ----
echo "[1/6] 检查容器 ${CONTAINER} ..."
if ! docker ps --format '{{.Names}}' 2>/dev/null | grep -q "^${CONTAINER}$"; then
  echo "  容器未运行,尝试 docker start ${CONTAINER} ..."
  if ! docker start "${CONTAINER}" >/dev/null 2>&1; then
    echo "  ERROR: docker start ${CONTAINER} 失败"
    exit 1
  fi
  sleep 2
  # 再确认一次
  if ! docker ps --format '{{.Names}}' 2>/dev/null | grep -q "^${CONTAINER}$"; then
    echo "  ERROR: ${CONTAINER} 启动后仍不在运行列表"
    exit 1
  fi
fi
echo "  OK"

# ---- 激活 CAN 总线 ----
# 对齐 agx_arm_ros README 约定 —— start_single_agx_arm.launch.py 假设 can0 已 up,
# 不自带 can_activate.sh。一键脚本里显式 step 2 跑一次 (幂等,UP 后无副作用)。
# sudo 不通时脚本会失败;真机部署前确保容器 user 有 passwordless sudo 或 CAN 已先 up。
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
# L2 wrapper (本脚本) 启 start_single_agx_arm + MoveIt + bt_runner,重复运行会
# 导致 CAN 抢占 / TF 冲突。auto_cleanup_stale 自动按 PID 阶梯清理 + 3s grace,
# 真机场景下用户在 panic window 内 Ctrl-C 可中止。
echo "[3/6] 检查残留进程 (有就自动按 PID 阶梯清,3s panic window)..."
if ! auto_cleanup_stale; then
  exit 1
fi
echo "  OK (无残留)"

# ---- 启动 agx_arm_ctrl hardware driver (L2 session-level) ----
# 与 can_activate.sh 处理方式一致 (commit 31195f9):driver 是会话级硬件,
# 不由 L3 launch 内嵌 —— 对齐 agx_arm_ros README 约定 (start_single_agx_arm
# 是顶层 launch,用户显式启动)。重启 launch = 重启 driver,会让真机
# joint_states 中断 → IK 失败 → move_group 状态错乱,真机不安全。
#
# tcp_offset 与 MoveArmToPose goal 内 tcp_offset 一致 (flange 工具 z 轴 +0.12 m,
# launch 文件默认)。
echo "[4/6] 启动 agx_arm_ctrl hardware driver..."
DRIVER_LOG="${WS_FORK_CT}/.run/lgw_driver_$(date +%Y%m%d_%H%M%S).log"
docker exec "${CONTAINER}" mkdir -p "$(dirname "${DRIVER_LOG}")"
docker exec -d "${CONTAINER}" bash -lc "
  source /my_ws/l4t/l4t_setup.sh
  ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
    effector_type:=agx_gripper tcp_offset:='[0.0, 0.0, 0.12, 0.0, 0.0, 0.0]' \
    > '${DRIVER_LOG}' 2>&1
"

# 等 driver up:start_single_agx_arm 加载 SDK + 读 fw_version + enable 控制器
# 通常 5-10s。15s 内 /agx_arm_ctrl_single_node 没起来就 WARN 继续。
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
echo "  with_perception= ${WITH_PERCEPTION} (LG launch 不接 perception,值会被忽略)"

# 注: 与 agx_arm_ws 版本不同,这里不需要 EXTRA_ARGS:with_perception:= 也不会传
# LG launch (bt_with_lgw_moveit.launch.py) 本身就不读 with_perception 这个 arg。

# Launch stderr/stdout → 时间戳文件 (host 直接可 tail,不用再 docker exec)
# 注意 docker exec -d 把 host stream 断了,redirect 必须在 container 内做;
# "${LAUNCH_LOG}" 在 host 先展开,再传进 container bash -lc
# mkdir 也必须在 container 内做 —— host 上没有 /my_ws 这个路径 (那是 container 内的 bind-mount 视图)
LAUNCH_LOG="${WS_FORK_CT}/.run/lgw_launch_$(date +%Y%m%d_%H%M%S).log"
docker exec "${CONTAINER}" mkdir -p "$(dirname "${LAUNCH_LOG}")"
echo "  launch_log     = ${LAUNCH_LOG}"

# detached launch —— 注意 source l4t_setup.sh (否则走 fastrtps + 无 CYCLONEDDS_URI)
docker exec -d "${CONTAINER}" bash -lc "
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
  echo "  WARN: 30s 内 /lgw_bt_execution 未起来,继续 send_goal (会 FAIL)"
fi

# ---- 提示用户手动 send_goal (launch 已就绪,前台 sleep 等 Ctrl-C) ----
# 改设计:本脚本不再自动 send_goal —— 自动 send_goal 一次就退出会让 launch
# 进程孤儿残留,用户没法继续测下一个 TreeID。改成启动完 prompt 提示,然后
# sleep infinity 等 Ctrl-C → trap 自动清理 launch 树。
# 用户可在另一终端(或本终端新开窗口)重复 send_goal / 换 TreeID。
echo "[6/6] Launch 已就绪 —— 请手动 send_goal。Ctrl-C 终止本脚本 → 自动清理。"
echo ""
echo "  ✅ 硬件安全自检 (真机必读):"
echo "      1) 机械臂 + 夹爪 + CAN 总线已上电"
echo "      2) 周围 0.5 m 范围内无障碍物 / 无关人员"
echo "      3) E-stop 按钮随手可触"
echo "      4) BT 树会真实移动机械臂 (MoveArmToPose / MoveArmToNamed)"
echo "      5) Go2 driver + 硬件 bringup 已在 T1/T2 终端启动"
echo ""
echo "  在另一终端(或本终端新开窗口)执行:"
echo ""
echo "    docker exec -it ${CONTAINER} bash -lc '"
echo "      source /my_ws/l4t/l4t_setup.sh"
echo "      ros2 action send_goal /lgw_bt_execution \\"
echo "        btcpp_ros2_interfaces/action/ExecuteTree \\"
echo "        \"{target_tree: ${TARGET_TREE}}\" --feedback"
echo "    '"
echo ""
echo "  当前默认 BT_XML: ${BT_XML}"
echo "  (脚本开头可用 BT_XML=... 覆盖;或直接在 send_goal 里改 target_tree)"
echo ""
echo "  install 下所有可用 BT xml + TreeID:"
docker exec "${CONTAINER}" bash -lc '
  for f in ${WS_FORK_CT}/install/lgw_bt_planner/share/lgw_bt_planner/bt_trees/*.xml; do
    tid=$(grep -oP "<BehaviorTree ID=\"\K[^\"]+" "$f" | head -1)
    printf "    %-32s TreeID=%s\n" "$(basename $f)" "$tid"
  done
'
echo ""
echo "  多次 send_goal 复用同一组 launch 进程,可换 TreeID。"
echo "  Ctrl-C 终止本脚本 → trap 自动按 PID 清理 launch 树。"
echo ""

# 前台等 Ctrl-C:sleep 期间若收到 SIGINT/SIGTERM,bash 转给 trap,
# trap 调用 cleanup_on_signal → 按 PID 阶梯 TERM→3s→KILL → exit 130
echo "=== 等 Ctrl-C ==="
sleep infinity
