#!/bin/bash
# scripts/start_lgw_head_to_arm_verify_grasp.sh
#
# 一键启动 HeadToArmVerifyGrasp BT(ReactiveFallback + Repeat num_cycles="-1"
# 视觉伺服 + 一致性核实 + 抓取回路)。完整 bringup 链(arm 先起 → YOLOE 后起,与
# test_yoloe_stability.sh Phase 1/1.5/1.6/2/2.5 对齐简化):
#   Phase 3 (容器)    CAN → agx_arm_ctrl → MoveIt → bt_runner(L2 wrapper,
#                     与 start_lgw_user_gated_auto_loop.sh 对齐)
#   Phase Y1 (host)   YOLOE Foxy inference loop
#   Phase Y1.5        等 YOLOE emit CONTINUOUS_DUAL_CAMERA_STARTED
#                     (dual_camera_node.__init__ 末尾 emit,模型加载完成 +
#                     subscriber 已注册,不依赖 cameras 已起)
#   Phase Y1.6        USB kernel unbind+bind 强制重枚举两路相机(Path A:
#                     解决上一次 realsense SIGSEGV 后 libusb handle 残留
#                     → 下次 RS2_USB_STATUS_BUSY,test_yoloe_stability.sh
#                     Phase 1.6 的同款 fix)
#   Phase Y2 (容器)   两路 realsense 相机 (two_realsense.launch.py)
#   Phase Y2.5        等 head + arm 首个 CONTINUOUS_RESULT(pipeline-alive 即可,
#                     不验 detection:目标物在不在 FOV 是 BT 运行时条件)
#   Phase Y3.5        校验 /perception/two_stage/camera_{head,arm}/result 在容
#                     器内有 publisher(与 test Phase 3.5 同,只验 pub_count≥1)
# head 看到目标 → ComputeLookatPose 算末端 lookat → MoveArmToPose 移动 →
# arm 相机确认 → VerifyHeadArmConsistency(head/arm world 系位置误差
# ≤ max_distance_m,默认 0.05 m)→ ComputeGraspPose(tcp 直接到 arm 观测
# 中心,姿态 = lookat 姿态)→ MoveArmToPose 到 grasp → gripper_open/
# close → 回 home。无限循环,直到 user msg halt。
#
# 这是 scripts/start_lgw_user_gated_auto_loop.sh 的 fork 专用变种
# (只换 BT_XML / log 路径 / TreeID / 加 YOLOE Y phases,其他保持对齐)。
#
# 与 start_lgw_user_gated_auto_loop.sh 的关键差异:
#   - 加 YOLOE workflow 集成(Y1/Y2/Y2.5/Y3.5,参考 test_yoloe_poll_loop.sh)
#   - WS_FORK(本 ws)硬编码,而不是默认 lg_walker_ws(主 ws)
#   - BT_XML 默认 head_to_arm_verify_grasp.xml (TreeID=HeadToArmVerifyGrasp)
#   - .run 日志目录指向 ${WS_FORK}/.run/
#   - LGW_INSTALL 在 source l4t_setup.sh 之前 export 到 fork,避免被默认
#     覆盖回主 ws(本脚本不用求用户记得 export)
#
# Env override(可选):
#   YOLOE_AUTOSTART=0           跳过 Y1/Y2/Y2.5(只跑 Y3.5 校验),假设用户
#                              已经在 host/容器外部启好 YOLOE + cameras
#   HEAD_CAMERA_SERIAL=...      默认 _261622078621
#   ARM_CAMERA_SERIAL=...       默认 _262422071246
#   YOLOE_DEPLOY=$HOME/...      默认 $HOME/yoloe_deploy
#   HUMBLE_YOLOE_INSTALL=...    默认 /opt/yoloe_transfer/humble_ws/install/local_setup.bash
#   HEAD_USB_PORT=2-1           head D435i USB 拓扑(Phase Y1.6 kernel
#                              unbind+bind 用;host 跑 `lsusb -t` 可重核)
#   ARM_USB_PORT=1-3.3          arm D435 USB 拓扑
#
# 启动后:
#   a. 从 ${BT_XML} 的 <BehaviorTree ID="..."> 属性提取 TreeID
#   b. 等 /lgw_bt_execution action server 就绪 (≤30s)
#   c. 前台 sleep infinity 等用户 Ctrl-C → trap 自动清理 launch 树
#      (用户可在另一终端多次 send_goal / 反复发 user msg halt+re-trigger)
#
# 与 reference start_lgw_pose_replay.sh 的差异:
#   - 多 Phase Y1/Y1.5/Y1.6/Y2/Y2.5/Y3.5(YOLOE + cameras 集成)
#   - WS_FORK(本 ws)硬编码;LGW_INSTALL 在 source l4t_setup.sh 前 export
#   - LG stack + YOLOE 容器内进程的 PID 联合清理 pattern
#     (LG_STACK_PIDS_PATTERN,见 §preflight / §cleanup_on_signal)
#   - 终止提示带 YOLOE 特有的 halt 指令:发 /bt_user_input data:true
#     让 BT ReactiveFallback root 立即整树 SUCCESS → launch 自然 idle

set -u
CONTAINER="self_l4t"

# shellcheck source=../_common.sh
source "$(dirname "${BASH_SOURCE[0]}")/../_common.sh"

# 与 start_lgw_user_gated_auto_loop.sh 完全一致的容器内 PID pattern match
# 基础(LG stack + MoveIt + 硬件 driver + bt_runner),再追加 YOLOE 引入的
# realsense2_camera_node / two_realsense.launch.py / yoloe_humble_orchestration
# 让一次 grep 就能覆盖所有需要清理的容器内进程(CLAUDE.md 反模式:pkill /
# killall / kill -1 全禁用)。
readonly LG_STACK_PIDS_PATTERN='bt_with_lgw_moveit\.launch\.py|lgw_bt_planner\.launch\.py|demo\.launch\.py|start_single_agx_arm|agx_arm_ctrl_single|bt_runner|move_group|robot_state_publisher|ros2_control_node|controller_manager|spawner_joint_state_broadcaster|spawner_arm_controller|spawner_gripper_controller|l4t_setup\.sh|can_activate\.sh|two_realsense\.launch\.py|realsense2_camera_node|yoloe_humble_orchestration'

# BT_XML 默认指向新树(TreeID=HeadToArmVerifyGrasp)
readonly DEFAULT_BT_XML="${WS_FORK_CT}/install/lgw_bt_planner/share/lgw_bt_planner/bt_trees/head_to_arm_verify_grasp.xml"
BT_XML="${BT_XML:-${DEFAULT_BT_XML}}"
readonly TREE_ID="HeadToArmVerifyGrasp"

# ---- YOLOE 配置(参考 test_yoloe_poll_loop.sh Phase 1+2) ----
YOLOE_AUTOSTART="${YOLOE_AUTOSTART:-1}"
HEAD_CAMERA_SERIAL="${HEAD_CAMERA_SERIAL:-_261622078621}"
ARM_CAMERA_SERIAL="${ARM_CAMERA_SERIAL:-_262422071246}"
YOLOE_DEPLOY="${YOLOE_DEPLOY:-$HOME/yoloe_deploy}"
HUMBLE_YOLOE_INSTALL="${HUMBLE_YOLOE_INSTALL:-/opt/yoloe_transfer/humble_ws/install/local_setup.bash}"
ROS_DOMAIN_ID_YOLOE="${ROS_DOMAIN_ID_YOLOE:-0}"
RMW_IMPLEMENTATION_YOLOE="${RMW_IMPLEMENTATION_YOLOE:-rmw_cyclonedds_cpp}"
# USB 端口路径(Phase Y1.6 kernel unbind+bind 用);默认按已确认的 topology,
# 可通过 env 覆盖。host 跑 `lsusb -t` 可查;容器内 /sys/bus/usb 已 bind mount。
HEAD_USB_PORT="${HEAD_USB_PORT:-2-1}"     # head D435i on USB 3.0 root hub
ARM_USB_PORT="${ARM_USB_PORT:-1-3.3}"     # arm  D435  on USB 2.0 hub
# YOLOE host bash 需要 \$YOLOE_DEPLOY 在 nohup subshell 里 expand,必须
# export(参考 test_yoloe_poll_loop.sh:67)。否则 subshell 视 "$YOLOE_DEPLOY"
# 为空,launch 收到 transfer_root:="" → "malformed launch argument"。
export YOLOE_DEPLOY

# Host-side PIDs(YOLOE Foxy 在 host 跑,要 host 端 kill,不走 docker exec)
HOST_PIDS=()

# ---- Ctrl-C 自动清理 ----
# 容器内进程(LG stack + cameras)走 docker exec kill;host 进程(YOLOE Foxy)
# 走 host kill。一次一个 PID,禁 pkill / killall (CLAUDE.md 反模式)。
# Orphan realsense2_camera_node 处理沿用 test_yoloe_poll_loop.sh:cleanup
# 后再 grep 一次,防止 reparent init 残留。
cleanup_on_signal() {
  if [ "${CLEANUP_DONE:-0}" = "1" ]; then return; fi
  CLEANUP_DONE=1
  echo ""
  echo "[cleanup] Ctrl-C / 信号触发,清理 launch 进程(容器内 PID + host PID)..."

  # ---- 容器内 PID 清理 ----
  local pids
  pids=$(docker exec "${CONTAINER}" bash -c "
    ps -eo pid,args | \
    grep -E '${LG_STACK_PIDS_PATTERN}' | \
    grep -v grep | \
    awk '{print \$1}'
  " 2>/dev/null) || pids=""

  if [ -n "${pids}" ]; then
    echo "[cleanup] kill candidates (container pids): ${pids}"
    # 第一轮:TERM
    for pid in ${pids}; do
      echo "[cleanup] TERM container pid=${pid}"
      docker exec "${CONTAINER}" kill -TERM "${pid}" 2>/dev/null || true
    done
    sleep 3
    # 第二轮:KILL 残留
    for pid in ${pids}; do
      if docker exec "${CONTAINER}" kill -0 "${pid}" 2>/dev/null; then
        echo "[cleanup] KILL container pid=${pid}"
        docker exec "${CONTAINER}" kill -KILL "${pid}" 2>/dev/null || true
      fi
    done
  fi

  # ---- Orphan realsense2_camera_node(reparent init)二次清理 ----
  # Pattern 来自 test_yoloe_poll_loop.sh:test_yoloe_stability.sh 也踩过
  # 这坑(camera_node 父进程死掉后 reparent 到 init,grep 一次抓不到所有)。
  local orphans
  orphans=$(docker exec "${CONTAINER}" bash -c "
    ps -eo pid,args 2>/dev/null \
      | grep 'realsense2_camera_node' \
      | grep -v grep | awk '{print \$1}'
  " 2>/dev/null) || orphans=""
  if [ -n "${orphans}" ]; then
    echo "[cleanup] orphan realsense2_camera_node pids: ${orphans}"
    for pid in ${orphans}; do
      docker exec "${CONTAINER}" kill -TERM "${pid}" 2>/dev/null || true
    done
    sleep 2
    for pid in ${orphans}; do
      if docker exec "${CONTAINER}" kill -0 "${pid}" 2>/dev/null; then
        docker exec "${CONTAINER}" kill -KILL "${pid}" 2>/dev/null || true
      fi
    done
  fi

  # ---- Host-side PID 清理(YOLOE Foxy) ----
  if [ ${#HOST_PIDS[@]} -gt 0 ]; then
    echo "[cleanup] host pids: ${HOST_PIDS[*]}"
    for pid in "${HOST_PIDS[@]}"; do
      if [ -n "${pid}" ] && kill -0 "${pid}" 2>/dev/null; then
        echo "[cleanup] TERM host pid=${pid}"
        kill -TERM "${pid}" 2>/dev/null || true
      fi
    done
    sleep 2
    for pid in "${HOST_PIDS[@]}"; do
      if [ -n "${pid}" ] && kill -0 "${pid}" 2>/dev/null; then
        echo "[cleanup] KILL host pid=${pid}"
        kill -KILL "${pid}" 2>/dev/null || true
      fi
    done
  fi

  # ---- Host orphan dual_camera_node(reparent init)二次扫描 ----
  # yoloe_deploy wrapper 不会 forward SIGTERM 给 dual_camera_node 子进程,
  # wrapper 死后子进程 reparent init(PPID=1)→ HOST_PIDS 抓不到,必须独立扫。
  # 沿用 test_yoloe_poll_loop.sh:cleanup 末尾 orphan sweep 的思路。
  local yoloe_orphans
  yoloe_orphans=$(ps -eo pid,ppid,args 2>/dev/null \
    | awk '$2 == 1' \
    | grep -E 'dual_camera_node|yoloe_perception' \
    | grep -v grep \
    | awk '{print $1}') || yoloe_orphans=""
  if [ -n "${yoloe_orphans}" ]; then
    echo "[cleanup] host orphan YOLOE (PPID=1): ${yoloe_orphans}"
    for pid in ${yoloe_orphans}; do
      kill -TERM "${pid}" 2>/dev/null || true
    done
    sleep 2
    for pid in ${yoloe_orphans}; do
      if kill -0 "${pid}" 2>/dev/null; then
        kill -KILL "${pid}" 2>/dev/null || true
      fi
    done
  fi

  echo "[cleanup] done"
}

# ---- 启动前清理(preflight)----
# 防止上次 Ctrl-C / SIGHUP 没杀干净的残留进程(尤其是 yoloe wrapper 不
# forward SIGTERM 给 dual_camera_node child 造成的 orphan,以及 realsense
# USB 控制器被独占占用导致本次启动 color stream 立刻 stale —— 2026-10-02
# 真机测试复现)。仅清理本次 launch 范围(ros2 launch yoloe_perception /
# dual_camera_node / realsense2_camera_node / yoloe_humble_orchestration),
# 不动 LG stack(MoveIt / bt_runner / agx_arm_ctrl),后者由 cleanup_on_signal
# 走 LG_STACK_PIDS_PATTERN 覆盖。
preflight_cleanup() {
  echo "[preflight] 清理 host 上 YOLOE + container 内 cameras 残留进程..."
  local pids

  # ---- host:ros2 launch yoloe_perception wrappers + dual_camera_node children ----
  pids=$(ps -eo pid,ppid,args 2>/dev/null \
    | grep -E 'ros2 launch yoloe_perception|dual_camera_node' \
    | grep -v grep \
    | awk '{print $1}') || pids=""
  if [ -n "${pids}" ]; then
    echo "[preflight] host YOLOE candidates: ${pids}"
    for pid in ${pids}; do
      if kill -0 "${pid}" 2>/dev/null; then
        kill -TERM "${pid}" 2>/dev/null || true
      fi
    done
    sleep 3
    for pid in ${pids}; do
      if kill -0 "${pid}" 2>/dev/null; then
        echo "[preflight]   KILL host pid=${pid}"
        kill -KILL "${pid}" 2>/dev/null || true
      fi
    done
  fi

  # ---- container:realsense2_camera_node + yoloe_humble_orchestration ----
  # (可能 LG stack 还没起,容器未必 up,要容忍 docker exec 失败)
  pids=$(docker exec "${CONTAINER}" bash -c "
    ps -eo pid,args 2>/dev/null \
      | grep -E 'realsense2_camera_node|yoloe_humble_orchestration|two_realsense\.launch\.py' \
      | grep -v grep \
      | awk '{print \$1}'
  " 2>/dev/null) || pids=""
  if [ -n "${pids}" ]; then
    echo "[preflight] container camera candidates: ${pids}"
    for pid in ${pids}; do
      docker exec "${CONTAINER}" kill -TERM "${pid}" 2>/dev/null || true
    done
    sleep 3
    for pid in ${pids}; do
      if docker exec "${CONTAINER}" kill -0 "${pid}" 2>/dev/null; then
        echo "[preflight]   KILL container pid=${pid}"
        docker exec "${CONTAINER}" kill -KILL "${pid}" 2>/dev/null || true
      fi
    done
  fi

  # ---- container orphan realsense2_camera_node(reparent init)二次扫描 ----
  # test_yoloe_stability.sh / test_yoloe_poll_loop.sh 都踩过:wrapper 死后
  # child reparent init,grep 一次抓不到,必须独立扫一次。
  pids=$(docker exec "${CONTAINER}" bash -c "
    ps -eo pid,ppid,args 2>/dev/null \
      | awk '\$2 == 1' \
      | grep 'realsense2_camera_node' \
      | grep -v grep \
      | awk '{print \$1}'
  " 2>/dev/null) || pids=""
  if [ -n "${pids}" ]; then
    echo "[preflight] orphan realsense2_camera_node (PPID=1): ${pids}"
    for pid in ${pids}; do
      docker exec "${CONTAINER}" kill -TERM "${pid}" 2>/dev/null || true
    done
    sleep 2
    for pid in ${pids}; do
      if docker exec "${CONTAINER}" kill -0 "${pid}" 2>/dev/null; then
        docker exec "${CONTAINER}" kill -KILL "${pid}" 2>/dev/null || true
      fi
    done
  fi

  # ---- USB settle(让 realsense 控制器完整释放带宽,再起新实例) ----
  sleep 2
  echo "[preflight] done"
}

trap cleanup_on_signal INT TERM
# trap 设好立刻跑 preflight(不依赖 docker container 是否 up)
preflight_cleanup

# ---- 启动前检查 ----
docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$" \
  || docker start "${CONTAINER}" >/dev/null

# CAN + hardware driver + MoveIt + bt_runner L2 wrapper
# (与 start_lgw_user_gated_auto_loop.sh L2 顺序一致;真机前置:can0 up、
# 机械臂 + 夹爪上电、Go2 driver 已起、YOLOE detection workflow 已起、
# 双 realsense 相机 stream 已起)
echo "[bringup] L2 wrapper (CAN → agx_arm_ctrl → MoveIt → bt_runner) ..."
# 关键:export LGW_INSTALL 指向本 fork,保证容器内 source l4t_setup.sh 后,
# ros2 pkg list / launch 路径都解析到 fork install 而不是主 ws。
# 这一行也覆盖用户提前 export 的 LGW_INSTALL(本脚本就是要固化到 fork)
export LGW_INSTALL="${WS_FORK_CT}/install/setup.bash"
echo "[bringup] LGW_INSTALL=${LGW_INSTALL}"

# .run/ 在 host 端通常是 root:root 755(unitree 写不进,容器以 root 跑可以写)。
# 所以 redirect 必须放在容器内的 bash -lc block 里,让容器写,而不是
# host shell redirect(会失败)。Canonical start_lgw_user_gated_auto_loop.sh
# 也是这个 pattern(DRIVER_LOG / LAUNCH_LOG 在容器内 redirect)。
#
# YOLOE/cameras log 例外:YOLOE Foxy 在 host 上跑(不经过 docker exec),
# 必须 host 端 bash redirect 到 .run/。需要先把 .run/ 改成 world-writable,
# 否则 host unitree 写不进(报错 "权限不够"——2026-10-02 真机调试踩过)。
# 改法:容器内 chmod 777 一次(容器是 root,可以 chmod host bind-mount 的 dir)。
LOG_TS="$(date +%Y%m%d_%H%M%S)"
docker exec "${CONTAINER}" mkdir -p "${WS_FORK_CT}/.run"
# 兜底:让 .run/ 对 host unitree 也可写(YOLOE/cameras log 是 host bash
# 重定向,默认 root:root 755 会报 "权限不够")。容器是 root,可以直接 chmod
# host bind-mount 进来的 dir。失败也无所谓(说明 dir 已经是 world-writable)。
docker exec "${CONTAINER}" chmod 777 "${WS_FORK_CT}/.run" 2>/dev/null || true

# ====================================================================
# Phase 3: CAN + driver + combined launch (对齐 start_lgw_pose_replay.sh)
# ====================================================================
# 拆成三步 detached docker exec(对齐 reference start_lgw_pose_replay.sh):
#   Step 1: CAN 总线 up (幂等)
#   Step 2: agx_arm_ctrl hardware driver detached(agx_arm_ros README 约定
#           start_single_agx_arm.launch.py 是顶层 launch,不由 L3 内嵌)
#   Step 3: bt_with_lgw_moveit.launch.py combined launch detached
#           (内含 demo.launch.py effector_type:=agx_gripper + use_rviz:=false
#            + follow:=true + lgw_bt_planner.launch.py,全部 hardcode 在
#            bt_with_lgw_moveit.launch.py:38-50)
#
# 不再手动传 effector_type / use_rviz / follow 给 demo.launch —— combined
# launch 已经 hardcode;gripper_open/close named_target 才能被 SRDF 展开。

# Step 1: CAN 总线 up
echo "[bringup] Step 1: 激活 CAN 总线 can0 @ 1 Mbps..."
if ! docker exec "${CONTAINER}" bash \
    ${AGX_CAN_ACTIVATE_CT} can0 1000000 2>&1 | tail -3; then
  echo "[bringup]   ERROR: can_activate.sh 失败 (机械臂未上电 / 容器无 NOPASSWD sudo)"
  exit 1
fi

# Step 2: agx_arm_ctrl hardware driver detached
echo "[bringup] Step 2: 启动 agx_arm_ctrl hardware driver (detached)..."
DRIVER_LOG="${WS_FORK_CT}/.run/lgw_head_to_arm_verify_grasp_driver_${LOG_TS}.log"
DRIVER_LOG_HOST="${WS_FORK_HOST}/.run/lgw_head_to_arm_verify_grasp_driver_${LOG_TS}.log"
docker exec -e LGW_INSTALL="${LGW_INSTALL}" -d "${CONTAINER}" bash -lc "
  source /my_ws/l4t/l4t_setup.sh
  ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
    effector_type:=agx_gripper tcp_offset:='[0.0, 0.0, 0.12, 0.0, 0.0, 0.0]' \
    > '${DRIVER_LOG}' 2>&1
"
echo "[bringup]   driver log → ${DRIVER_LOG_HOST}"

# 等 driver up:轮询 ≤15s 内 /agx_arm_ctrl_single_node 注册
WAITED=0
while [ ${WAITED} -lt 15 ]; do
  if docker exec "${CONTAINER}" bash -lc "
    source /my_ws/l4t/l4t_setup.sh
    ros2 node list 2>/dev/null | grep -q '/agx_arm_ctrl_single_node\$'
  " >/dev/null 2>&1; then
    echo "[bringup]   agx_arm_ctrl_single_node 就绪 (等了 ${WAITED}s)"
    break
  fi
  sleep 1
  WAITED=$((WAITED + 1))
done
if [ ${WAITED} -eq 15 ]; then
  echo "[bringup]   WARN: 15s 内 driver 未就绪,继续 (combined launch 会失败)"
fi

# Step 3: combined launch detached (demo + bt_planner,参数全 hardcode)
# BT runner 走固定单一文件 .run/lgw_head_to_arm_verify_grasp_bt.log(覆盖式,
# 每次启动 fresh)。launch 文件里 bt_runner node 配 output="screen",stdout
# 从 launch process 继承 → 下面 redirect 能接住。否则 ROS 2 默认会写到
# /root/.ros/log/bt_runner_*.log,start 脚本接不到 BT 日志。
echo "[bringup] Step 3: 启动 bt_with_lgw_moveit.launch.py (detached)..."
LGW_LAUNCH_LOG="${WS_FORK_CT}/.run/lgw_head_to_arm_verify_grasp_bt.log"
LGW_LAUNCH_LOG_HOST="${WS_FORK_HOST}/.run/lgw_head_to_arm_verify_grasp_bt.log"
# truncate 旧文件,避免文件无限增长(每次 BT 调试只看最近一次)
: > "${LGW_LAUNCH_LOG_HOST}" || true
docker exec -e LGW_INSTALL="${LGW_INSTALL}" -e BT_PLANNER_DEBUG=1 -d "${CONTAINER}" bash -lc "
  source /my_ws/l4t/l4t_setup.sh
  ros2 launch lgw_bt_planner bt_with_lgw_moveit.launch.py \
    >> '${LGW_LAUNCH_LOG}' 2>&1
"
echo "[bringup]   launch log → ${LGW_LAUNCH_LOG_HOST}"

# 等 /lgw_bt_execution action server up(move_group + bt_runner 加载 .so/.xml 都需要时间)
WAITED=0
while [ ${WAITED} -lt 30 ]; do
  if docker exec "${CONTAINER}" bash -lc "
    source /my_ws/l4t/l4t_setup.sh
    ros2 action list 2>/dev/null | grep -q '^/lgw_bt_execution\$'
  " >/dev/null 2>&1; then
    echo "[bringup]   /lgw_bt_execution action server 就绪 (等了 ${WAITED}s)"
    break
  fi
  sleep 1
  WAITED=$((WAITED + 1))
done
if [ ${WAITED} -eq 30 ]; then
  echo "[bringup]   WARN: 30s 内 action server 未起来,后续 send_goal 会 FAIL"
fi

# ====================================================================
# Phase Y1: Start YOLOE on host Foxy (test_yoloe_poll_loop.sh Phase 1)
# ====================================================================
if [ "${YOLOE_AUTOSTART}" = "1" ]; then
  echo "[yoloe] Phase Y1: start YOLOE Foxy on host (transfer_root=${YOLOE_DEPLOY})..."
  YOLOE_LOG_HOST="${WS_FORK_HOST}/.run/lgw_head_to_arm_verify_grasp_yoloe_${LOG_TS}.log"
  # YOLOE 在 host Foxy 跑,不走容器,所以重定向直接在 host 上做。
  nohup bash -c "
    set +u
    set -a
    source ${YOLOE_DEPLOY}/config/foxy_humble_dds.env
    set +a
    source /opt/ros/foxy/setup.bash
    source ${YOLOE_DEPLOY}/ros2_ws/install/local_setup.bash
    export LD_PRELOAD=/lib/aarch64-linux-gnu/libgomp.so.1
    export PYTHONUNBUFFERED=1
    ros2 launch yoloe_perception dual_camera_foxy.launch.py \
        transfer_root:=\"\${YOLOE_DEPLOY}\"
  " > "${YOLOE_LOG_HOST}" 2>&1 &
  YOLOE_PID=$!
  HOST_PIDS+=("${YOLOE_PID}")
  echo "[yoloe]   YOLOE host PID=${YOLOE_PID}, log → ${YOLOE_LOG_HOST}"

  # 等 yoloe_dual_camera_perception 节点在 host Foxy 上注册
  yoloe_ready=0
  for attempt in $(seq 1 30); do
    sleep 2
    if bash -c "
      source ${YOLOE_DEPLOY}/ros2_ws/install/local_setup.bash >/dev/null 2>&1
      ros2 node list 2>/dev/null | grep -q yoloe_dual_camera_perception
    " 2>/dev/null; then
      echo "[yoloe]   YOLOE node registered (${attempt} polls ~${attempt}*2s)"
      yoloe_ready=1
      break
    fi
  done
  if [ "${yoloe_ready}" != "1" ]; then
    echo "[yoloe] ERROR: YOLOE 未在 60s 内注册节点。请检查:"
    echo "[yoloe]   - ${YOLOE_DEPLOY}/ros2_ws/install/local_setup.bash 是否存在"
    echo "[yoloe]   - host Foxy cyclonedds 是否 up"
    echo "[yoloe]   - 模型 / prompt assets 是否在位"
    echo "[yoloe]   - log: ${YOLOE_LOG_HOST}"
    exit 1
  fi

  # ====================================================================
  # Phase Y1.5: Wait for CONTINUOUS_DUAL_CAMERA_STARTED (YOLOE 模型加载完成)
  # ====================================================================
  # 出处:test_yoloe_stability.sh Phase 1.5。
  # dual_camera_node.py 在 __init__ 末尾 emit 该 marker(2026-10-04 引入,
  # 模型加载 + 内部 subscriber 注册完毕信号),不依赖 cameras 已起。继续往
  # 下走 Phase Y1.6 / Y2 前 YOLOE 必须 in-process 就绪,否则 cameras USB
  # init 与模型 CPU 尖峰撞车(旧版"Phase Y2 直接起 cameras"是 2026-10-02
  # head_color ~64s stall 的根因候选之一)。
  echo "[yoloe] Phase Y1.5: wait for CONTINUOUS_DUAL_CAMERA_STARTED..."
  seen_started=0
  for attempt in $(seq 1 60); do
    sleep 2
    [ -f "${YOLOE_LOG_HOST}" ] || continue
    if grep -q 'CONTINUOUS_DUAL_CAMERA_STARTED' "${YOLOE_LOG_HOST}" 2>/dev/null; then
      echo "[yoloe]   CONTINUOUS_DUAL_CAMERA_STARTED seen (${attempt} polls ~${attempt}*2s)"
      seen_started=1
      break
    fi
  done
  if [ "${seen_started}" != "1" ]; then
    echo "[yoloe] ERROR: YOLOE 未在 120s 内 emit CONTINUOUS_DUAL_CAMERA_STARTED — 模型加载可能失败"
    echo "[yoloe]   - 确认 ${YOLOE_DEPLOY}/models/yoloe-26n-seg.pt 与 prompt_assets/ 在位"
    echo "[yoloe]   - host Foxy cyclonedds 是否 up(CONTINUOUS_DUAL_CAMERA_STARTED 在模型加载完才 emit)"
    echo "[yoloe]   - log: ${YOLOE_LOG_HOST}"
    exit 1
  fi

  # ====================================================================
  # Phase Y1.6: USB force reset (kernel unbind+bind for both cameras)
  # ====================================================================
  # 出处:test_yoloe_stability.sh Phase 1.6(2026-10-04 Path A 新增)。
  # 上一次 run realsense2_camera_node SIGSEGV 时 libusb 不跑 exit handler,
  # kernel 仍保留 device handle,下次 claim interface 0 → RS2_USB_STATUS_BUSY。
  # unbind + bind 强制 kernel 重枚举设备,等价于 kernel 端拔插。必须在
  # cameras 启动之前做(否则 in-flight USB claim 会失败)。
  #
  # 配合其它路径(沿用 test_yoloe_stability.sh 的组合):
  #   - Path C initial_reset=true(device 端硬件重置,realsense 节点侧)
  #   - Path D reconnect_timeout=15.0(给 arm 同样的 12s+ 恢复窗口)
  #   - launch 端 TimerAction 错开两相机启动 5s(避免并发抢 libusb context)
  #
  # 在容器内执行(容器是 root 且 /sys/bus/usb/ 已 bind mount)。
  # USB 端口路径默认按已知 topology 写死(head=2-1 / arm=1-3.3);
  # 若设备换了 USB 端口,需重跑 `lsusb -t` 后用 HEAD_USB_PORT= / ARM_USB_PORT= 覆盖。
  echo "[yoloe] Phase Y1.6: USB force reset (kernel unbind+bind)..."
  HEAD_USB_PORT="${HEAD_USB_PORT:-2-1}"
  ARM_USB_PORT="${ARM_USB_PORT:-1-3.3}"
  docker exec "${CONTAINER}" bash -c '
    set -u
    HEAD_PORT="'"$HEAD_USB_PORT"'"
    ARM_PORT="'"$ARM_USB_PORT"'"
    SYS=/sys/bus/usb/drivers/usb
    for pair in "${HEAD_PORT}:camera_head" "${ARM_PORT}:camera_arm"; do
      port="${pair%:*}"
      name="${pair#*:}"
      echo "[force-reset] ${name} @ port ${port}"
      if [ ! -d "/sys/bus/usb/devices/${port}" ]; then
        echo "[force-reset] ERR: /sys/bus/usb/devices/${port} 不存在" >&2
        exit 1
      fi
      echo "${port}" > "${SYS}/unbind" || {
        echo "[force-reset] ERR: unbind ${port} 失败" >&2
        exit 1
      }
      sleep 1
      echo "${port}" > "${SYS}/bind" || {
        echo "[force-reset] ERR: bind ${port} 失败" >&2
        exit 1
      }
      # 等 device 重新出现 (最多 5s)
      waited=0
      while [ "${waited}" -lt 10 ]; do
        if [ -f "/sys/bus/usb/devices/${port}/idVendor" ]; then
          echo "[force-reset] ${name} @ ${port} re-enumerated after $((waited*5))00ms"
          break
        fi
        sleep 0.5
        waited=$((waited+1))
      done
      if [ "${waited}" -ge 10 ]; then
        echo "[force-reset] ERR: ${port} bind 后 5s 内未重新出现" >&2
        exit 1
      fi
    done
    echo "[force-reset] done"
  ' || {
    echo "[yoloe] ERROR: USB force reset 失败 — 可能是 USB 端口变了或权限问题"
    echo "[yoloe]   排查:host 上跑 \`lsusb -t\` 确认 head/arm 各在哪个 port,"
    echo "[yoloe]   然后通过 HEAD_USB_PORT= / ARM_USB_PORT= 环境变量覆盖。"
    exit 1
  }
  echo "[yoloe]   USB force reset OK"

  # ====================================================================
  # Phase Y2: Start cameras in container (test_yoloe_poll_loop.sh Phase 2)
  # ====================================================================
  echo "[yoloe] Phase Y2: start cameras in container (head=${HEAD_CAMERA_SERIAL} arm=${ARM_CAMERA_SERIAL})..."
  CAMERAS_LOG_HOST="${WS_FORK_HOST}/.run/lgw_head_to_arm_verify_grasp_cameras_${LOG_TS}.log"
  # 容器内需要单独 source Humble + ros2_ws + HUMBLE_YOLOE_INSTALL(l4t_setup.sh
  # 不覆盖 yoloe_humble_orchestration 的 install)。容器内 nohup + echo PID
  # 是 test_yoloe_poll_loop.sh Phase 2 用法。
  docker exec -e LGW_INSTALL="${LGW_INSTALL}" "${CONTAINER}" bash -c "
    source /opt/ros/humble/setup.bash >/dev/null 2>&1
    source /my_ws/l4t/ros2_ws/install/local_setup.bash >/dev/null 2>&1
    source ${HUMBLE_YOLOE_INSTALL} >/dev/null 2>&1
    export HEAD_CAMERA_SERIAL='${HEAD_CAMERA_SERIAL}'
    export ARM_CAMERA_SERIAL='${ARM_CAMERA_SERIAL}'
    export ROS_DOMAIN_ID='${ROS_DOMAIN_ID_YOLOE}'
    export RMW_IMPLEMENTATION='${RMW_IMPLEMENTATION_YOLOE}'
    nohup ros2 launch yoloe_humble_orchestration two_realsense.launch.py \
      head_serial:=\"\${HEAD_CAMERA_SERIAL}\" \
      arm_serial:=\"\${ARM_CAMERA_SERIAL}\" \
      > /tmp/two_realsense_inside.log 2>&1 &
    echo \$!
  " > "${CAMERAS_LOG_HOST}" 2>&1
  CAMERAS_IN_PID=$(tail -1 "${CAMERAS_LOG_HOST}" | tr -d '[:space:]')
  echo "[yoloe]   cameras inside-container PID=${CAMERAS_IN_PID}, log → /tmp/two_realsense_inside.log"

  # 等 /camera_head/camera_head + /camera_arm/camera_arm 两个节点注册
  cameras_ready=0
  for attempt in $(seq 1 30); do
    sleep 2
    nc=$(docker exec -e LGW_INSTALL="${LGW_INSTALL}" "${CONTAINER}" bash -c "
      source /opt/ros/humble/setup.bash >/dev/null 2>&1
      source ${HUMBLE_YOLOE_INSTALL} >/dev/null 2>&1
      ros2 node list 2>/dev/null \
        | grep -E '^/camera_(head|arm)/camera_(head|arm)\$' \
        | sort -u | wc -l
    " 2>/dev/null || echo 0)
    if [ "${nc}" = "2" ]; then
      echo "[yoloe]   cameras up (${attempt} polls)"
      cameras_ready=1
      break
    fi
  done
  if [ "${cameras_ready}" != "1" ]; then
    echo "[yoloe] ERROR: cameras 未在 60s 内 ready。请检查:"
    echo "[yoloe]   - 两路 realsense 物理连接 + 序列号(HEAD_CAMERA_SERIAL=${HEAD_CAMERA_SERIAL},ARM_CAMERA_SERIAL=${ARM_CAMERA_SERIAL})"
    echo "[yoloe]   - 容器内 /dev/video* 设备映射(应 ≥2)"
    echo "[yoloe]   - 容器内 ros2_ws + HUMBLE_YOLOE_INSTALL 是否 source 通"
    echo "[yoloe]   - log: /tmp/two_realsense_inside.log(在容器内)"
    exit 1
  fi

  # ====================================================================
  # Phase Y2.5: Wait for first CONTINUOUS_RESULT for head AND arm
  #            (pipeline-alive only, no detection requirement)
  # ====================================================================
  # Y2.5 只验 "YOLOE 至少为每路相机跑过一次推理 emit CONTINUOUS_RESULT"——
  # 不验 accepted>=1,因为检测到目标是 BT 运行时的条件,不是 bringup 前置。
  # 目标物不在 FOV / 模型没匹配到 → accepted=0 但 pipeline 健康 → raw result
  # 仍然会 emit。YOLOE pipeline 真死了(crash / 没订阅 topic)则不会 emit,这条
  # 失败能抓住。
  # 相机 stream 本身(USB / 帧率)由 Phase 3 arm 起来后再启动 cameras 兜底
  # (2026-10-02 重排:先 arm,后 cameras / YOLOE,head_color stall 排查中)。
  # Working pattern 参考:test_yoloe_poll_loop.sh §Phase 2.5。
  echo "[yoloe] Phase Y2.5: wait for first CONTINUOUS_RESULT (head + arm, pipeline-alive)..."
  seen_head=0
  seen_arm=0
  for attempt in $(seq 1 60); do
    sleep 2
    [ -f "${YOLOE_LOG_HOST}" ] || continue
    grep -q  'CONTINUOUS_RESULT camera=head' "${YOLOE_LOG_HOST}" 2>/dev/null && seen_head=1
    grep -q  'CONTINUOUS_RESULT camera=arm ' "${YOLOE_LOG_HOST}" 2>/dev/null && seen_arm=1
    if [ "${seen_head}" = "1" ] && [ "${seen_arm}" = "1" ]; then
      echo "[yoloe]   first CONTINUOUS_RESULT for both cameras (${attempt} polls ~${attempt}*2s)"
      break
    fi
  done
  if [ "${seen_head}" != "1" ] || [ "${seen_arm}" != "1" ]; then
    missing=""
    [ "${seen_head}" != "1" ] && missing="${missing} head"
    [ "${seen_arm}"  != "1" ] && missing="${missing} arm"
    echo "[yoloe] ERROR: ${missing} 未在 120s 内出现 CONTINUOUS_RESULT(YOLOE pipeline 没跑起来)。请检查:"
    echo "[yoloe]   - 相机 color stream 是否健康(arm 先起后通常 USB 干净,见上)"
    echo "[yoloe]   - YOLOE dual_camera_node 是否订阅 /camera_{head,arm}/* topic"
    echo "[yoloe]   - 双 stage prompt engines 是否加载成功(INFERENCE_BACKEND 行)"
    echo "[yoloe]   - log: ${YOLOE_LOG_HOST}"
    exit 1
  fi

  # 2026-10-02 简化:删去 Y2.7 (image_raw ≥ 5 Hz 校验) + Y2.9 (35s warmup)。
  # 这两步只在 start_lgw_head_to_arm_lookat.sh / 本脚本的 fork 出现过,test_yoloe_poll_loop.sh
  # 没有;head_color ~64s stall 的根因调查另走 phase 3 测试(arm 先起).
else
  echo "[yoloe] YOLOE_AUTOSTART=0,跳过 Phase Y1/Y1.5/Y1.6/Y2/Y2.5(假设外部已起好)。Phase Y3.5 会校验 publisher。"
fi

# ====================================================================
# Phase Y3.5: 校验 /perception/two_stage/camera_{head,arm}/result 在容器内有
# publisher(与 test_yoloe_poll_loop.sh Phase 3.5 同,只验 pub_count≥1)
# ====================================================================
# 即使 YOLOE_AUTOSTART=0 也跑这步 —— 这是硬性 precondition,YOLOE 不发结果
# BT 就只能反复跑 home → home(参 compute_lookat_pose_node.cpp:277 obs
# 为空直接 FAILURE)。2026-10-02 简化:删除原版的 msg rate ≥ 1 Hz 校验,
# 改为与 test Phase 3.5 一一对应;publisher 形式活着但 emit 0 Hz 的情况留给
# 后续 Phase 3 实验(head_color stall 调查)定位。
echo "[yoloe] Phase Y3.5: verify /perception/two_stage/camera_{head,arm}/result publishers..."
for topic in \
    /perception/two_stage/camera_head/result \
    /perception/two_stage/camera_arm/result
do
  pub_ok=0
  for _ in $(seq 1 12); do
    sleep 1
    pub_count=$(docker exec -e LGW_INSTALL="${LGW_INSTALL}" "${CONTAINER}" bash -c "
      source /my_ws/l4t/l4t_setup.sh >/dev/null 2>&1
      ros2 topic info ${topic} 2>/dev/null | grep 'Publisher count' | awk '{print \$NF}'
    " 2>/dev/null || echo "?")
    if [[ "${pub_count}" =~ ^[0-9]+$ ]] && [ "${pub_count}" -ge 1 ]; then
      pub_ok=1
      break
    fi
  done
  if [ "${pub_ok}" != "1" ]; then
    echo "[yoloe] ERROR: ${topic} 在容器内无 publisher(cross-DDS 可能断)。"
    echo "[yoloe]   - 确认 source /my_ws/l4t/l4t_setup.sh 后 CYCLONEDDS_URI=eth10 生效"
    echo "[yoloe]   - 确认 host Foxy 的 YOLOE + 容器 cameras 都还在跑"
    echo "[yoloe]   - 即使 YOLOE_AUTOSTART=0 也得过这步,否则 BT 只会跑 home → home"
    exit 1
  fi
  echo "[yoloe]   ${topic}: publisher=${pub_count}"
done

# ---- 提示用户:BT 已自动开始循环 ----
# 2026-10-07 设计变更:head_to_arm_verify_grasp.xml 去掉了最外层
# WaitForUserInput / ReactiveFallback,Repeat num_cycles="-1" 直接做根。
# BT runner 一加载 XML 就开始 verify_grasp_cycle 循环 —— 不需要 send_goal,
# 也不需要 /bt_user_input 触发。终止方式只有 Ctrl-C 本脚本 → trap 走
# cleanup_on_signal → 按 PID 阶梯 kill 容器内 bt_runner / launch 进程。
echo "[bringup] Launch 已就绪 —— BT verify_grasp_cycle 自动开始循环。"
echo "            Ctrl-C 终止本脚本 → 自动按 PID 阶梯清理 launch 树。"
echo ""
echo "  ✅ 硬件安全自检 (真机必读):"
echo "      1) 机械臂 + 夹爪 + CAN 总线已上电"
echo "      2) 周围 0.5 m 范围内无障碍物 / 无关人员"
echo "      3) E-stop 按钮随手可触"
echo "      4) BT 树会真实移动机械臂 (MoveArmToPose / MoveArmToNamed"
echo "         MoveArmToNamed gripper_open/close 真实夹取)"
echo "      5) Go2 driver + YOLOE Foxy inference + 两路 realsense 都已起来"
echo ""
echo "  BT 日志(单一固定文件,truncate-on-start):"
echo "    ${LGW_LAUNCH_LOG_HOST}"
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
echo "  Ctrl-C 终止本脚本 → trap 自动按 PID 阶梯清理 launch 树。"
echo ""

# 前台等 Ctrl-C:sleep 期间若收到 SIGINT/SIGTERM,bash 转给 trap,
# trap 调用 cleanup_on_signal → 按 PID 阶梯 TERM→3s→KILL → exit 130
echo "=== 等 Ctrl-C ==="
sleep infinity
