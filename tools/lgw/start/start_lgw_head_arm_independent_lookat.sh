#!/bin/bash
# scripts/start_lgw_head_arm_independent_lookat.sh
#
# 一键启动 HeadArmIndependentLookat BT —— 与 head_to_arm_verify_grasp 同源,
# 但去掉 head/arm 一致性核实,改为在同一个 Sequence 内先后驱动 tcp 到
# "head 视角算出的目标位置" 和 "arm 视角算出的目标位置"。用户通过观察
# 这两次到达的物理位置,反推误差来自 head→world 还是 arm→tcp→world
# 这条 transform 链。
#
# 完整 bringup 链(对齐 start_lgw_head_to_arm_verify_grasp.sh):
#   Phase 3 (容器)    CAN → agx_arm_ctrl → MoveIt → bt_runner
#   Phase Y1 (host)   YOLOE Foxy inference loop
#   Phase Y1.5        等 YOLOE emit CONTINUOUS_DUAL_CAMERA_STARTED
#   Phase Y1.6        USB kernel unbind+bind 强制重枚举两路相机
#   Phase Y2 (容器)   两路 realsense 相机 (two_realsense.launch.py)
#   Phase Y2.5        等 head + arm 首个 CONTINUOUS_RESULT
#   Phase Y3.5        校验 /perception/two_stage/camera_{head,arm}/result
#                     publisher 存在
#
# 启动后:BT verify_grasp_cycle 立即开始循环 —— 不需要 send_goal,也不需
# 要 /bt_user_input 触发(2026-10-07 起去掉了最外层 WaitForUserInput)。
# 终止:Ctrl-C 本脚本 → trap 走 cleanup_on_signal → 按 PID 阶梯 kill
# 容器内 bt_runner / launch 进程。
#
# 与 start_lgw_head_to_arm_verify_grasp.sh 的关键差异:
#   - BT_XML 默认 head_arm_independent_lookat.xml (TreeID=HeadArmIndependentLookat)
#   - .run 日志目录指向 ${WS_FORK}/.run/,前缀
#     lgw_head_arm_independent_lookat_*
#   - BT 日志单一固定文件: .run/lgw_head_arm_independent_lookat_bt.log
#   - 其他 phase / cleanup / 信号处理保持一致
#
# Env override(可选):
#   YOLOE_AUTOSTART=0           跳过 Y1/Y2/Y2.5(假设外部已起好)
#   HEAD_CAMERA_SERIAL=...      默认 _261622078621
#   ARM_CAMERA_SERIAL=...       默认 _262422071246
#   YOLOE_DEPLOY=$HOME/...      默认 $HOME/yoloe_deploy
#   HUMBLE_YOLOE_INSTALL=...    默认 /opt/yoloe_transfer/humble_ws/install/local_setup.bash
#   HEAD_USB_PORT=2-1           head D435i USB 拓扑
#   ARM_USB_PORT=1-3.3          arm D435 USB 拓扑
#
# 硬件安全(真机必读):
#   1) 机械臂 + 夹爪 + CAN 总线已上电
#   2) 周围 0.5 m 范围内无障碍物 / 无关人员
#   3) E-stop 按钮随手可触
#   4) BT 树会真实移动机械臂(每个 cycle head→arm→home 各一次 MoveArmTo*)
#   5) Go2 driver + YOLOE Foxy inference + 两路 realsense 都已起来

set -u
CONTAINER="self_l4t"

# shellcheck source=../_common.sh
source "$(dirname "${BASH_SOURCE[0]}")/../_common.sh"

# 与 start_lgw_head_to_arm_verify_grasp.sh 完全一致的容器内 PID pattern
readonly LG_STACK_PIDS_PATTERN='bt_with_lgw_moveit\.launch\.py|lgw_bt_planner\.launch\.py|demo\.launch\.py|start_single_agx_arm|agx_arm_ctrl_single|bt_runner|move_group|robot_state_publisher|ros2_control_node|controller_manager|spawner_joint_state_broadcaster|spawner_arm_controller|spawner_gripper_controller|l4t_setup\.sh|can_activate\.sh|two_realsense\.launch\.py|realsense2_camera_node|yoloe_humble_orchestration|select_arm_observation_pose_service_node'

# BT_XML 默认指向新树(TreeID=HeadArmIndependentLookat)
readonly DEFAULT_BT_XML="${WS_FORK_CT}/install/lgw_bt_planner/share/lgw_bt_planner/bt_trees/head_arm_independent_lookat.xml"
BT_XML="${BT_XML:-${DEFAULT_BT_XML}}"
readonly TREE_ID="HeadArmIndependentLookat"

# ---- YOLOE 配置(参考 test_yoloe_poll_loop.sh Phase 1+2) ----
YOLOE_AUTOSTART="${YOLOE_AUTOSTART:-1}"
HEAD_CAMERA_SERIAL="${HEAD_CAMERA_SERIAL:-_261622078621}"
ARM_CAMERA_SERIAL="${ARM_CAMERA_SERIAL:-_262422071246}"
YOLOE_DEPLOY="${YOLOE_DEPLOY:-$HOME/yoloe_deploy}"
HUMBLE_YOLOE_INSTALL="${HUMBLE_YOLOE_INSTALL:-/opt/yoloe_transfer/humble_ws/install/local_setup.bash}"
ROS_DOMAIN_ID_YOLOE="${ROS_DOMAIN_ID_YOLOE:-0}"
RMW_IMPLEMENTATION_YOLOE="${RMW_IMPLEMENTATION_YOLOE:-rmw_cyclonedds_cpp}"
HEAD_USB_PORT="${HEAD_USB_PORT:-2-1}"
ARM_USB_PORT="${ARM_USB_PORT:-1-3.3}"
export YOLOE_DEPLOY

HOST_PIDS=()

# ---- Ctrl-C 自动清理(同 verify_grasp) ----
cleanup_on_signal() {
  if [ "${CLEANUP_DONE:-0}" = "1" ]; then return; fi
  CLEANUP_DONE=1
  echo ""
  echo "[cleanup] Ctrl-C / 信号触发,清理 launch 进程(容器内 PID + host PID)..."

  local pids
  pids=$(docker exec "${CONTAINER}" bash -c "
    ps -eo pid,args | \
    grep -E '${LG_STACK_PIDS_PATTERN}' | \
    grep -v grep | \
    awk '{print \$1}'
  " 2>/dev/null) || pids=""

  if [ -n "${pids}" ]; then
    echo "[cleanup] kill candidates (container pids): ${pids}"
    for pid in ${pids}; do
      echo "[cleanup] TERM container pid=${pid}"
      docker exec "${CONTAINER}" kill -TERM "${pid}" 2>/dev/null || true
    done
    sleep 3
    for pid in ${pids}; do
      if docker exec "${CONTAINER}" kill -0 "${pid}" 2>/dev/null; then
        echo "[cleanup] KILL container pid=${pid}"
        docker exec "${CONTAINER}" kill -KILL "${pid}" 2>/dev/null || true
      fi
    done
  fi

  # Orphan realsense2_camera_node
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

  # Host-side PID 清理
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

  # Host orphan dual_camera_node
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

# ---- preflight(同 verify_grasp) ----
preflight_cleanup() {
  echo "[preflight] 清理 host 上 YOLOE + container 内 cameras 残留进程..."
  local pids

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

  pids=$(docker exec "${CONTAINER}" bash -c "
    ps -eo pid,ppid,args 2>/dev/null \
      | awk '\$2 == 1' \
      | grep 'realsense2_camera_node' \
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

  sleep 2
  echo "[preflight] done"
}

trap cleanup_on_signal INT TERM
preflight_cleanup

docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$" \
  || docker start "${CONTAINER}" >/dev/null

# L2 wrapper
echo "[bringup] L2 wrapper (CAN → agx_arm_ctrl → MoveIt → bt_runner) ..."
export LGW_INSTALL="${WS_FORK_CT}/install/setup.bash"
echo "[bringup] LGW_INSTALL=${LGW_INSTALL}"

LOG_TS="$(date +%Y%m%d_%H%M%S)"
docker exec "${CONTAINER}" mkdir -p "${WS_FORK_CT}/.run"
docker exec "${CONTAINER}" chmod 777 "${WS_FORK_CT}/.run" 2>/dev/null || true

# ====================================================================
# Phase 3: CAN + driver + combined launch
# ====================================================================

# Step 1: CAN 总线 up
echo "[bringup] Step 1: 激活 CAN 总线 can0 @ 1 Mbps..."
if ! docker exec "${CONTAINER}" bash \
    /my_ws/l4t/agx_arm_ws/src/agx_arm_ros/scripts/can_activate.sh can0 1000000 2>&1 | tail -3; then
  echo "[bringup]   ERROR: can_activate.sh 失败"
  exit 1
fi

# Step 2: agx_arm_ctrl hardware driver
echo "[bringup] Step 2: 启动 agx_arm_ctrl hardware driver (detached)..."
DRIVER_LOG="${WS_FORK_CT}/.run/lgw_head_arm_independent_lookat_driver_${LOG_TS}.log"
DRIVER_LOG_HOST="${WS_FORK_HOST}/.run/lgw_head_arm_independent_lookat_driver_${LOG_TS}.log"
docker exec -e LGW_INSTALL="${LGW_INSTALL}" -d "${CONTAINER}" bash -lc "
  source /my_ws/l4t/l4t_setup.sh
    source ${LGW_INSTALL:-${WS_FORK_CT}/install/setup.bash}
  ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
    effector_type:=agx_gripper tcp_offset:='[0.0, 0.0, 0.12, 0.0, 0.0, 0.0]' \
    > '${DRIVER_LOG}' 2>&1
"
echo "[bringup]   driver log → ${DRIVER_LOG_HOST}"

WAITED=0
while [ ${WAITED} -lt 15 ]; do
  if docker exec "${CONTAINER}" bash -lc "
    source /my_ws/l4t/l4t_setup.sh
    source ${LGW_INSTALL:-${WS_FORK_CT}/install/setup.bash}
    ros2 node list 2>/dev/null | grep -q '/agx_arm_ctrl_single_node\$'
  " >/dev/null 2>&1; then
    echo "[bringup]   agx_arm_ctrl_single_node 就绪 (等了 ${WAITED}s)"
    break
  fi
  sleep 1
  WAITED=$((WAITED + 1))
done
if [ ${WAITED} -eq 15 ]; then
  echo "[bringup]   WARN: 15s 内 driver 未就绪"
fi

# Step 3: MoveIt 单独起(走自己的 log),BT 单独起(走自己的 log)
# 之前用 bt_with_lgw_moveit.launch.py 一起起,BT 节点 INFO 和 move_group
# 噪音混在同一份 .run/..._bt.log,debug 时翻日志痛;拆成两个独立
# ros2 launch 后,各自截到自己的 .run/..._moveit.log / .run/..._bt.log
# (顺序:MoveIt 先,等 /move_action → 再起 BT,等 /lgw_bt_execution)

# Step 3a: MoveIt
echo "[bringup] Step 3a: 启动 MoveIt (agx_arm_moveit/launch/demo.launch.py,detached)..."
MOVEIT_LOG="${WS_FORK_CT}/.run/lgw_head_arm_independent_lookat_moveit.log"
MOVEIT_LOG_HOST="${WS_FORK_HOST}/.run/lgw_head_arm_independent_lookat_moveit.log"
: > "${MOVEIT_LOG_HOST}" || true
docker exec -e LGW_INSTALL="${LGW_INSTALL}" -d "${CONTAINER}" bash -lc "
  source /my_ws/l4t/l4t_setup.sh
    source ${LGW_INSTALL:-${WS_FORK_CT}/install/setup.bash}
  ros2 launch agx_arm_moveit demo.launch.py \
    use_rviz:=false follow:=true \
    tcp_offset:='[0.0, 0.0, 0.12, 0.0, 0.0, 0.0]' \
    effector_type:=agx_gripper \
    >> '${MOVEIT_LOG}' 2>&1
"
echo "[bringup]   moveit log → ${MOVEIT_LOG_HOST}"

WAITED=0
while [ ${WAITED} -lt 30 ]; do
  if docker exec "${CONTAINER}" bash -lc "
    source /my_ws/l4t/l4t_setup.sh
    source ${LGW_INSTALL:-${WS_FORK_CT}/install/setup.bash}
    ros2 action list 2>/dev/null | grep -q '^/move_action\$'
  " >/dev/null 2>&1; then
    echo "[bringup]   /move_action action server 就绪 (等了 ${WAITED}s)"
    break
  fi
  sleep 1
  WAITED=$((WAITED + 1))
done
if [ ${WAITED} -eq 30 ]; then
  echo "[bringup]   WARN: 30s 内 /move_action 未起来,后续 MoveArmTo* 会 FAIL"
fi

# Step 3b: BT runner(单独的 log)
echo "[bringup] Step 3b: 启动 BT runner (lgw_bt_planner.launch.py,detached)..."
LGW_LAUNCH_LOG="${WS_FORK_CT}/.run/lgw_head_arm_independent_lookat_bt.log"
LGW_LAUNCH_LOG_HOST="${WS_FORK_HOST}/.run/lgw_head_arm_independent_lookat_bt.log"
: > "${LGW_LAUNCH_LOG_HOST}" || true
docker exec -e LGW_INSTALL="${LGW_INSTALL}" -e BT_PLANNER_DEBUG=1 -d "${CONTAINER}" bash -lc "
  source /my_ws/l4t/l4t_setup.sh
    source ${LGW_INSTALL:-${WS_FORK_CT}/install/setup.bash}
  ros2 launch lgw_bt_planner lgw_bt_planner.launch.py \
    >> '${LGW_LAUNCH_LOG}' 2>&1
"
echo "[bringup]   bt log → ${LGW_LAUNCH_LOG_HOST}"

WAITED=0
while [ ${WAITED} -lt 30 ]; do
  if docker exec "${CONTAINER}" bash -lc "
    source /my_ws/l4t/l4t_setup.sh
    source ${LGW_INSTALL:-${WS_FORK_CT}/install/setup.bash}
    ros2 action list 2>/dev/null | grep -q '^/lgw_bt_execution\$'
  " >/dev/null 2>&1; then
    echo "[bringup]   /lgw_bt_execution action server 就绪 (等了 ${WAITED}s)"
    break
  fi
  sleep 1
  WAITED=$((WAITED + 1))
done
if [ ${WAITED} -eq 30 ]; then
  echo "[bringup]   WARN: 30s 内 action server 未起来"
fi

# /select_arm_observation_pose service 由 lgw_bt_planner.launch.py
# (Step 3b) 自动起。等它就绪再 send_goal,否则 BT 第一次 tick
# SelectArmObservationPose 会因 service 不可用 FAILURE。
WAITED=0
while [ ${WAITED} -lt 10 ]; do
  if docker exec "${CONTAINER}" bash -lc "
    source /my_ws/l4t/l4t_setup.sh
    source ${LGW_INSTALL:-${WS_FORK_CT}/install/setup.bash}
    ros2 service list 2>/dev/null | grep -q '^/select_arm_observation_pose\$'
  " >/dev/null 2>&1; then
    echo "[bringup]   /select_arm_observation_pose service 就绪 (等了 ${WAITED}s)"
    break
  fi
  sleep 1
  WAITED=$((WAITED + 1))
done
if [ ${WAITED} -eq 10 ]; then
  echo "[bringup]   WARN: 10s 内 /select_arm_observation_pose service 未起来"
  echo "[bringup]          BT 走到 SelectArmObservationPose 会 FAILURE"
fi

# ====================================================================
# Phase Y1: YOLOE on host
# ====================================================================
if [ "${YOLOE_AUTOSTART}" = "1" ]; then
  echo "[yoloe] Phase Y1: start YOLOE Foxy on host..."
  YOLOE_LOG_HOST="${WS_FORK_HOST}/.run/lgw_head_arm_independent_lookat_yoloe_${LOG_TS}.log"
  nohup bash -c "
    set +u
    set -a
    source ${YOLOE_DEPLOY}/config/foxy_humble_dds.env
    set +a
    source /opt/ros/foxy/setup.bash
    source ${YOLOE_DEPLOY}/ros2_ws/install/local_setup.bash
    export LD_PRELOAD=/lib/aarch64-linux-gnu/libgomp.so.1
    export PYUNBUFFERED=1
    ros2 launch yoloe_perception dual_camera_foxy.launch.py \
        transfer_root:=\"\${YOLOE_DEPLOY}\"
  " > "${YOLOE_LOG_HOST}" 2>&1 &
  YOLOE_PID=$!
  HOST_PIDS+=("${YOLOE_PID}")
  echo "[yoloe]   YOLOE host PID=${YOLOE_PID}, log → ${YOLOE_LOG_HOST}"

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
    echo "[yoloe] ERROR: YOLOE 未在 60s 内注册"
    exit 1
  fi

  # Phase Y1.5
  echo "[yoloe] Phase Y1.5: wait for CONTINUOUS_DUAL_CAMERA_STARTED..."
  seen_started=0
  for attempt in $(seq 1 60); do
    sleep 2
    [ -f "${YOLOE_LOG_HOST}" ] || continue
    if grep -q 'CONTINUOUS_DUAL_CAMERA_STARTED' "${YOLOE_LOG_HOST}" 2>/dev/null; then
      echo "[yoloe]   CONTINUOUS_DUAL_CAMERA_STARTED seen"
      seen_started=1
      break
    fi
  done
  if [ "${seen_started}" != "1" ]; then
    echo "[yoloe] ERROR: YOLOE 未在 120s 内 emit CONTINUOUS_DUAL_CAMERA_STARTED"
    exit 1
  fi

  # Phase Y1.6
  echo "[yoloe] Phase Y1.6: USB force reset..."
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
      echo "${port}" > "${SYS}/unbind" || exit 1
      sleep 1
      echo "${port}" > "${SYS}/bind" || exit 1
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
    echo "[yoloe] ERROR: USB force reset 失败"
    exit 1
  }
  echo "[yoloe]   USB force reset OK"

  # Phase Y2
  echo "[yoloe] Phase Y2: start cameras in container..."
  CAMERAS_LOG_HOST="${WS_FORK_HOST}/.run/lgw_head_arm_independent_lookat_cameras_${LOG_TS}.log"
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
  echo "[yoloe]   cameras inside-container PID=${CAMERAS_IN_PID}"

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
    echo "[yoloe] ERROR: cameras 未在 60s 内 ready"
    exit 1
  fi

  # Phase Y2.5
  echo "[yoloe] Phase Y2.5: wait for first CONTINUOUS_RESULT (head + arm)..."
  seen_head=0
  seen_arm=0
  for attempt in $(seq 1 60); do
    sleep 2
    [ -f "${YOLOE_LOG_HOST}" ] || continue
    grep -q  'CONTINUOUS_RESULT camera=head' "${YOLOE_LOG_HOST}" 2>/dev/null && seen_head=1
    grep -q  'CONTINUOUS_RESULT camera=arm ' "${YOLOE_LOG_HOST}" 2>/dev/null && seen_arm=1
    if [ "${seen_head}" = "1" ] && [ "${seen_arm}" = "1" ]; then
      echo "[yoloe]   first CONTINUOUS_RESULT for both cameras"
      break
    fi
  done
  if [ "${seen_head}" != "1" ] || [ "${seen_arm}" != "1" ]; then
    missing=""
    [ "${seen_head}" != "1" ] && missing="${missing} head"
    [ "${seen_arm}"  != "1" ] && missing="${missing} arm"
    echo "[yoloe] ERROR: ${missing} 未在 120s 内出现 CONTINUOUS_RESULT"
    exit 1
  fi
else
  echo "[yoloe] YOLOE_AUTOSTART=0,跳过 Phase Y1/Y1.5/Y1.6/Y2/Y2.5"
fi

# ====================================================================
# Phase Y3.5
# ====================================================================
# 替代原 ros2 topic info polling:原方案在容器内
# `docker exec ... bash -c "ros2 topic info <topic>"`
# 1) 缺 `-l` login shell,无法加载 /etc/profile;2) YOLOE 在 host
# Foxy 跑、Humble 容器订阅,跨 distro DDS discovery 偶发不可见;
# 3) `2>/dev/null` 吞掉所有错误,误报率极高。改为直接 tail YOLOE
# 日志统计 CONTINUOUS_RESULT camera={head,arm} 出现次数 —— YOLOE 每
# 完成一帧推理就 emit 一次,稳态 5-10Hz,只要 pipeline 还在跑,
# 30s 内一定能集到 ≥3 条;日志 truncate-on-start (`nohup ... > LOG`),
# 旧 run 的残留条目不会污染判断。
echo "[yoloe] Phase Y3.5: verify CONTINUOUS_RESULT 流 (head + arm 各 ≥3 次,日志近 30 行)..."
[ -f "${YOLOE_LOG_HOST}" ] || {
  echo "[yoloe] ERROR: YOLOE 日志不存在 (${YOLOE_LOG_HOST}),Phase Y1 应先就绪"
  exit 1
}
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
    echo "[yoloe] ERROR: ${cam} 相机 30s 内未连续出现 ≥3 次 CONTINUOUS_RESULT"
    echo "[yoloe]        末 20 行 log 摘录:"
    tail -n 20 "${YOLOE_LOG_HOST}" | sed 's/^/             /'
    exit 1
  fi
  echo "[yoloe]   ${cam}: CONTINUOUS_RESULT 连续出现 (近 30 行 ≥3 次)"
done

# ---- 提示:BT 已就绪,需 send_goal 启动 ----
echo "[bringup] Launch 已就绪 —— /lgw_bt_execution action server 已 UP。"
echo "            BT 不会自动跑,需在另一终端 send_goal 启动循环;Ctrl-C 本脚本"
echo "            → 自动按 PID 阶梯清理 launch 树(同时取消 action goal)。"
echo ""
echo "  ✅ 硬件安全自检 (真机必读):"
echo "      1) 机械臂 + 夹爪 + CAN 总线已上电"
echo "      2) 周围 0.5 m 范围内无障碍物 / 无关人员"
echo "      3) E-stop 按钮随手可触"
echo "      4) BT 树会真实移动机械臂 (每 cycle head→arm→home)"
echo "      5) Go2 driver + YOLOE Foxy inference + 两路 realsense 都已起来"
echo ""
echo "  日志(BT 与 MoveIt 各自一份,truncate-on-start):"
echo "    MoveIt  (move_group / controllers / robot_state_publisher):"
echo "      ${MOVEIT_LOG_HOST}"
echo "    BT      (bt_runner + select_arm service + Compute* + MoveArmTo*):"
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
echo "  本脚本 Ctrl-C 仍然会清理整个 launch 树(且会顺带 cancel action)。"
echo ""
echo ""

echo "=== 等 Ctrl-C ==="
sleep infinity
