#!/bin/bash
# scripts/start_lgw_arm.sh
#
# 机械臂 bringup:CAN 总线 + agx_arm_ctrl hardware driver + MoveIt
# (agx_arm_moveit/demo.launch.py)。脚本就绪后 /move_action action server
# 已 UP,可独立长期跑(不会退出),给 start_lgw_bt.sh 当底盘。
#
# 启动项(对应原 start_lgw_head_arm_independent_lookat.sh 的 Step 1/2/3a):
#   Step 1  CAN 总线 can0 @ 1 Mbps
#   Step 2  agx_arm_ctrl_single_node  (hardware driver)
#   Step 3a MoveIt (move_group + controllers + robot_state_publisher)
#
# 不启动:BT runner(在 start_lgw_bt.sh)、YOLOE(在 start_lgw_yoloe.sh)。
# 多次重启本脚本不会冲突 CAN / driver / move_group —— preflight 会先清
# 掉上次残留。
#
# Env override(可选):
#   CONTAINER=self_l4t   Docker 容器名
#
# 终止:Ctrl-C → cleanup_arm 按 PID 阶梯 kill 容器内 arm 相关进程
# (driver / move_group / controllers / robot_state_publisher /
#  can_activate.sh),不影响 YOLOE / BT(各自独立 cleanup)。
#
# 硬件安全(真机必读):
#   1) 机械臂 + 夹爪 + CAN 总线已上电
#   2) 周围 0.5 m 范围内无障碍物 / 无关人员
#   3) E-stop 按钮随手可触

set -u
CONTAINER="${CONTAINER:-self_l4t}"

# --driver-only: 只做 Step 1 (CAN) + Step 2 (agx_arm_ctrl),跳过 Step 3a 的
# MoveIt。lgw_full.launch.py 自己会 include agx_arm_moveit/demo.launch.py
# (它文档里写明 "the ONLY controller_manager / RSP in this stack"),两者都起
# 就是两个 controller_manager。真机跑 servo 全栈时用这个开关:
#
#   bash scripts/start_lgw_arm.sh --driver-only      # CAN + driver
#   ros2 launch lgw_bt_planner lgw_full.launch.py follow:=true use_bt:=false
DRIVER_ONLY=0
for arg in "$@"; do
  case "${arg}" in
    --driver-only) DRIVER_ONLY=1 ;;
    -h|--help)
      echo "用法: bash scripts/start_lgw_arm.sh [--driver-only]"
      echo "  (默认)        CAN + driver + MoveIt"
      echo "  --driver-only CAN + driver only (给 lgw_full.launch.py 当底座)"
      exit 0
      ;;
    *)
      echo "[arm] 未知参数: ${arg} (试 --help)" >&2
      exit 2
      ;;
  esac
done

# shellcheck source=../_common.sh
source "$(dirname "${BASH_SOURCE[0]}")/../_common.sh"

# 仅 arm 相关 PID pattern(不含 BT / YOLOE,各自脚本独立 cleanup)
readonly LG_ARM_PIDS_PATTERN='start_single_agx_arm|agx_arm_ctrl_single|robot_state_publisher|ros2_control_node|controller_manager|spawner_joint_state_broadcaster|spawner_arm_controller|spawner_gripper_controller|can_activate\.sh|demo\.launch\.py|move_group|joint_state_publisher_gui|rviz2'

# ---- Ctrl-C 清理 ----
cleanup_on_signal() {
  if [ "${CLEANUP_DONE:-0}" = "1" ]; then return; fi
  CLEANUP_DONE=1
  echo ""
  echo "[arm-cleanup] Ctrl-C / 信号触发,清理 arm bringup 进程(容器内)..."

  local pids
  pids=$(docker exec "${CONTAINER}" bash -c "
    ps -eo pid,args 2>/dev/null | \
    grep -E '${LG_ARM_PIDS_PATTERN}' | \
    grep -v grep | \
    awk '{print \$1}'
  " 2>/dev/null) || pids=""

  if [ -n "${pids}" ]; then
    echo "[arm-cleanup] kill candidates (container pids): ${pids}"
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

  echo "[arm-cleanup] done"
}

# ---- preflight(只清 arm 残留,不碰 BT / YOLOE) ----
preflight_cleanup() {
  echo "[arm-preflight] 清理容器内 arm 残留进程..."
  local pids
  pids=$(docker exec "${CONTAINER}" bash -c "
    ps -eo pid,args 2>/dev/null | \
    grep -E '${LG_ARM_PIDS_PATTERN}' | \
    grep -v grep | \
    awk '{print \$1}'
  " 2>/dev/null) || pids=""
  if [ -n "${pids}" ]; then
    echo "[arm-preflight] candidates: ${pids}"
    for pid in ${pids}; do
      docker exec "${CONTAINER}" kill -TERM "${pid}" 2>/dev/null || true
    done
    sleep 3
    for pid in ${pids}; do
      if docker exec "${CONTAINER}" kill -0 "${pid}" 2>/dev/null; then
        echo "[arm-preflight]   KILL container pid=${pid}"
        docker exec "${CONTAINER}" kill -KILL "${pid}" 2>/dev/null || true
      fi
    done
  fi
  sleep 1
  echo "[arm-preflight] done"
}

trap cleanup_on_signal INT TERM
preflight_cleanup

docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$" \
  || docker start "${CONTAINER}" >/dev/null

# L2 wrapper env
export LGW_INSTALL="${WS_FORK_CT}/install/setup.bash"
echo "[arm] LGW_INSTALL=${LGW_INSTALL}"

LOG_TS="$(date +%Y%m%d_%H%M%S)"
docker exec "${CONTAINER}" mkdir -p "${WS_FORK_CT}/.run"
docker exec "${CONTAINER}" chmod 777 "${WS_FORK_CT}/.run" 2>/dev/null || true

# ====================================================================
# Step 1: CAN 总线 up
# ====================================================================
echo "[arm] Step 1: 激活 CAN 总线 can0 @ 1 Mbps..."
if ! docker exec "${CONTAINER}" bash \
    ${AGX_CAN_ACTIVATE_CT} can0 1000000 2>&1 | tail -3; then
  echo "[arm]   ERROR: can_activate.sh 失败"
  exit 1
fi

# ====================================================================
# Step 2: agx_arm_ctrl hardware driver
# ====================================================================
echo "[arm] Step 2: 启动 agx_arm_ctrl hardware driver (detached)..."
DRIVER_LOG="${WS_FORK_CT}/.run/lgw_arm_driver_${LOG_TS}.log"
DRIVER_LOG_HOST="${WS_FORK_HOST}/.run/lgw_arm_driver_${LOG_TS}.log"
docker exec -e LGW_INSTALL="${LGW_INSTALL}" -d "${CONTAINER}" bash -lc "
  source /my_ws/l4t/l4t_setup.sh
    source ${LGW_INSTALL:-${WS_FORK_CT}/install/setup.bash}
  # speed_percent 是固件规划类运动(move_j/P/L/C)的速度百分比；fast_mode:=true
  # 会把 /control/joint_states 换成 move_js(MIT: 无平滑/无规划/无限速)，而驱动
  # 的 move_home 与 emergency_stop 也按 is_mit_mode 分支 —— 控制链一断就是满速
  # 直冲(2026-10-30 末端支架因此损坏)。保持 false，用 80 配速换跟踪余量。
  ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \
    effector_type:=agx_gripper tcp_offset:='[0.0, 0.0, 0.12, 0.0, 0.0, 0.0]' \
    speed_percent:=80 fast_mode:=false \
    > '${DRIVER_LOG}' 2>&1
"
echo "[arm]   driver log → ${DRIVER_LOG_HOST}"

WAITED=0
while [ ${WAITED} -lt 15 ]; do
  if docker exec "${CONTAINER}" bash -lc "
    source /my_ws/l4t/l4t_setup.sh
    source ${LGW_INSTALL:-${WS_FORK_CT}/install/setup.bash}
    ros2 node list 2>/dev/null | grep -q '/agx_arm_ctrl_single_node\$'
  " >/dev/null 2>&1; then
    echo "[arm]   agx_arm_ctrl_single_node 就绪 (等了 ${WAITED}s)"
    break
  fi
  sleep 1
  WAITED=$((WAITED + 1))
done
if [ ${WAITED} -eq 15 ]; then
  echo "[arm]   WARN: 15s 内 driver 未就绪"
fi

# ====================================================================
# Step 3a: MoveIt
# ====================================================================
if [ "${DRIVER_ONLY}" = "0" ]; then
echo "[arm] Step 3a: 启动 MoveIt (agx_arm_moveit/launch/demo.launch.py,detached)..."
MOVEIT_LOG="${WS_FORK_CT}/.run/lgw_arm_moveit.log"
MOVEIT_LOG_HOST="${WS_FORK_HOST}/.run/lgw_arm_moveit.log"
: > "${MOVEIT_LOG_HOST}" || true
docker exec -e LGW_INSTALL="${LGW_INSTALL}" -d "${CONTAINER}" bash -lc "
  source /my_ws/l4t/l4t_setup.sh
    source ${LGW_INSTALL:-${WS_FORK_CT}/install/setup.bash}
  ros2 launch agx_arm_moveit demo.launch.py \
    use_rviz:=false follow:=true \
    tcp_offset:='[0.0, 0.0, 0.12, 0.0, 0.0, 0.0]' \
    effector_type:=agx_gripper \
    frame_prefix:='arm1/' \
    >> '${MOVEIT_LOG}' 2>&1
"
echo "[arm]   moveit log → ${MOVEIT_LOG_HOST}"
echo "[arm]   机械臂 TF frame 前缀 = 'arm1/' (arm1/tcp_link 等;置空可退回裸名)"

WAITED=0
while [ ${WAITED} -lt 30 ]; do
  if docker exec "${CONTAINER}" bash -lc "
    source /my_ws/l4t/l4t_setup.sh
    source ${LGW_INSTALL:-${WS_FORK_CT}/install/setup.bash}
    ros2 action list 2>/dev/null | grep -q '^/move_action\$'
  " >/dev/null 2>&1; then
    echo "[arm]   /move_action action server 就绪 (等了 ${WAITED}s)"
    break
  fi
  sleep 1
  WAITED=$((WAITED + 1))
done
if [ ${WAITED} -eq 30 ]; then
  echo "[arm]   WARN: 30s 内 /move_action 未起来"
fi
fi  # DRIVER_ONLY

# ---- 提示 ----
if [ "${DRIVER_ONLY}" = "1" ]; then
  echo "[arm] 仅 driver 模式完成 —— CAN + agx_arm_ctrl 已就绪,MoveIt 未启动。"
  echo "      下一步(另开终端):"
  echo "        ros2 launch lgw_bt_planner lgw_full.launch.py follow:=true use_bt:=false"
  echo "      保持本脚本运行(不要 Ctrl-C): driver 退出会让 MoveIt/Servo 失去反馈。"
  echo ""
  echo "  日志:"
  echo "    driver: ${DRIVER_LOG_HOST}"
else
  echo "[arm] 机械臂 bringup 完成 —— /move_action 已就绪。"
  echo "      保持本脚本运行(不要 Ctrl-C),start_lgw_bt.sh 才会正常工作。"
  echo ""
  echo "  日志:"
  echo "    driver: ${DRIVER_LOG_HOST}"
  echo "    moveit: ${MOVEIT_LOG_HOST}"
fi
echo ""
echo "=== 等 Ctrl-C ==="
sleep infinity
