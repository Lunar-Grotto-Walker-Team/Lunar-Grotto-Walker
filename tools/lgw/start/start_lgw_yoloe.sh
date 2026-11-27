#!/bin/bash
# scripts/start_lgw_yoloe.sh
#
# YOLOE 推理 bringup:host Foxy inference loop + 容器内两路 realsense 相机
# (yoloe_humble_orchestration/two_realsense.launch.py)。
#
# 启动项(对应原 start_lgw_head_arm_independent_lookat.sh 的 Phase
# Y1/Y1.5/Y1.6/Y2/Y2.5):
#   Y1    YOLOE Foxy 在 host 上跑(ros2 launch yoloe_perception
#         dual_camera_foxy.launch.py)
#   Y1.5  等 YOLOE emit CONTINUOUS_DUAL_CAMERA_STARTED
#   Y1.6  USB kernel unbind+bind 强制重枚举两路相机
#   Y2    容器内两路 realsense (two_realsense.launch.py)
#   Y2.5  等 head + arm 首个 CONTINUOUS_RESULT
#
# Y3.5(校验 CONTINUOUS_RESULT 流)在 start_lgw_bt.sh 里完成 —— 它读
# ${WS_FORK_HOST}/.run/lgw_yoloe_current.log(本脚本写入的固定路径,
# 区别于原 timestamp 路径,跨脚本才能找到)。
#
# 不启动:arm / MoveIt(在 start_lgw_arm.sh)、BT(在 start_lgw_bt.sh)。
#
# lo 模式(2026-11-16):加 --lo 让 cameras + YOLOE 整体跑在 lo 接口 + domain 7,
# 大帧不再走 eth10。此时主栈(domain 0)需靠 scripts/start_lgw_perception_bridge.sh
# 才能拿到检测结果(start_lgw_chain.sh 会自动起)。
#   bash scripts/start_lgw_yoloe.sh --lo
# 不加 --lo = 原来的 domain 0 + eth10,行为完全不变。
#
# Env override(可选):
#   HEAD_CAMERA_SERIAL=...      默认 _261622078621
#   ARM_CAMERA_SERIAL=...       默认 _262422071246
#   YOLOE_DEPLOY=$HOME/...      默认 $HOME/yoloe_deploy
#   HUMBLE_YOLOE_INSTALL=...    默认 /opt/yoloe_transfer/humble_ws/install/local_setup.bash
#   ROS_DOMAIN_ID_YOLOE=0
#   RMW_IMPLEMENTATION_YOLOE=rmw_cyclonedds_cpp
#   HEAD_USB_PORT=2-1           head D435i USB 拓扑
#   ARM_USB_PORT=1-3.3          arm D435 USB 拓扑
#   OVERLAY_MIN_PERIOD_SEC=999.0  overlay 最小发布间隔(秒)
#                                999.0=几乎不发(生产节能)
#                                0.1  =10Hz 持续流(调试 trajectory_visualizer 时覆盖)
#
# 终止:Ctrl-C → cleanup_yoloe kill host 端 YOLOE + 容器内 realsense,
# 不影响 arm / BT(各自独立 cleanup)。

set -u
CONTAINER="${CONTAINER:-self_l4t}"

# shellcheck source=../_common.sh
source "$(dirname "${BASH_SOURCE[0]}")/../_common.sh"

# YOLOE + 容器内 cameras 的 PID pattern(不含 arm / BT)
readonly LG_YOLOE_HOST_PATTERN='ros2 launch yoloe_perception|dual_camera_node|yoloe_perception'
readonly LG_YOLOE_CT_PATTERN='yoloe_humble_orchestration|two_realsense\.launch\.py|realsense2_camera_node'

# 固定 YOLOE 日志路径 —— start_lgw_bt.sh 靠这个读 Y3.5 verification
readonly YOLOE_LOG_HOST="${WS_FORK_HOST}/.run/lgw_yoloe_current.log"
readonly YOLOE_LOG_CT="${WS_FORK_CT}/.run/lgw_yoloe_current.log"

# ---- YOLOE 配置 ----
HEAD_CAMERA_SERIAL="${HEAD_CAMERA_SERIAL:-_261622078621}"
ARM_CAMERA_SERIAL="${ARM_CAMERA_SERIAL:-_262422071246}"
YOLOE_DEPLOY="${YOLOE_DEPLOY:-$HOME/yoloe_deploy}"
HUMBLE_YOLOE_INSTALL="${HUMBLE_YOLOE_INSTALL:-/opt/yoloe_transfer/humble_ws/install/local_setup.bash}"
ROS_DOMAIN_ID_YOLOE="${ROS_DOMAIN_ID_YOLOE:-0}"
RMW_IMPLEMENTATION_YOLOE="${RMW_IMPLEMENTATION_YOLOE:-rmw_cyclonedds_cpp}"
# --lo 开关:等价于把下面两个变量一起设好。必须在计算 DDS_ENV_ARGS 之前解析。
#   --lo            → config/dds_local_lo.xml + domain 7
#   --lo-domain=N   → 同上配置,domain 用 N(必须和 perception_bridge.yaml 的
#                     from_domain 一致,否则主栈收不到结果)
for arg in "$@"; do
  case "$arg" in
    --lo|lo)
      CYCLONEDDS_URI_YOLOE="${WS_FORK_HOST}/config/dds_local_lo.xml"
      ROS_DOMAIN_ID_YOLOE=7
      ;;
    --lo-domain=*)
      CYCLONEDDS_URI_YOLOE="${WS_FORK_HOST}/config/dds_local_lo.xml"
      ROS_DOMAIN_ID_YOLOE="${arg#*=}"
      ;;
  esac
done
# 可选:把 cameras(容器) + YOLOE(宿主) 整体切到另一套 CycloneDDS 配置
# (例如 lo-only + 大 FragmentSize + 大 socket 缓冲)。留空 = 沿用现有环境。
CYCLONEDDS_URI_YOLOE="${CYCLONEDDS_URI_YOLOE:-}"
DDS_ENV_ARGS=(-e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID_YOLOE}"
              -e "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION_YOLOE}")
DDS_URI_CT="${CYCLONEDDS_URI_YOLOE}"
if [ -n "${CYCLONEDDS_URI_YOLOE}" ]; then
  export CYCLONEDDS_URI="${CYCLONEDDS_URI_YOLOE}"
  # 宿主路径 → 容器路径,同一个文件两边都能读(bind mount)。
  DDS_URI_CT="${CYCLONEDDS_URI_YOLOE//${WS_FORK_HOST}/${WS_FORK_CT}}"
  DDS_ENV_ARGS+=(-e "CYCLONEDDS_URI=${DDS_URI_CT}")
fi
# 本脚本自己的 ros2 探测必须和 cameras/YOLOE 在同一个 domain,否则
# "YOLOE 未在 60s 内注册" 是假警报。
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_YOLOE}"
HEAD_USB_PORT="${HEAD_USB_PORT:-2-1}"
ARM_USB_PORT="${ARM_USB_PORT:-1-3.3}"
OVERLAY_MIN_PERIOD_SEC="${OVERLAY_MIN_PERIOD_SEC:-999.0}"
export OVERLAY_MIN_PERIOD_SEC
export YOLOE_DEPLOY

# ---- CLI args override (precedence: CLI > env > default) ----
# 用法:bash start_lgw_yoloe.sh overlay_min_period_sec=0.1 head_camera_serial=_xxx
for arg in "$@"; do
    case "$arg" in
        overlay_min_period_sec=*) OVERLAY_MIN_PERIOD_SEC="${arg#*=}" ;;
        transfer_root=*)          CLI_TRANSFER_ROOT="${arg#*=}" ;;
        head_camera_serial=*)     HEAD_CAMERA_SERIAL="${arg#*=}" ;;
        arm_camera_serial=*)      ARM_CAMERA_SERIAL="${arg#*=}" ;;
        ros_domain_id=*)          ROS_DOMAIN_ID_YOLOE="${arg#*=}" ;;
        --lo|lo|--lo-domain=*)    ;;   # 已在变量区提前解析
        -h|--help)
            cat <<'USAGE'
Usage: bash start_lgw_yoloe.sh [--lo] [key=value ...]
  --lo                          cameras + YOLOE 整体切到 lo 接口 + domain 7
                                (= CYCLONEDDS_URI_YOLOE=config/dds_local_lo.xml
                                   ROS_DOMAIN_ID_YOLOE=7)
  --lo-domain=<int>             同上,但 domain 用指定值(需与
                                config/perception_bridge.yaml 的 from_domain 一致)
  overlay_min_period_sec=<sec>  默认 999.0 (生产) / 0.1 (调试 visualizer)
  transfer_root=<path>          默认 $HOME/yoloe_deploy
  head_camera_serial=<serial>   默认 _261622078621
  arm_camera_serial=<serial>    默认 _262422071246
  ros_domain_id=<int>           默认 0
也支持 env var:OVERLAY_MIN_PERIOD_SEC=0.1 bash start_lgw_yoloe.sh
USAGE
            exit 0
            ;;
        *)
            echo "[yoloe] WARN: unknown CLI arg '$arg' (忽略)" >&2
            ;;
    esac
done
# CLI 覆盖 transfer_root
[ -n "${CLI_TRANSFER_ROOT:-}" ] && YOLOE_DEPLOY="${CLI_TRANSFER_ROOT}"

HOST_PIDS=()

# ---- Ctrl-C 清理 ----
cleanup_on_signal() {
  if [ "${CLEANUP_DONE:-0}" = "1" ]; then return; fi
  CLEANUP_DONE=1
  echo ""
  echo "[yoloe-cleanup] Ctrl-C / 信号触发,清理 YOLOE + 容器内 cameras..."

  # Host 端
  if [ ${#HOST_PIDS[@]} -gt 0 ]; then
    echo "[yoloe-cleanup] host pids: ${HOST_PIDS[*]}"
    for pid in "${HOST_PIDS[@]}"; do
      if [ -n "${pid}" ] && kill -0 "${pid}" 2>/dev/null; then
        kill -TERM "${pid}" 2>/dev/null || true
      fi
    done
    sleep 2
    for pid in "${HOST_PIDS[@]}"; do
      if [ -n "${pid}" ] && kill -0 "${pid}" 2>/dev/null; then
        kill -KILL "${pid}" 2>/dev/null || true
      fi
    done
  fi

  # Host orphan dual_camera_node (PPID=1)
  local yoloe_orphans
  yoloe_orphans=$(ps -eo pid,ppid,args 2>/dev/null \
    | awk '$2 == 1' \
    | grep -E 'dual_camera_node|yoloe_perception' \
    | grep -v grep \
    | awk '{print $1}') || yoloe_orphans=""
  if [ -n "${yoloe_orphans}" ]; then
    echo "[yoloe-cleanup] host orphan YOLOE (PPID=1): ${yoloe_orphans}"
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

  # 容器内 cameras
  local pids
  pids=$(docker exec "${CONTAINER}" bash -c "
    ps -eo pid,args 2>/dev/null | \
    grep -E '${LG_YOLOE_CT_PATTERN}' | \
    grep -v grep | \
    awk '{print \$1}'
  " 2>/dev/null) || pids=""
  if [ -n "${pids}" ]; then
    echo "[yoloe-cleanup] container camera pids: ${pids}"
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

  # 容器内 orphan realsense2_camera_node (PPID=1)
  local orphans
  orphans=$(docker exec "${CONTAINER}" bash -c "
    ps -eo pid,ppid,args 2>/dev/null | \
    awk '\$2 == 1' | grep 'realsense2_camera_node' | awk '{print \$1}'
  " 2>/dev/null) || orphans=""
  if [ -n "${orphans}" ]; then
    echo "[yoloe-cleanup] orphan realsense2_camera_node (PPID=1): ${orphans}"
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

  echo "[yoloe-cleanup] done"
}

# ---- preflight ----
preflight_cleanup() {
  echo "[yoloe-preflight] 清理 host YOLOE + 容器 cameras 残留..."

  local pids
  pids=$(ps -eo pid,ppid,args 2>/dev/null \
    | grep -E 'ros2 launch yoloe_perception|dual_camera_node' \
    | grep -v grep \
    | awk '{print $1}') || pids=""
  if [ -n "${pids}" ]; then
    echo "[yoloe-preflight] host YOLOE candidates: ${pids}"
    for pid in ${pids}; do
      kill -TERM "${pid}" 2>/dev/null || true
    done
    sleep 3
    for pid in ${pids}; do
      if kill -0 "${pid}" 2>/dev/null; then
        kill -KILL "${pid}" 2>/dev/null || true
      fi
    done
  fi

  pids=$(docker exec "${CONTAINER}" bash -c "
    ps -eo pid,args 2>/dev/null | \
    grep -E '${LG_YOLOE_CT_PATTERN}' | \
    grep -v grep | awk '{print \$1}'
  " 2>/dev/null) || pids=""
  if [ -n "${pids}" ]; then
    echo "[yoloe-preflight] container camera candidates: ${pids}"
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

  pids=$(docker exec "${CONTAINER}" bash -c "
    ps -eo pid,ppid,args 2>/dev/null | \
    awk '\$2 == 1' | grep 'realsense2_camera_node' | awk '{print \$1}'
  " 2>/dev/null) || pids=""
  if [ -n "${pids}" ]; then
    echo "[yoloe-preflight] orphan realsense2_camera_node (PPID=1): ${pids}"
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
  echo "[yoloe-preflight] done"
}

trap cleanup_on_signal INT TERM
preflight_cleanup

docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$" \
  || docker start "${CONTAINER}" >/dev/null

export LGW_INSTALL="${WS_FORK_CT}/install/setup.bash"
echo "[yoloe] LGW_INSTALL=${LGW_INSTALL}"

docker exec "${CONTAINER}" mkdir -p "${WS_FORK_CT}/.run"
docker exec "${CONTAINER}" chmod 777 "${WS_FORK_CT}/.run" 2>/dev/null || true

# 换了一套 CycloneDDS 配置(另一套接口/domain)时,残留的 ros2 daemon 会用旧
# 配置回答 `ros2 node list`,表现为 "YOLOE 未在 60s 内注册" 的假警报。
# 下面所有探测都走 --no-daemon,直接发现,不依赖 daemon 的启动环境。

# 自定义 DDS 配置(通常是 lo-only)的两个运行时前提,重启会丢,每次启动重设。
# 容器是 host 网络,所以在容器里设等于设宿主机。
if [ -n "${CYCLONEDDS_URI_YOLOE}" ]; then
  echo "[yoloe]   准备 lo 前提条件(multicast 标志 + socket 缓冲上限)"
  docker exec "${CONTAINER}" ip link set lo multicast on 2>/dev/null \
    || echo "[yoloe]   WARN: lo multicast on 失败(lo-only 配置可能发现不到对端)"
  docker exec "${CONTAINER}" sysctl -w net.core.wmem_max=67108864 >/dev/null 2>&1 \
    || echo "[yoloe]   WARN: net.core.wmem_max 提高失败(CycloneDDS socket 缓冲会受限)"
fi

# ====================================================================
# Phase Y1: YOLOE on host
# ====================================================================
echo "[yoloe] Phase Y1: start YOLOE Foxy on host..."
nohup bash -c "
  set +u
  set -a
  source ${YOLOE_DEPLOY}/config/foxy_humble_dds.env
  set +a
  export ROS_DOMAIN_ID='${ROS_DOMAIN_ID_YOLOE}'
  source /opt/ros/foxy/setup.bash
  source ${YOLOE_DEPLOY}/ros2_ws/install/local_setup.bash
  export LD_PRELOAD=/lib/aarch64-linux-gnu/libgomp.so.1
  export PYUNBUFFERED=1
  ros2 launch yoloe_perception dual_camera_foxy.launch.py \
      transfer_root:=\"\${YOLOE_DEPLOY}\" \
      overlay_min_period_sec:="\${OVERLAY_MIN_PERIOD_SEC}"
" > "${YOLOE_LOG_HOST}" 2>&1 &
YOLOE_PID=$!
HOST_PIDS+=("${YOLOE_PID}")
echo "[yoloe]   YOLOE host PID=${YOLOE_PID}, log → ${YOLOE_LOG_HOST}"

yoloe_ready=0
for attempt in $(seq 1 30); do
  sleep 2
  if bash -c "
    source ${YOLOE_DEPLOY}/ros2_ws/install/local_setup.bash >/dev/null 2>&1
    ros2 node list --no-daemon 2>/dev/null | grep -q yoloe_dual_camera_perception
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

# ====================================================================
# Phase Y1.5
# ====================================================================
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

# ====================================================================
# Phase Y1.6: USB force reset
# ====================================================================
echo "[yoloe] Phase Y1.6: USB force reset..."
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

# ====================================================================
# Phase Y2: cameras in container
# ====================================================================
echo "[yoloe] Phase Y2: start cameras in container..."
CAMERAS_LOG_HOST="${WS_FORK_HOST}/.run/lgw_yoloe_cameras.log"
docker exec -e LGW_INSTALL="${LGW_INSTALL}" "${DDS_ENV_ARGS[@]}" "${CONTAINER}" bash -c "
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
  nc=$(docker exec -e LGW_INSTALL="${LGW_INSTALL}" "${DDS_ENV_ARGS[@]}" "${CONTAINER}" bash -c "
    source /opt/ros/humble/setup.bash >/dev/null 2>&1
    source ${HUMBLE_YOLOE_INSTALL} >/dev/null 2>&1
    ros2 node list --no-daemon 2>/dev/null \
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

# ====================================================================
# Phase Y2.5: wait for first CONTINUOUS_RESULT
# ====================================================================
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

# ---- 提示 ----
echo "[yoloe] YOLOE bringup 完成 —— host Foxy + 容器内 realsense + 推理流已就绪。"
echo "        保持本脚本运行(不要 Ctrl-C),start_lgw_bt.sh 才会正常订阅到"
echo "        /perception/two_stage/camera_{head,arm}/result。"
echo ""
echo "  日志:"
echo "    YOLOE Foxy inference: ${YOLOE_LOG_HOST}"
echo "    容器内 cameras:       /tmp/two_realsense_inside.log (容器内)"
echo "                          ${CAMERAS_LOG_HOST} (host 端 launch 句柄)"
echo ""
echo "=== 等 Ctrl-C ==="
sleep infinity
