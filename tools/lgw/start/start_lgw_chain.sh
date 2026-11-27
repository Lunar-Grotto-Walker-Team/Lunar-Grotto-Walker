#!/bin/bash

# shellcheck source=../_common.sh
source "$(dirname "${BASH_SOURCE[0]}")/../_common.sh"
# scripts/start_lgw_chain.sh
#
# 当前扫描链路的统一启动入口(2026-11-15)。宿主机脚本:容器内起 launch,
# 本脚本只做「docker + 环境 + 日志 + 清理」。
#
#   lgw_chain.launch.py = lgw_full(MoveIt/Servo/lgw servo/可选 BT) + 3D tracker
#
# 前置(不在本脚本内):
#   1) 机械臂驱动 + CAN —— 宿主机另开终端:
#        bash scripts/start_lgw_arm.sh --driver-only
#   2) 两路 realsense —— 在 yoloe_humble_orchestration 的 install 空间里:
#        source /opt/ros/humble/setup.bash
#        source /my_ws/l4t/ros2_ws/install/local_setup.bash
#        source /opt/yoloe_transfer/humble_ws/install/local_setup.bash
#        ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
#        ros2 launch yoloe_humble_orchestration two_realsense.launch.py \
#          head_serial:=_261622078621 arm_serial:=_262422071246
#   3) YOLOE —— 宿主机 foxy(scripts/start_lgw_yoloe.sh)
#
#   ＊ cameras/YOLOE 若跑在 domain 7(lo),本脚本会自动起一个跨域桥把结果
#     搬回 domain 0(scripts/start_lgw_perception_bridge.sh)。不需要时设
#     PERCEPTION_BRIDGE=0 关掉。
#
# 用法:
#   bash scripts/start_lgw_chain.sh [launch 参数...]
#   bash scripts/start_lgw_chain.sh use_bt:=true follow:=true
#
#   use_bt:=true 会一并起 go2_transit/transit_node(TransitToLocalPoint 的
#   动作服务器, V2 系列树必需)。若别处已经起了 transit_node, 传
#   use_transit:=false 关掉本链这一份 —— 同名两个节点会让 goal 随机落一个。
#
# 终止: Ctrl-C → 按 PID 清理容器内本链路的节点(不碰 driver / 相机)。
set -u
CONTAINER="${CONTAINER:-self_l4t}"
WS="${WS_FORK_CT}"
LOG_TS="$(date +%Y%m%d_%H%M%S)"
LOG_HOST="/tmp/lgw_chain_${LOG_TS}.log"

case "${1:-}" in
  -h|--help)
    sed -n '2,29p' "$0" | sed 's/^# \{0,1\}//'
    exit 0
    ;;
esac

if ! docker ps --format '{{.Names}}' | grep -qx "${CONTAINER}"; then
  echo "[chain] 容器 ${CONTAINER} 未运行" >&2
  exit 1
fi

if ! docker exec "${CONTAINER}" bash -c "ps -eo args | grep -q '[a]gx_arm_ctrl_single'"; then
  echo "[chain] 警告: agx_arm_ctrl 未运行,机械臂不会动。"
  echo "[chain]       另开终端执行: bash scripts/start_lgw_arm.sh --driver-only"
fi

# ---- 感知结果跨域桥:domain 7(lo) → domain 0 ----
# cameras/YOLOE 跑在 lo 上时,主栈(domain 0)看不到 /perception/two_stage/*/result。
# 桥只搬三条 KB 级消息,图像和点云仍旧留在 domain 7。
# PERCEPTION_BRIDGE=0 可关闭(例如把 YOLOE 直接跑回 domain 0 时)。
PERCEPTION_BRIDGE="${PERCEPTION_BRIDGE:-1}"
BRIDGE_STARTED=0
if [ "${PERCEPTION_BRIDGE}" != "0" ]; then
  if bash "$(dirname "$0")/start_lgw_perception_bridge.sh"; then
    BRIDGE_STARTED=1
  else
    echo "[chain] 警告: 感知桥启动失败,主栈将收不到检测结果" >&2
  fi
fi

cleanup() {
  echo
  echo "[chain] 清理容器内本链路节点(driver / 相机不动)..."
  docker exec -w "${WS}" "${CONTAINER}" bash tools/lgw/bench/bench_cleanup.sh >/dev/null 2>&1 || true
  if [ "${BRIDGE_STARTED}" = "1" ]; then
    bash "$(dirname "$0")/start_lgw_perception_bridge.sh" --stop >/dev/null 2>&1 || true
    echo "[chain] 感知桥已停止"
  fi
}
trap cleanup INT TERM

echo "[chain] log → ${LOG_HOST}"
# -it needs a real terminal; fall back to -i under nohup / CI.
TTY_FLAGS="-i"
if [ -t 0 ] && [ -t 1 ]; then
  TTY_FLAGS="-it"
fi
docker exec ${TTY_FLAGS} -w "${WS}" "${CONTAINER}" bash -lc "
  source /my_ws/l4t/.l4t_env.sh
  ros2 launch lgw_bt_planner lgw_chain.launch.py $* 2>&1 | tee '${LOG_HOST}'
"
