#!/usr/bin/env bash
# 感知结果跨域桥:domain 7(lo) → domain 0(eth10)
#
# 把 YOLOE 的检测结果搬回主栈所在的 domain 0,让伺服 / tracker / BT 能直接用。
# 只桥三条小消息,图像和点云留在 domain 7(那是 lo 存在的意义)。
#
#   bash scripts/start_lgw_perception_bridge.sh          # 启动
#   bash scripts/start_lgw_perception_bridge.sh --stop   # 停止
#   bash scripts/start_lgw_perception_bridge.sh --status # 查看
#
# 前置条件(由 start_lgw_yoloe.sh 在启用 lo 配置时设置,这里兜底一次):
#   ip link set lo multicast on   —— lo 默认没有 MULTICAST 标志
#
# 注意:bridge 进程的 CYCLONEDDS_URI 必须同时包含两个 domain 的段,
# 见 config/perception_bridge.xml。

set -u

CONTAINER="${CONTAINER:-self_l4t}"
# shellcheck source=../_common.sh
source "$(dirname "${BASH_SOURCE[0]}")/../_common.sh"
BRIDGE_XML="${WS_FORK_CT}/config/perception_bridge.xml"
BRIDGE_YAML="${WS_FORK_CT}/config/perception_bridge.yaml"
LOG_CT="/tmp/lgw_perception_bridge.log"

find_pid() {
  docker exec "${CONTAINER}" pgrep -f 'lib/domain_bridge/domain_bridge' 2>/dev/null | head -1
}

stop_bridge() {
  local pid
  pid="$(find_pid)"
  if [ -z "${pid}" ]; then
    echo "[bridge] 未在运行"
    return 0
  fi
  docker exec "${CONTAINER}" kill -15 "${pid}" 2>/dev/null
  for _ in $(seq 1 20); do
    sleep 0.5
    [ -z "$(find_pid)" ] && { echo "[bridge] 已停止 (pid=${pid})"; return 0; }
  done
  echo "[bridge] WARN: pid=${pid} 未在 10s 内退出" >&2
  return 1
}

case "${1:-start}" in
  --stop|stop)
    stop_bridge
    exit $?
    ;;
  --status|status)
    pid="$(find_pid)"
    if [ -z "${pid}" ]; then
      echo "[bridge] 未在运行"
      exit 1
    fi
    echo "[bridge] 运行中 pid=${pid}"
    docker exec "${CONTAINER}" bash -c \
      "source /my_ws/l4t/.l4t_env.sh >/dev/null 2>&1; \
       timeout 8 ros2 topic hz /perception/two_stage/camera_arm/result 2>&1 | tail -2"
    exit 0
    ;;
esac

# ---- 启动 ----
stop_bridge >/dev/null 2>&1

# 两个运行时前提(都是重启会丢的,这里兜底):
#   lo 的 MULTICAST 标志 —— lo-only 配置发现对端的前提
#   net.core.wmem_max   —— 配置里 SocketSendBufferSize 的 max 不能超过它
docker exec "${CONTAINER}" ip link set lo multicast on 2>/dev/null || true
docker exec "${CONTAINER}" sysctl -w net.core.wmem_max=67108864 >/dev/null 2>&1 || true

# 日志写在容器内:-d 脱离模式下宿主机这侧接不到 stdout
docker exec -d "${CONTAINER}" bash -c "
  source /opt/ros/humble/setup.bash
  source /opt/yoloe_transfer/humble_ws/install/local_setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=${BRIDGE_XML}
  exec ros2 run domain_bridge domain_bridge ${BRIDGE_YAML} > ${LOG_CT} 2>&1
"

for _ in $(seq 1 20); do
  sleep 0.5
  pid="$(find_pid)"
  [ -n "${pid}" ] && break
done

# 进程起来后可能立刻挂掉(配置/DDS 错误),隔一会儿再确认一次
if [ -n "${pid:-}" ]; then
  sleep 1.5
  if [ -z "$(find_pid)" ]; then
    echo "[bridge] ERROR: 启动后立即退出" >&2
    docker exec "${CONTAINER}" cat "${LOG_CT}" 2>/dev/null | tail -20 >&2
    exit 1
  fi
fi

if [ -z "${pid:-}" ]; then
  echo "[bridge] ERROR: 启动失败" >&2
  docker exec "${CONTAINER}" cat "${LOG_CT}" 2>/dev/null | tail -20 >&2
  exit 1
fi
echo "[bridge] 已启动 pid=${pid} (log: 容器内 ${LOG_CT})"
echo "[bridge] 桥接: camera_head/result, camera_arm/result, state  7 → 0"
