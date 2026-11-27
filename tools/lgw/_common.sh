#!/usr/bin/env bash
# tools/lgw/_common.sh — 行为树启动脚本的公共路径。
#
# 所有 start_*.sh / run_bt.sh 都 source 本文件，不再各自写死工作区路径。
# 需要覆盖时用环境变量（仓库不在默认位置、或容器挂载点不同）：
#
#   WS_FORK_CT=/my_ws/l4t/Lunar-Grotto-Walker bash tools/lgw/start/start_lgw_chain.sh
#   HOST_MOUNT=/home/<user>/my_ws CT_MOUNT=/my_ws bash tools/lgw/start/run_bt.sh
#
# 注意：l4t_setup.sh / .l4t_env.sh 属于容器环境本身（不在本仓库里），
# 仍按固定路径 /my_ws/l4t/... 引用，需要时用 L4T_ENV_CT 覆盖。

# 仓库根（宿主机）：本文件在 tools/lgw/ 下
LGW_ROOT_HOST="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

# 宿主机 ↔ 容器的挂载根（run_bt.sh 用它把自身路径翻译进容器）
HOST_MOUNT="${HOST_MOUNT:-/home/unitree/my_ws}"
CT_MOUNT="${CT_MOUNT:-/my_ws}"

# 本仓库在宿主机 / 容器内的路径
WS_FORK_HOST="${WS_FORK_HOST:-$LGW_ROOT_HOST}"
if [ -n "${WS_FORK_CT:-}" ]; then
  :
elif [[ "$WS_FORK_HOST" == "$HOST_MOUNT"* ]]; then
  WS_FORK_CT="${WS_FORK_HOST/#$HOST_MOUNT/$CT_MOUNT}"
else
  # 仓库不在默认挂载根下：按容器里的约定位置回退，可用 WS_FORK_CT 覆盖
  WS_FORK_CT="${CT_MOUNT}/l4t/Lunar-Grotto-Walker"
fi

CONTAINER="${CONTAINER:-self_l4t}"

# vendor 仓库（第三方驱动与基础库），默认与本仓库同级
VENDOR_ROOT_HOST="${VENDOR_ROOT_HOST:-$(cd "$LGW_ROOT_HOST/.." && pwd)/Lunar-Grotto-Walker-Vendor}"
if [ -n "${VENDOR_ROOT_CT:-}" ]; then
  :
elif [[ "$VENDOR_ROOT_HOST" == "$HOST_MOUNT"* ]]; then
  VENDOR_ROOT_CT="${VENDOR_ROOT_HOST/#$HOST_MOUNT/$CT_MOUNT}"
else
  VENDOR_ROOT_CT="${CT_MOUNT}/l4t/Lunar-Grotto-Walker-Vendor"
fi
# can_activate.sh 随 vendor 的 agx_arm_ros 源码提供（不安装到 install/）
AGX_CAN_ACTIVATE_CT="${AGX_CAN_ACTIVATE_CT:-${VENDOR_ROOT_CT}/src/agx_arm_ros/scripts/can_activate.sh}"

L4T_ENV_CT="${L4T_ENV_CT:-${CT_MOUNT}/l4t/.l4t_env.sh}"

export LGW_ROOT_HOST WS_FORK_HOST WS_FORK_CT CONTAINER HOST_MOUNT CT_MOUNT \
       VENDOR_ROOT_HOST VENDOR_ROOT_CT AGX_CAN_ACTIVATE_CT L4T_ENV_CT
