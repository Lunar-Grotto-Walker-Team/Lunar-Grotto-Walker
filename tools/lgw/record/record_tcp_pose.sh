#!/usr/bin/env bash
# scripts/record_tcp_pose.sh
#
# 把当前机械臂 TCP 位姿记录成 BT `SetPoseStamped` 的字面量:
#     frame_id;x;y;z;qx;qy;qz;qw
#
# /feedback/tcp_pose 是驱动 200 Hz 发布的真机 tcp 位姿(setGoal 目标不是),
# 已包含 launch 的 tcp_offset [0,0,0.12]。frame 用 `world`
# (== arm1/base_link, world→base_link 是 xyz=0/rpy=0 的 fixed joint)。
#
# 用法:
#   bash scripts/record_tcp_pose.sh                    # 只打印
#   bash scripts/record_tcp_pose.sh above_handle       # 打印并追加到 waypoints 文件
#
# 追加文件: scripts/tcp_waypoints_<date>.txt,每行 "名字: 字面量"
set -u

NAME="${1:-}"
CONTAINER="${CONTAINER:-self_l4t}"

RAW="$(docker exec "${CONTAINER}" bash -c '
  source /my_ws/l4t/.l4t_env.sh >/dev/null 2>&1
  timeout 10 ros2 topic echo /feedback/tcp_pose --once 2>/dev/null
')"

LITERAL="$(printf '%s' "${RAW}" | python3 -c '
import re, sys
raw = sys.stdin.read()
if "position:" not in raw:
    sys.exit(1)
pos_block, rest = raw.split("position:", 1)[1].split("orientation:", 1)
ori_block = rest

def field(block, key):
    m = re.search(r"^\s*" + key + r":\s*(-?[0-9.eE+-]+)", block, re.M)
    if not m:
        raise SystemExit(1)
    return float(m.group(1))

x, y, z = (field(pos_block, k) for k in ("x", "y", "z"))
qx, qy, qz, qw = (field(ori_block, k) for k in ("x", "y", "z", "w"))

# 归一化到 6 位小数,和现有 bt_trees 里的写法一致
print("world;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f" % (x, y, z, qx, qy, qz, qw))
')" || { echo "[record] ERROR: 读不到 /feedback/tcp_pose" >&2; exit 1; }

if [ -n "${NAME}" ]; then
  OUT="$(dirname "$0")/tcp_waypoints_$(date +%Y%m%d).txt"
  printf '%-16s %s\n' "${NAME}:" "${LITERAL}" >> "${OUT}"
  printf '[record] %-16s %s  (→ %s)\n' "${NAME}:" "${LITERAL}" "${OUT}"
else
  printf '%s\n' "${LITERAL}"
fi
