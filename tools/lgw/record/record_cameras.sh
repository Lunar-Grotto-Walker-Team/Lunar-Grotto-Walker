#!/usr/bin/env bash
# scripts/record_cameras.sh — 用 rosbag2 录制 head / arm 两个 D435 的
# 彩色图 + 对齐深度图(2026-11-20)。
#
# 用法:
#   bash scripts/record_cameras.sh                 # 默认录到 Ctrl-C
#   bash scripts/record_cameras.sh -d 30           # 录 30 秒自动停
#   bash scripts/record_cameras.sh --lo            # 相机在 lo + domain 7 时
#   bash scripts/record_cameras.sh --lo --raw-depth --info
#   bash scripts/record_cameras.sh -o /home/unitree/my_ws/bags/test1
#
# 和 start_lgw_yoloe.sh 的 --lo 开关对应:
#   --lo            cameras 跑在 lo 网卡 + domain 7(dds_local_lo.xml)
#   --lo-domain=N   同上, domain 用 N
#   不加 = domain 0 + eth10(默认栈)
#
# 录制的话题(默认):
#   /camera_head/camera_head/color/image_raw                    1280x720x15
#   /camera_head/camera_head/aligned_depth_to_color/image_raw    848x480x15
#   /camera_arm/camera_arm/color/image_raw                       1280x720x6
#   /camera_arm/camera_arm/aligned_depth_to_color/image_raw      848x480x6
# 可选:
#   --raw-depth  额外录 depth/image_rect_raw(未对齐到彩色的原始深度)
#   --info       额外录 camera_info + /tf_static(回放时要重建 TF 树就得带上)
#
# ⚠ 体积: 原始图每秒约 40~60 MB(head 720p 彩色就占 ~41 MB/s),
#   一 分钟 2.5~3.6 GB。脚本默认开 zstd 文件压缩(--no-compress 关掉),
#   实测对图像大概 1.5~2.5x,但会吃 CPU(Jetson 上 2 线程)。长时间录制建议:
#     * 先 `--no-compress` 看磁盘够不够,
#     * 或者录完用 `ros2 bag convert` 离线压。
#
# 前置: 相机节点在跑(`bash scripts/start_lgw_yoloe.sh [--lo]`)。
# 本脚本只订阅, 不改任何东西, 不影响正在跑的行为树。
#
# 容器/宿主机: 两边都能直接跑; 宿主机调用会自动转进容器(相机发布在容器里)。
# 录制目录默认 /my_ws/l4t/lg_walker_ws/bags/cameras_<时间戳>,
# 宿主机对应 /home/unitree/my_ws/l4t/lg_walker_ws/bags/...
set -uo pipefail

CONTAINER="${CONTAINER:-self_l4t}"
HOST_WS="/home/unitree/my_ws"
CT_WS="/my_ws"
CT_LGW="${CT_WS}/l4t/lg_walker_ws"
HOST_LGW="${HOST_WS}/l4t/lg_walker_ws"

# ---------------------------------------------------------------
# 宿主机调用 -> 原样转进容器执行
# ---------------------------------------------------------------
if [ ! -f /my_ws/l4t/.l4t_env.sh ]; then
  self_path="$(readlink -f "$0")"
  case "$self_path" in
    "$HOST_WS"/*) ct_path="${CT_WS}${self_path#"$HOST_WS"}" ;;
    *) echo "[rec] 既不在容器内、也不在 ${HOST_WS} 下, 无法定位: $self_path" >&2; exit 1 ;;
  esac
  tty_flags="-i"
  if [ -t 0 ] && [ -t 1 ]; then tty_flags="-it"; fi
  echo "[rec] 宿主机调用 -> docker exec ${CONTAINER} ${ct_path} $*"
  exec docker exec ${tty_flags} "${CONTAINER}" "${ct_path}" "$@"
fi

# ---------------------------------------------------------------
# 参数
# ---------------------------------------------------------------
DURATION=0
OUT=""
RAW_DEPTH=0
WITH_INFO=0
COMPRESS=1
ROS_DOMAIN_ID_CAM="${ROS_DOMAIN_ID:-0}"
CYCLONEDDS_URI_CAM="${CYCLONEDDS_URI:-}"

while [ $# -gt 0 ]; do
  case "$1" in
    -d|--duration)   DURATION="${2:-0}"; shift 2 ;;
    -o|--out)        OUT="${2:-}"; shift 2 ;;
    --raw-depth)     RAW_DEPTH=1; shift ;;
    --info)          WITH_INFO=1; shift ;;
    --no-compress)   COMPRESS=0; shift ;;
    --lo)            CYCLONEDDS_URI_CAM="${CT_LGW}/config/dds_local_lo.xml"; ROS_DOMAIN_ID_CAM=7; shift ;;
    --lo-domain=*)   CYCLONEDDS_URI_CAM="${CT_LGW}/config/dds_local_lo.xml"; ROS_DOMAIN_ID_CAM="${1#*=}"; shift ;;
    -h|--help)       sed -n '2,45p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "[rec] 未知参数: $1 (--help 看用法)" >&2; exit 2 ;;
  esac
done

# ---------------------------------------------------------------
# 环境
# ---------------------------------------------------------------
source /my_ws/l4t/.l4t_env.sh
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_CAM}"
if [ -n "${CYCLONEDDS_URI_CAM}" ]; then
  export CYCLONEDDS_URI="${CYCLONEDDS_URI_CAM}"
fi
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export RMW_IMPLEMENTATION

if [ -n "${CYCLONEDDS_URI_CAM}" ]; then
  echo "[rec] domain=${ROS_DOMAIN_ID} rmw=${RMW_IMPLEMENTATION} iface=lo(${CYCLONEDDS_URI_CAM##*/})"
else
  echo "[rec] domain=${ROS_DOMAIN_ID} rmw=${RMW_IMPLEMENTATION} iface=eth10(默认)"
fi

# ---------------------------------------------------------------
# 话题清单
# ---------------------------------------------------------------
TOPICS=(
  /camera_head/camera_head/color/image_raw
  /camera_head/camera_head/aligned_depth_to_color/image_raw
  /camera_arm/camera_arm/color/image_raw
  /camera_arm/camera_arm/aligned_depth_to_color/image_raw
)
if [ "${RAW_DEPTH}" = "1" ]; then
  TOPICS+=(
    /camera_head/camera_head/depth/image_rect_raw
    /camera_arm/camera_arm/depth/image_rect_raw
  )
fi
if [ "${WITH_INFO}" = "1" ]; then
  TOPICS+=(
    /camera_head/camera_head/color/camera_info
    /camera_head/camera_head/aligned_depth_to_color/camera_info
    /camera_arm/camera_arm/color/camera_info
    /camera_arm/camera_arm/aligned_depth_to_color/camera_info
    /tf_static
  )
fi

# 只录当前真有 publisher 的话题: rosbag2 对"零 publisher"的话题会直接报错退出。
LIVE_TOPICS=()
for t in "${TOPICS[@]}"; do
  n=$(timeout 8 ros2 topic info "$t" 2>/dev/null | awk -F': *' '/Publisher count/{print $2; exit}')
  if [ -n "${n:-}" ] && [ "${n:-0}" -gt 0 ]; then
    LIVE_TOPICS+=("$t")
    echo "[rec]   + $t (pub=$n)"
  else
    echo "[rec]   - $t (无 publisher, 跳过) -- 相机起了吗?"
  fi
done

if [ ${#LIVE_TOPICS[@]} -eq 0 ]; then
  echo "[rec] 没有任何可录的相机话题, 退出。先跑: bash scripts/start_lgw_yoloe.sh [--lo]" >&2
  exit 1
fi

# ---------------------------------------------------------------
# QoS 覆盖: 相机用 SensorDataQoS(BEST_EFFORT), rosbag2 默认 RELIABLE 订阅
# 会收不到(rclcpp 的 QoS incompatibility 在 rmw 层是静默的)。
# best_effort 的请求方对 reliable 的发布方也是兼容的, 所以对所有话题统一
# 覆盖成 best_effort 是安全的。
# ---------------------------------------------------------------
QOS_YAML="$(mktemp /tmp/lgw_bag_qos.XXXXXX.yaml)"
{
  echo "# auto-generated by record_cameras.sh"
  for t in "${LIVE_TOPICS[@]}"; do
    echo "${t}:"
    echo "  reliability: best_effort"
    echo "  durability: volatile"
    echo "  history: keep_last"
    echo "  depth: 10"
  done
} > "${QOS_YAML}"
trap 'rm -f "${QOS_YAML}"' EXIT

# ---------------------------------------------------------------
# 输出目录
# ---------------------------------------------------------------
if [ -z "${OUT}" ]; then
  OUT="${CT_LGW}/bags/cameras_$(date +%Y%m%d_%H%M%S)"
fi
mkdir -p "$(dirname "${OUT}")"
if [ -e "${OUT}" ]; then
  echo "[rec] 输出路径已存在: ${OUT}" >&2; exit 1
fi

# ---------------------------------------------------------------
# 录制
# ---------------------------------------------------------------
REC_ARGS=(-o "${OUT}" --qos-profile-overrides-path "${QOS_YAML}")
if [ "${COMPRESS}" = "1" ]; then
  REC_ARGS+=(--compression-mode file --compression-format zstd
             --compression-queue-size 4 --compression-threads 2)
fi

# stdin 必须给 /dev/null: rosbag2 有一个键盘处理线程在读 stdin
# ("Press SPACE for pausing")。在有 tty 的 docker exec -it 会话里它会去
# 读终端, 一旦进程组不是前台就收到 SIGTTIN -> 进程被停止(T 状态),
# 而且停在 "Open database" 之前 —— 现象就是"卡住、bag 目录都没建"
# (2026-11-20 实测)。喂 /dev/null 后 rosbag2 自己打印
# "keyboard handling disabled" 并走纯录制路径。
#
# timeout 同时给 -k: 30 s 录像收尾(flush 缓存)可能超过 SIGINT 之后的一瞬,
# 超时仍不退才 SIGKILL, 避免又卡死。
if [ "${DURATION}" -gt 0 ] 2>/dev/null; then
  echo "[rec] 录制 ${DURATION}s -> ${OUT}"
  timeout -k 30 -s INT "${DURATION}" \
    ros2 bag record "${REC_ARGS[@]}" "${LIVE_TOPICS[@]}" < /dev/null
  rc=$?
  [ "${rc}" = "124" ] && rc=0     # timeout 正常结束
else
  echo "[rec] 录制中, Ctrl-C 停止 -> ${OUT}"
  ros2 bag record "${REC_ARGS[@]}" "${LIVE_TOPICS[@]}" < /dev/null
  rc=$?
fi

# ---------------------------------------------------------------
# 结果
# ---------------------------------------------------------------
if [ -d "${OUT}" ]; then
  size=$(du -sh "${OUT}" 2>/dev/null | cut -f1)
  host_out="${OUT/${CT_WS}/${HOST_WS}}"
  echo "[rec] 完成: ${size} -> ${OUT}"
  echo "[rec] 宿主机: ${host_out}"
  echo "[rec] 回放:   ros2 bag play ${OUT} [--clock]"
  echo "[rec] 看信息: ros2 bag info ${OUT}"
else
  echo "[rec] 没有产生 bag 目录(rc=${rc})" >&2
fi
exit "${rc}"
