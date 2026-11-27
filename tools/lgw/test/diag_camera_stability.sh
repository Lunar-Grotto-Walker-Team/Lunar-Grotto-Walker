#!/bin/bash
# diag_camera_stability.sh — 2026-10-02
# 定位 head_color 实际 publish ~3.5 Hz (vs 目标 15 Hz) 的根因。
# 不 kill 任何进程,纯只读诊断。cameras 必须已经启动 (Phase 1 之后)。
#
# 前置:  self_l4t 容器内 realsense2_camera_node 已在跑(两相机节点可见)。
# 用法:  bash diag_camera_stability.sh

set -uo pipefail

ts() { date +'%H:%M:%S'; }
section() {
    echo
    echo "================================================================"
    echo "  [$(ts)] $1"
    echo "================================================================"
}

CONTAINER="${CONTAINER:-self_l4t}"
HEAD_SERIAL="${HEAD_CAMERA_SERIAL:-261622078621}"
ARM_SERIAL="${ARM_CAMERA_SERIAL:-262422071246}"

# ===== Section A: USB 拓扑 =====
section "A. USB topology (lsusb -t) — 看 head/arm 各在哪个 hub/port"
if command -v lsusb >/dev/null 2>&1; then
    lsusb -t 2>&1 | head -60
else
    echo "lsusb 不在 host PATH 中"
    docker exec "$CONTAINER" bash -c "command -v lsusb >/dev/null 2>&1 && lsusb -t || echo 'lsusb 也不在容器内'"
fi

# ===== Section B: RealSense 设备清单 =====
section "B. RealSense device enumeration"
if command -v rs-enumerate-devices >/dev/null 2>&1; then
    rs-enumerate-devices 2>&1
else
    echo "host 上无 rs-enumerate-devices,改用容器内 librealsense2-utils"
    docker exec "$CONTAINER" bash -c "
        if command -v rs-enumerate-devices >/dev/null 2>&1; then
            rs-enumerate-devices
        elif command -v realsense-viewer >/dev/null 2>&1; then
            echo '(只有 realsense-viewer,装 librealsense2-utils 后再跑)'
        else
            echo '(容器内也找不到 librealsense CLI)'
        fi"
fi

# ===== Section C: dmesg USB 事件 =====
section "C. dmesg 最近 USB/xhci/realsense 事件 (5 min 内)"
if [ "$(id -u)" -eq 0 ] || sudo -n true 2>/dev/null; then
    dmesg --since "-5 minutes" 2>/dev/null \
        | grep -iE 'usb|realsense|xhci|reset|error|disconnect|reconnect|stall|timeout' \
        | tail -50
else
    echo "需要 root,试 sudo"
    sudo -n dmesg --since "-5 minutes" 2>&1 \
        | grep -iE 'usb|realsense|xhci|reset|error|disconnect|reconnect|stall|timeout' \
        | tail -50
fi

# ===== Section D: 容器内 realsense 日志 =====
section "D. 容器内 realsense/camera 日志的 ERROR/WARN"
docker exec "$CONTAINER" bash -c "
    cd /tmp 2>/dev/null
    echo '  -- D.1 /tmp/*.log 列表 (最新修改的 10 个) --'
    ls -lt /tmp/*.log 2>/dev/null | head -10
    echo
    echo '  -- D.2 关键字过滤 (error|warn|endpoint|stall|reset|drop|timeout|reconnect) --'
    for log in /tmp/*.log; do
        [ -f \"\$log\" ] || continue
        size=\$(stat -c%s \"\$log\" 2>/dev/null || echo 0)
        [ \"\$size\" -gt 0 ] || continue
        echo '----'
        echo \"FILE: \$log (size=\${size}B)\"
        echo '----'
        grep -iE 'error|warn|endpoint|stall|reset|drop|timeout|reconnect|usb|realsense' \
            \"\$log\" 2>/dev/null | tail -25
    done" 2>&1 | head -200

# ===== Section E: live topic publisher / QoS =====
section "E. 4 路 image_raw 当前的 publisher 数 + QoS"
docker exec "$CONTAINER" bash -c "
    source /opt/ros/humble/setup.bash
    source /my_ws/l4t/ros2_ws/install/setup.bash 2>/dev/null
    for t in \
        /camera_head/camera_head/color/image_raw \
        /camera_head/camera_head/aligned_depth_to_color/image_raw \
        /camera_arm/camera_arm/color/image_raw \
        /camera_arm/camera_arm/aligned_depth_to_color/image_raw; do
        echo \"-- \$t --\"
        ros2 topic info -v \$t 2>/dev/null \
            | grep -E 'Publisher count|Reliability|History|Depth|Durability' \
            | sed 's/^/    /'
    done"

# ===== Section F: 物理 swap 操作建议 =====
section "F. 物理对调 (定位是相机问题还是端口问题)"
cat <<'EOF'
  这一步要动硬件,执行前先确认 A-E 结果:
    1. 记下当前两个相机各接在 USB 哪个物理口 (A 节)
    2. 关掉所有 YOLOE + cameras (手动 kill,不依赖脚本)
    3. 把两相机的 USB 线**互相对调**插到对方原来的端口
    4. 启动 cameras + YOLOE,再跑 `test_yoloe_stability.sh 30`
    5. 看症状跟相机走还是跟端口走:
        - 跟相机走 (同样的 serial 还是 missing): 相机本身问题
        - 跟端口走 (换端口后还是 head 那个物理口 missing): USB hub / 线缆
        - 都不缓解: realsync 驱动内部 pipeline 状态机问题
EOF

echo
echo "================================================================"
echo "  [$(ts)] diag done — copy 上面 A/B/C/D/E 输出贴回来"
echo "================================================================"