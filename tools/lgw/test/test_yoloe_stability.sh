#!/bin/bash
# test_yoloe_stability.sh — focused diagnostic for the YOLOE dual-camera
# inference pipeline (cameras + YOLOE perception, no bt_runner / BT).
#
# Use this to verify that the camera + YOLOE pipeline is healthy before
# running the full BT E2E (test_yoloe_e2e.sh). Symptom this catches:
# the color/depth stream dies a few seconds into YOLOE startup, leaving
# no CONFIRMED ObservationResult for evaluate_detection() to see.
#
# ==== 2026-10-04: YOLOE-first plan ====
# 顺序与 start_lgw_head_to_arm_lookat.sh / test_yoloe_e2e.sh 对齐:
#   Phase 0     prerequisites
#   Phase 1     host Foxy 起 YOLOE(先;避开模型加载 CPU 尖峰与 cameras USB init 撞车)
#   Phase 1.5   等 YOLOE log 出现 CONTINUOUS_DUAL_CAMERA_STARTED
#               (dual_camera_node.py:335 在 __init__ 末尾 emit,
#                不依赖 cameras 已起,纯 YOLOE 模型加载完成信号)
#   Phase 2     容器内起 cameras(YOLOE 已就绪,USB init 时 YOLOE 不再 spike)
#   Phase 2.5   等 head + arm 首个 CONTINUOUS_RESULT(pipeline-alive 即可)
#   Phase 3     持续监控 pub_count + topic hz,直到用户 Ctrl-C
#   终态        Ctrl-C 触发 cleanup():跑 Phase 4/5/6 总结 → 杀进程 → exit
#
# Usage:
#   bash test_yoloe_stability.sh             # 持续运行,Ctrl-C 停止(默认)
#   bash test_yoloe_stability.sh --check     # 只跑 Phase 0 prerequisites
#   bash test_yoloe_stability.sh 300         # 跑 300s 然后自动退出
#   SAMPLE_INTERVAL=10 bash test_yoloe_stability.sh   # 10s 采样间隔
#
# 另开终端监控:cat 同目录 MONITOR_COMMANDS.txt

set -uo pipefail

DURATION_SEC="${1:-0}"   # 0 = infinite (run until Ctrl-C)
SAMPLE_INTERVAL="${SAMPLE_INTERVAL:-5}"
CHECK_ONLY=0
if [ "$DURATION_SEC" = "--check" ]; then
    DURATION_SEC=0
    CHECK_ONLY=1
fi

HEAD_CAMERA_SERIAL="${HEAD_CAMERA_SERIAL:-_261622078621}"
ARM_CAMERA_SERIAL="${ARM_CAMERA_SERIAL:-_262422071246}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
YOLOE_DEPLOY="${YOLOE_DEPLOY:-$HOME/yoloe_deploy}"
CONTAINER="self_l4t"
WS_L4T="/my_ws/l4t"
HUMBLE_YOLOE_INSTALL="/opt/yoloe_transfer/humble_ws/install/local_setup.bash"
export YOLOE_DEPLOY  # inherited by nohup bash -c subshell

CONTAINER_PIDS=()
HOST_PIDS=()
LOG_FILES=()
YOLOE_LOG=""
CAMERAS_LOG=""
SAMPLES_DIR="/tmp/test_yoloe_stability_samples"
mkdir -p "$SAMPLES_DIR"
START_TS=$(date +%s)
YOLOE_PID=0
CAMERAS_IN_PID=0
CAM_CHILD_PIDS=()

ts() { date +'%H:%M:%S'; }
log() { echo "[$(ts)] $*"; }
err() { echo "[$(ts)] [ERROR] $*" >&2; }

# ===== Phase 4/5/6 summary functions (called from cleanup) =====
fps_from_log() {
    local cam="$1"
    if [ ! -f "$YOLOE_LOG" ]; then echo "0.00"; return; fi
    grep "CONTINUOUS_RESULT camera=$cam" "$YOLOE_LOG" 2>/dev/null \
        | grep -oE '\[[0-9]+\.[0-9]+\]' | tr -d '[]' \
        | awk '
            NR==1 { first=$1; last=$1; n=1; next }
            { last=$1; n++ }
            END {
                if (n<2 || last-first<=0) { print "0.00"; exit }
                printf "%.2f", (n-1)/(last-first)
            }
        '
}

msg_total() {
    if [ -f "$YOLOE_LOG" ]; then
        grep -c "CONTINUOUS_RESULT camera=$1" "$YOLOE_LOG" 2>/dev/null || echo 0
    else
        echo 0
    fi
}

status_count() {
    if [ -f "$YOLOE_LOG" ]; then
        grep "CONTINUOUS_RESULT camera=$1" "$YOLOE_LOG" 2>/dev/null \
            | awk -F'accepted=' '{print $2}' | awk -v code="$2" '$1==code{n++} END{print n+0}'
    else
        echo 0
    fi
}

status_name() {
    case "$1" in
        0) echo REJECTED;;
        1) echo CONFIRMED;;
        2) echo NO_TARGET;;
        3) echo TIMEOUT;;
        4) echo CANCELLED;;
        5) echo ERROR;;
        *) echo "UNKNOWN($1)";;
    esac
}

run_phase4_tally() {
    log "===== Phase 4: result status tally ====="
    if [ ! -f "$YOLOE_LOG" ]; then
        err "  no YOLOE log at $YOLOE_LOG"
        return
    fi
    print_breakdown() {
        local cam="$1" label="$2"
        echo "  status breakdown ($label):"
        grep "CONTINUOUS_RESULT camera=$cam" "$YOLOE_LOG" 2>/dev/null \
            | awk -F'accepted=' '{print $2}' | awk '{print $1}' \
            | sort | uniq -c \
            | while read count status; do
                case "$status" in
                    0) name="REJECTED";;
                    1) name="CONFIRMED";;
                    2) name="NO_TARGET";;
                    3) name="TIMEOUT";;
                    4) name="CANCELLED";;
                    5) name="ERROR";;
                    *) name="UNKNOWN($status)";;
                esac
                printf "    %4d × %s\n" "$count" "$name"
            done
    }
    print_breakdown head "camera=head"
    print_breakdown arm  "camera=arm"
    local head_msgs arm_msgs
    head_msgs=$(grep -c 'CONTINUOUS_RESULT camera=head' "$YOLOE_LOG" 2>/dev/null || echo 0)
    arm_msgs=$(grep -c 'CONTINUOUS_RESULT camera=arm'  "$YOLOE_LOG" 2>/dev/null || echo 0)
    log "  total: ${head_msgs} head msgs + ${arm_msgs} arm msgs"
    log "  YOLOE log: $YOLOE_LOG"
}

run_phase5_csv() {
    log "===== Phase 5: per-camera CSV stats ====="
    local CSV_OUT="${CSV_OUT:-/tmp/test_yoloe_stability_stats.csv}"
    local RUN_TS
    RUN_TS=$(date -Iseconds)
    local HEAD_FPS ARM_FPS
    HEAD_FPS=$(fps_from_log head)
    ARM_FPS=$(fps_from_log arm)

    {
        echo "timestamp,camera,metric,value"
        for cam in head arm; do
            for code in 0 1 2 3 4 5; do
                local count metric
                count=$(status_count "$cam" "$code")
                metric=$(status_name "$code")
                echo "${RUN_TS},${cam},${metric},${count}"
            done
            echo "${RUN_TS},${cam},msg_total,$(msg_total "$cam")"
            case "$cam" in
                head) local fps="$HEAD_FPS";;
                arm)  local fps="$ARM_FPS";;
            esac
            echo "${RUN_TS},${cam},fps_avg,${fps}"
        done
    } > "$CSV_OUT"

    echo "  per-camera summary:"
    for cam in head arm; do
        case "$cam" in
            head) local fps="$HEAD_FPS";;
            arm)  local fps="$ARM_FPS";;
        esac
        echo "    ${cam}:"
        for code in 0 1 2 3 4 5; do
            local count metric
            count=$(status_count "$cam" "$code")
            metric=$(status_name "$code")
            printf "      %-10s %4d\n" "$metric" "$count"
        done
        printf "      %-10s %4d\n" "msg_total" "$(msg_total "$cam")"
        printf "      %-10s %5s Hz (from log timestamps)\n" "fps_avg" "$fps"
    done
    log "  CSV: $CSV_OUT"
}

run_phase6_health() {
    log "===== Phase 6: final camera stream health ====="
    for ns in head arm; do
        for stream in color/image_raw aligned_depth_to_color/image_raw; do
            local pub
            pub=$(docker exec -e LGW_INSTALL="${LGW_INSTALL:-}" "$CONTAINER" bash -c "
                source /opt/ros/humble/setup.bash >/dev/null 2>&1
                source $WS_L4T/ros2_ws/install/local_setup.bash >/dev/null 2>&1
                ros2 topic info /camera_${ns}/camera_${ns}/${stream} 2>/dev/null \
                    | grep 'Publisher count' | awk '{print \$NF}'
            " 2>/dev/null | tr -d '[:space:]')
            log "  /camera_${ns}/camera_${ns}/${stream}: pub=${pub:-0}"
        done
    done
}

# ===== cleanup =====
cleanup() {
    local exit_code=$?
    if [ "${CLEANUP_DONE:-0}" = "1" ]; then
        return
    fi
    CLEANUP_DONE=1

    log "===== cleanup (exit=$exit_code) ====="

    # Final summary only if YOLOE came up (YOLOE_LOG created)
    if [ -n "$YOLOE_LOG" ] && [ -f "$YOLOE_LOG" ]; then
        run_phase4_tally
        run_phase5_csv
        run_phase6_health
    fi

    if [ ${#HOST_PIDS[@]} -gt 0 ]; then
        for ((i=${#HOST_PIDS[@]}-1; i>=0; i--)); do
            local pid="${HOST_PIDS[$i]}"
            log "  host: kill -15 $pid"
            kill -15 "$pid" 2>/dev/null || true
            sleep 1
        done
        sleep 2
    fi
    if [ ${#CONTAINER_PIDS[@]} -gt 0 ]; then
        for ((i=${#CONTAINER_PIDS[@]}-1; i>=0; i--)); do
            local pid="${CONTAINER_PIDS[$i]}"
            log "  container: kill -15 $pid"
            docker exec -e LGW_INSTALL="${LGW_INSTALL:-}" "$CONTAINER" bash -c "kill -15 $pid 2>/dev/null || true"
            sleep 1
        done
        sleep 2
    fi
    log "verifying no orphan realsense2_camera_node children..."
    local orphans
    orphans=$(docker exec -e LGW_INSTALL="${LGW_INSTALL:-}" "$CONTAINER" bash -c \
        "ps -eo pid,args 2>/dev/null \
          | grep 'realsense2_camera_node' \
          | grep -v grep | awk '{print \$1}'" 2>/dev/null || true)
    if [ -n "$orphans" ]; then
        for pid in $orphans; do
            err "  killing orphan: $pid"
            docker exec -e LGW_INSTALL="${LGW_INSTALL:-}" "$CONTAINER" bash -c "kill -15 $pid 2>/dev/null || true"
        done
    fi
    log "===== cleanup done ====="
    exit "$exit_code"
}
trap cleanup EXIT INT TERM

# ===== Phase 0: prerequisites =====
log "===== Phase 0: prerequisites ====="
fail=0
docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$" || { err "container down"; fail=1; }
[ -f "$YOLOE_DEPLOY/ros2_ws/install/local_setup.bash" ] || { err "yoloe deploy not built"; fail=1; }
docker exec -e LGW_INSTALL="${LGW_INSTALL:-}" "$CONTAINER" bash -c "[ -f $HUMBLE_YOLOE_INSTALL ]" 2>/dev/null \
    || { err "humble_yoloe_install missing"; fail=1; }
[ -f "$YOLOE_DEPLOY/models/yoloe-26n-seg.pt" ] || { err "model missing"; fail=1; }
[ -f "$YOLOE_DEPLOY/prompt_assets/red_black_c2.jpg" ] || { err "prompt asset missing"; fail=1; }
[ "$(ls /dev/video* 2>/dev/null | wc -l)" -ge 2 ] || { err "<2 video devices"; fail=1; }
[ "$fail" -ne 0 ] && exit 1
log "  Phase 0 PASS"
[ "$CHECK_ONLY" -eq 1 ] && exit 0

# ===== Phase 1: YOLOE host Foxy (FIRST per plan) =====
log "===== Phase 1: YOLOE host Foxy (FIRST,避开模型加载 CPU spike 与 cameras USB init 撞车) ====="
YOLOE_LOG=/tmp/test_yoloe_stability_yoloe.log
LOG_FILES+=("$YOLOE_LOG")
nohup bash -c "
    set +u
    set -a
    source $YOLOE_DEPLOY/config/foxy_humble_dds.env
    set +a
    source /opt/ros/foxy/setup.bash
    source $YOLOE_DEPLOY/ros2_ws/install/local_setup.bash
    export LD_PRELOAD=/lib/aarch64-linux-gnu/libgomp.so.1
    export PYTHONUNBUFFERED=1
    ros2 launch yoloe_perception dual_camera_foxy.launch.py \
        transfer_root:=\"\$YOLOE_DEPLOY\"
" > "$YOLOE_LOG" 2>&1 &
YOLOE_PID=$!
HOST_PIDS+=("$YOLOE_PID")
log "  YOLOE host PID: $YOLOE_PID, log: $YOLOE_LOG"

yoloe_ready=0
for attempt in $(seq 1 30); do
    sleep 2
    if bash -c "
        source $YOLOE_DEPLOY/ros2_ws/install/local_setup.bash >/dev/null 2>&1
        ros2 node list 2>/dev/null | grep -q yoloe_dual_camera_perception
    " 2>/dev/null; then
        log "  YOLOE node registered after ${attempt} polls (~$((attempt*2))s)"
        yoloe_ready=1
        break
    fi
done
[ "$yoloe_ready" -ne 1 ] && { err "YOLOE didn't come up — see $YOLOE_LOG"; exit 1; }

# ===== Phase 1.5: wait for CONTINUOUS_DUAL_CAMERA_STARTED =====
# 出处: dual_camera_node.py:335,在 __init__ 末尾 emit(YOLOE 模型加载完、
# subscriber 注册、timer 已调度);不依赖 cameras 已起。
log "===== Phase 1.5: wait for CONTINUOUS_DUAL_CAMERA_STARTED (YOLOE 模型加载完成) ====="
seen_started=0
for attempt in $(seq 1 60); do
    sleep 2
    [ -f "$YOLOE_LOG" ] || continue
    if grep -q 'CONTINUOUS_DUAL_CAMERA_STARTED' "$YOLOE_LOG" 2>/dev/null; then
        log "  CONTINUOUS_DUAL_CAMERA_STARTED seen after ${attempt} polls (~$((attempt*2))s)"
        seen_started=1
        break
    fi
done
if [ "$seen_started" != "1" ]; then
    err "YOLOE 未在 120s 内 emit CONTINUOUS_DUAL_CAMERA_STARTED — 模型加载可能失败"
    err "  log: $YOLOE_LOG"
    exit 1
fi

# ===== Phase 1.6: USB force reset (kernel-level unbind+bind) =====
# 2026-10-04 Path A: 解决 kernel libusb handle 残留。
# 上一次 run realsense2_camera_node SIGSEGV 时 libusb 不会跑 exit handler,
# kernel 保留 device handle,下次 claim interface 0 → RS2_USB_STATUS_BUSY。
# unbind + bind 强制 kernel 重枚举设备,等价于 kernel 端拔插。
#
# 配合:
#   - Path C initial_reset=true(device 端硬件重置,realsense_node_factory.cpp:247)
#   - Path D reconnect_timeout=15.0(给 arm 同样的 12s+ 恢复窗口)
#   - launch 端 TimerAction 错开两相机启动 5s(避免并发抢 libusb context)
#
# 在容器内执行(容器是 root 且 /sys/bus/usb/ 已 bind mount)。
# USB 端口路径通过 ID_PATH 或扫描确定;此处按已知 topology 写死:
#   head (D435i, serial 261622078621) -> /sys/bus/usb/devices/2-1
#   arm  (D435,  serial 262422071246) -> /sys/bus/usb/devices/1-3.3
# 若设备换了 USB 端口,需重跑 `lsusb -t` 修正。
log "===== Phase 1.6: USB force reset (kernel unbind+bind for both cameras) ====="
HEAD_USB_PORT="${HEAD_USB_PORT:-2-1}"     # head D435i on USB 3.0 root hub
ARM_USB_PORT="${ARM_USB_PORT:-1-3.3}"     # arm  D435  on USB 2.0 hub
docker exec "$CONTAINER" bash -c '
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
    err "USB force reset 失败 — 可能是 USB 端口变了或权限问题"
    err "  排查:在 host 上跑 \`lsusb -t\` 确认 head/arm 各在哪个 port,"
    err "  然后通过 HEAD_USB_PORT= / ARM_USB_PORT= 环境变量覆盖。"
    exit 1
}
log "  USB force reset OK"

# ===== Phase 2: cameras in container (AFTER YOLOE ready) =====
log "===== Phase 2: cameras in container (YOLOE 已就绪,USB init 不再与 CPU spike 撞车) ====="
CAMERAS_LOG=/tmp/test_yoloe_stability_cameras.log
LOG_FILES+=("$CAMERAS_LOG")
docker exec -e LGW_INSTALL="${LGW_INSTALL:-}" "$CONTAINER" bash -c "
    source /opt/ros/humble/setup.bash >/dev/null 2>&1
    source $WS_L4T/ros2_ws/install/local_setup.bash >/dev/null 2>&1
    source $HUMBLE_YOLOE_INSTALL >/dev/null 2>&1
    export HEAD_CAMERA_SERIAL='$HEAD_CAMERA_SERIAL'
    export ARM_CAMERA_SERIAL='$ARM_CAMERA_SERIAL'
    export ROS_DOMAIN_ID='$ROS_DOMAIN_ID'
    export RMW_IMPLEMENTATION='$RMW_IMPLEMENTATION'
    nohup ros2 launch yoloe_humble_orchestration two_realsense.launch.py \
        head_serial:=\"\$HEAD_CAMERA_SERIAL\" \
        arm_serial:=\"\$ARM_CAMERA_SERIAL\" \
        > /tmp/two_realsense_inside.log 2>&1 &
    echo \$!
" > "$CAMERAS_LOG" 2>&1
CAMERAS_IN_PID=$(tail -1 "$CAMERAS_LOG" | tr -d '[:space:]')
log "  cameras inside-container PID: $CAMERAS_IN_PID, log: /tmp/two_realsense_inside.log"

cameras_ready=0
for attempt in $(seq 1 30); do
    sleep 2
    nc=$(docker exec -e LGW_INSTALL="${LGW_INSTALL:-}" "$CONTAINER" bash -c "
        source /opt/ros/humble/setup.bash >/dev/null 2>&1
        source $HUMBLE_YOLOE_INSTALL >/dev/null 2>&1
        ros2 node list 2>/dev/null \
          | grep -E '^/camera_(head|arm)/camera_(head|arm)\$' \
          | sort -u | wc -l
    " 2>/dev/null || echo 0)
    [ "$nc" = "2" ] && { log "  cameras up after ${attempt} polls (~$((attempt*2))s)"; cameras_ready=1; break; }
done
[ "$cameras_ready" -ne 1 ] && { err "cameras didn't come up — see /tmp/two_realsense_inside.log"; exit 1; }

# Capture both realsense2_camera_node PIDs for cleanup (they reparent to
# init when the wrapper dies, so we track them explicitly).
mapfile -t CAM_CHILD_PIDS < <(docker exec -e LGW_INSTALL="${LGW_INSTALL:-}" "$CONTAINER" bash -c \
    "ps -eo pid,args 2>/dev/null \
      | grep 'realsense2_camera_node' | grep -v grep | awk '{print \$1}'" \
    2>/dev/null)
for pid in "${CAM_CHILD_PIDS[@]}"; do
    CONTAINER_PIDS+=("$pid")
done
log "  captured camera child PIDs: ${CAM_CHILD_PIDS[*]}"

# ===== Phase 2.5: wait for first CONTINUOUS_RESULT for head AND arm =====
# Cameras are up, so YOLOE has image input by the time it lands.
# Polls $YOLOE_LOG every 2s up to 60 polls (120s); accepts ANY
# CONTINUOUS_RESULT line (status=0..5 — even NO_TARGET counts as proof
# that YOLOE is reading images and producing results).
log "===== Phase 2.5: wait for first CONTINUOUS_RESULT for head & arm ====="
seen_head=0
seen_arm=0
for attempt in $(seq 1 60); do
    sleep 2
    [ -f "$YOLOE_LOG" ] || continue
    grep -q 'CONTINUOUS_RESULT camera=head' "$YOLOE_LOG" 2>/dev/null && seen_head=1
    grep -q 'CONTINUOUS_RESULT camera=arm'  "$YOLOE_LOG" 2>/dev/null && seen_arm=1
    if [ "$seen_head" = "1" ] && [ "$seen_arm" = "1" ]; then
        log "  first CONTINUOUS_RESULT seen for both cameras after ${attempt} polls (~$((attempt*2))s)"
        break
    fi
done
if [ "$seen_head" != "1" ] || [ "$seen_arm" != "1" ]; then
    err "YOLOE didn't publish CONTINUOUS_RESULT for both cameras (head=$seen_head arm=$seen_arm) — see $YOLOE_LOG"
    exit 1
fi

# ===== Print monitor command panel =====
log "===== Pipeline live. Ctrl-C to stop. Monitoring... ====="
log "  监控命令清单(另开 host Foxy 终端):"
log "    cat /home/unitree/my_ws/l4t/lg_walker_ws/test/MONITOR_COMMANDS.txt"
log "  快速参考:"
log "    tail -f $YOLOE_LOG"
log "    ros2 topic hz /camera_head/camera_head/color/image_raw"
log "    ros2 topic hz /camera_arm/camera_arm/color/image_raw"
log "    ros2 node list | grep -E 'camera|yoloe'"
log "    docker exec self_l4t tail -f /tmp/two_realsense_inside.log"

# ===== Phase 3: continuous monitor loop (until Ctrl-C or DURATION_SEC) =====
log "===== Phase 3: continuous monitor (Ctrl-C to stop, sampling every ${SAMPLE_INTERVAL}s) ====="
iteration=0
while true; do
    iteration=$((iteration + 1))
    sleep "$SAMPLE_INTERVAL"
    elapsed=$(( $(date +%s) - START_TS ))

    # Check optional DURATION_SEC timeout (default 0 = infinite)
    if [ "$DURATION_SEC" -gt 0 ] && [ "$elapsed" -ge "$DURATION_SEC" ]; then
        log "===== DURATION_SEC=${DURATION_SEC} reached, exiting loop ====="
        break
    fi

    log "----- t=${elapsed}s sample ${iteration} -----"
    for ns in head arm; do
        # Combined liveness probe: pub_count + color hz rate, single
        # docker exec per camera per iteration (saves ~3-5s vs two
        # separate execs — Phase 3 was running 5x slower than DURATION_SEC
        # due to docker startup overhead).
        sample=$(timeout 6 docker exec -e LGW_INSTALL="${LGW_INSTALL:-}" "$CONTAINER" bash -c "
            source /opt/ros/humble/setup.bash >/dev/null 2>&1
            source $WS_L4T/ros2_ws/install/local_setup.bash >/dev/null 2>&1
            echo \"PUB:\$(ros2 topic info /camera_${ns}/camera_${ns}/color/image_raw 2>/dev/null \
                | grep -E 'Publisher count' | awk '{print \$NF}')\"
            echo \"RATE:\$(timeout 3 ros2 topic hz /camera_${ns}/camera_${ns}/color/image_raw 2>&1 | head -1)\"
        " 2>/dev/null)
        c_pub=$(echo "$sample" | grep '^PUB:' | sed 's/^PUB://' | tr -d '[:space:]')
        # 'average rate: X.XXX' is the FIRST line of ros2 topic hz output.
        # Earlier bug used `tail -2 | head -1` which grabbed the
        # 'min: ...' line and never matched 'rate: N'.
        rate=$(echo "$sample" | grep '^RATE:' | sed 's/^RATE://' \
            | grep -oE 'rate: [0-9.]+' || echo "rate: ?")

        # Latest CONTINUOUS_RESULT for this camera (from YOLOE log)
        latest_result=""
        if [ -f "$YOLOE_LOG" ]; then
            latest_result=$(grep "CONTINUOUS_RESULT camera=$ns" "$YOLOE_LOG" 2>/dev/null \
                | tail -1 \
                | grep -oE 'accepted=[0-9]+' | head -1)
        fi
        log "  ${ns} color: pub_count=${c_pub:-?} ${rate}  | latest ${latest_result:-NO RESULT YET}"
    done

    # Process liveness check
    if [ "$YOLOE_PID" -gt 0 ] && ! kill -0 "$YOLOE_PID" 2>/dev/null; then
        err "YOLOE host PID $YOLOE_PID died — check $YOLOE_LOG"
    fi
done

# Loop exit only via DURATION_SEC timeout (Ctrl-C went through trap cleanup).
# Calling cleanup() explicitly triggers Phase 4/5/6 summary + kill + exit.
# The trap fires again on exit but CLEANUP_DONE guard makes it a no-op.
cleanup
