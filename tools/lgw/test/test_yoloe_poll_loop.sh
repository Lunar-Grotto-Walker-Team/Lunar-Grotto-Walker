#!/bin/bash
# test_yoloe_poll_loop.sh — self-contained YOLOE-first e2e: bring up
# YOLOE → cameras → bt_runner, then poll BT goals while perf_monitor.py
# counts YOLOE results. Tally BT SUCCESS/ABORTED and the perf_monitor
# SUM line.
#
# Why YOLOE-first (decided 2026-09-29): the user wants startup ordering
# that proves YOLOE actually emits a result before BT begins — but the
# YOLOE inference loop needs camera images to write CONTINUOUS_RESULT
# (idle otherwise), so the verification happens AFTER cameras are up:
#   Phase 1 (host Foxy): start YOLOE node, register publishers (no image
#     input yet, so no CONTINUOUS_RESULT — that's OK, just node up)
#   Phase 2 (container): start cameras
#   Phase 2.5: poll $YOLOE_LOG until head AND arm each publish at least
#     one CONTINUOUS_RESULT (any status 1..5 — proof the inference loop
#     is running end-to-end, not a dead node shell)
#   Phase 3: start bt_runner (source l4t_setup.sh)
#   Phase 3.5: verify YOLOE result topic pub_count≥1 inside container
#     (cross-DDS via cyclonedds)
#   Phase 4: launch run_perf_monitor.sh in background for the polling
#     window
#   Phase 5: while-elapsed < DURATION_SEC, send /lgw_bt_execution goals,
#     tally SUCCEEDED vs ABORTED
#   Phase 6: wait ~12s for the next perf_monitor SUM (it ticks every
#     10s), kill it, parse the last SUM line
#   Phase 7: final summary — BT tallies + SUM head/arm + fps (res/s)
#
# Process tree (all cleanup tracked via PID arrays, no blanket kill):
#   HOST:    YOLOE dual_camera_foxy.launch.py   (Phase 1)
#            perf_monitor.py via run_perf_monitor.sh  (Phase 4)
#   CONTAINER (self_l4t): cameras two_realsense.launch.py  (Phase 2)
#                          bt_runner lgw_bt_planner.launch.py  (Phase 3)
#
# Usage:
#   bash test_yoloe_poll_loop.sh [duration_sec]   # default 60
#   bash test_yoloe_poll_loop.sh --check          # prerequisites only
#
# Env overrides:
#   DURATION_SEC, POLL_DURATION_SEC (alias)
#   HEAD_CAMERA_SERIAL, ARM_CAMERA_SERIAL
#   BT_TREE_ID (default DetectionConditionTest)
#   BT_GOAL_TIMEOUT_SEC (default 15)
#   PERF_MONITOR_WIN_SEC (default 12, must be > perf_monitor.py WIN=10)

set -uo pipefail

DURATION_SEC="${1:-${POLL_DURATION_SEC:-60}}"
if [ "$DURATION_SEC" = "--check" ]; then
    DURATION_SEC=0
fi

HEAD_CAMERA_SERIAL="${HEAD_CAMERA_SERIAL:-_261622078621}"
ARM_CAMERA_SERIAL="${ARM_CAMERA_SERIAL:-_262422071246}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
YOLOE_DEPLOY="${YOLOE_DEPLOY:-$HOME/yoloe_deploy}"
CONTAINER="${CONTAINER:-self_l4t}"
WS_L4T="${WS_L4T:-/my_ws/l4t}"
HUMBLE_YOLOE_INSTALL="${HUMBLE_YOLOE_INSTALL:-/opt/yoloe_transfer/humble_ws/install/local_setup.bash}"
BT_TREE_ID="${BT_TREE_ID:-DetectionConditionTest}"
BT_GOAL_TIMEOUT_SEC="${BT_GOAL_TIMEOUT_SEC:-15}"
PERF_MONITOR_SH="${PERF_MONITOR_SH:-$YOLOE_DEPLOY/evidence/usb_diag_20260918/run_perf_monitor.sh}"
PERF_MONITOR_LOG="${PERF_MONITOR_LOG:-/tmp/test_yoloe_poll_loop_perf.log}"
# Wait this long after polling ends to capture perf_monitor's next SUM
# (perf_monitor.py prints SUM every 10s).
PERF_MONITOR_WIN_SEC="${PERF_MONITOR_WIN_SEC:-12}"
export YOLOE_DEPLOY  # inherited by nohup bash -c subshell

CONTAINER_PIDS=()
HOST_PIDS=()
LOG_FILES=()

ts() { date +'%H:%M:%S'; }
log() { echo "[$(ts)] $*"; }
err() { echo "[$(ts)] [ERROR] $*" >&2; }

cleanup() {
    local exit_code=$?
    log "===== cleanup (exit=$exit_code) ====="
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
    # Catch orphan realsense2_camera_node children that reparented to init
    # (same pattern as test_yoloe_stability.sh).
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
log "===== Phase 0: prerequisites (duration=${DURATION_SEC}s) ====="
fail=0
docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$" || { err "container down"; fail=1; }
[ -f "$YOLOE_DEPLOY/ros2_ws/install/local_setup.bash" ] || { err "yoloe deploy not built"; fail=1; }
docker exec -e LGW_INSTALL="${LGW_INSTALL:-}" "$CONTAINER" bash -c "[ -f $HUMBLE_YOLOE_INSTALL ]" 2>/dev/null \
    || { err "humble_yoloe_install missing"; fail=1; }
[ -f "$YOLOE_DEPLOY/models/yoloe-26n-seg.pt" ] || { err "model missing"; fail=1; }
[ -f "$YOLOE_DEPLOY/prompt_assets/red_black_c2.jpg" ] || { err "prompt asset missing"; fail=1; }
[ -x "$PERF_MONITOR_SH" ] || { err "perf_monitor.sh missing or not executable: $PERF_MONITOR_SH"; fail=1; }
[ "$(ls /dev/video* 2>/dev/null | wc -l)" -ge 2 ] || { err "<2 video devices"; fail=1; }
[ "$fail" -ne 0 ] && exit 1
log "  Phase 0 PASS"
[ "$DURATION_SEC" = "0" ] && exit 0

# ===== Phase 1: start YOLOE on host Foxy =====
log "===== Phase 1: YOLOE host Foxy ====="
YOLOE_LOG=/tmp/test_yoloe_poll_loop_yoloe.log
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
log "  YOLOE host PID: $YOLOE_PID"

yoloe_ready=0
for attempt in $(seq 1 30); do
    sleep 2
    if bash -c "
        source $YOLOE_DEPLOY/ros2_ws/install/local_setup.bash >/dev/null 2>&1
        ros2 node list 2>/dev/null | grep -q yoloe_dual_camera_perception
    " 2>/dev/null; then
        log "  YOLOE node registered after ${attempt} polls"
        yoloe_ready=1
        break
    fi
done
[ "$yoloe_ready" -ne 1 ] && { err "YOLOE didn't come up — see $YOLOE_LOG"; exit 1; }

# ===== Phase 2: start cameras =====
log "===== Phase 2: cameras ====="
CAMERAS_LOG=/tmp/test_yoloe_poll_loop_cameras.log
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
log "  cameras inside-container PID: $CAMERAS_IN_PID"

cameras_ready=0
for attempt in $(seq 1 30); do
    sleep 2
    nc=$(docker exec -e LGW_INSTALL="${LGW_INSTALL:-}" "$CONTAINER" bash -c "
        source /opt/ros/humble/setup.bash >/dev/null 2>&1
        source $HUMBLE_YOLOE_INSTALL >/dev/null 2>&1
        ros2 node list 2>/dev/null \
          | grep -E '^/camera_(head|arm)/camera_(head|arm)$' \
          | sort -u | wc -l
    " 2>/dev/null || echo 0)
    [ "$nc" = "2" ] && { log "  cameras up after ${attempt} polls"; cameras_ready=1; break; }
done
[ "$cameras_ready" -ne 1 ] && { err "cameras didn't come up"; exit 1; }

mapfile -t CAM_CHILD_PIDS < <(docker exec -e LGW_INSTALL="${LGW_INSTALL:-}" "$CONTAINER" bash -c \
    "ps -eo pid,args 2>/dev/null \
      | grep 'realsense2_camera_node' | grep -v grep | awk '{print \$1}'" \
    2>/dev/null)
for pid in "${CAM_CHILD_PIDS[@]}"; do
    CONTAINER_PIDS+=("$pid")
done
log "  captured camera child PIDs: ${CAM_CHILD_PIDS[*]}"

# ===== Phase 2.5: wait for first CONTINUOUS_RESULT for both cameras =====
# Cameras are up (Phase 2), so YOLOE has image input. Polls $YOLOE_LOG
# every 2s up to 60 polls (120s); accepts ANY CONTINUOUS_RESULT line
# (status 1..5 — even NO_TARGET counts as proof that YOLOE is reading
# images and producing results).
log "===== Phase 2.5: waiting for first CONTINUOUS_RESULT for head & arm ====="
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

# ===== Phase 3: start bt_runner in self_l4t =====
log "===== Phase 3: bt_runner in self_l4t ====="
BT_LOG=/tmp/test_yoloe_poll_loop_bt_runner.log
LOG_FILES+=("$BT_LOG")
docker exec -e LGW_INSTALL="${LGW_INSTALL:-}" "$CONTAINER" bash -c "
    source $WS_L4T/l4t_setup.sh
    export PYTHONUNBUFFERED=1
    nohup ros2 launch lgw_bt_planner lgw_bt_planner.launch.py \
        > /tmp/bt_runner_inside.log 2>&1 &
    echo \$!
" > "$BT_LOG" 2>&1
BT_IN_CONTAINER_PID=$(tail -1 "$BT_LOG" | tr -d '[:space:]')
log "  bt_runner inside-container PID: $BT_IN_CONTAINER_PID"

bt_ready=0
for attempt in $(seq 1 30); do
    sleep 1
    if docker exec -e LGW_INSTALL="${LGW_INSTALL:-}" "$CONTAINER" bash -c "
        source $WS_L4T/l4t_setup.sh 2>/dev/null
        ros2 action list 2>/dev/null | grep -q '^/lgw_bt_execution$'
    " 2>/dev/null; then
        log "  bt_runner up after ${attempt}s"
        bt_ready=1
        break
    fi
done
[ "$bt_ready" -ne 1 ] && { err "bt_runner did not come up"; exit 1; }

# Add bt_runner wrapper PID (in container) so cleanup kills it too.
BT_WRAPPER_PID=$(docker exec -e LGW_INSTALL="${LGW_INSTALL:-}" "$CONTAINER" bash -c \
    "ps -eo pid,args 2>/dev/null | grep 'ros2 launch lgw_bt_planner' | grep -v grep | awk '{print \$1}' | head -1" \
    2>/dev/null || echo "")
if [ -n "$BT_WRAPPER_PID" ]; then
    CONTAINER_PIDS+=("$BT_WRAPPER_PID")
fi

# ===== Phase 3.5: verify YOLOE result topics are reachable from container =====
# perf_monitor.sh will subscribe to these (cross-DDS via cyclonedds).
log "===== Phase 3.5: verifying YOLOE result topic publishers ====="
for topic in \
    /perception/two_stage/camera_head/result \
    /perception/two_stage/camera_arm/result
do
    pub_ok=0
    for _ in $(seq 1 12); do
        sleep 1
        pub_count=$(docker exec -e LGW_INSTALL="${LGW_INSTALL:-}" "$CONTAINER" bash -c "
            source $WS_L4T/l4t_setup.sh >/dev/null 2>&1
            ros2 topic info $topic 2>/dev/null | grep 'Publisher count' | awk '{print \$NF}'
        " 2>/dev/null || echo "?")
        if [[ "$pub_count" =~ ^[0-9]+$ ]] && [ "$pub_count" -ge 1 ]; then
            pub_ok=1
            break
        fi
    done
    if [ "$pub_ok" -ne 1 ]; then
        err "YOLOE not publishing on $topic — aborting"
        exit 1
    fi
    log "    $topic: publisher active"
done

# ===== Phase 4: launch run_perf_monitor.sh in background =====
log "===== Phase 4: starting run_perf_monitor.sh (logs: $PERF_MONITOR_LOG) ====="
# run_perf_monitor.sh does `exec python3 ...` so the nohup PID is the
# python3 process. Background it; cleanup kills it via HOST_PIDS.
nohup bash "$PERF_MONITOR_SH" > "$PERF_MONITOR_LOG" 2>&1 &
PERF_PID=$!
HOST_PIDS+=("$PERF_PID")
log "  perf_monitor host PID: $PERF_PID"
# Give it ~2s to spin up rclpy and subscribe before we start the
# polling loop (otherwise the first SUM tick might miss the first
# few result messages).
sleep 2

# ===== Phase 5: BT polling loop =====
START_TS=$(date +%s)
SUCCESS_COUNT=0
ABORT_COUNT=0
TOTAL_COUNT=0

log "===== Phase 5: polling $BT_TREE_ID for ${DURATION_SEC}s (per-goal timeout ${BT_GOAL_TIMEOUT_SEC}s) ====="
while :; do
    ELAPSED=$(( $(date +%s) - START_TS ))
    if [ "$ELAPSED" -ge "$DURATION_SEC" ]; then
        break
    fi
    TOTAL_COUNT=$((TOTAL_COUNT + 1))
    GOAL_OUT=$(timeout "$BT_GOAL_TIMEOUT_SEC" docker exec -e LGW_INSTALL="${LGW_INSTALL:-}" "$CONTAINER" bash -c "
        source $WS_L4T/l4t_setup.sh >/dev/null 2>&1
        ros2 action send_goal /lgw_bt_execution \
            btcpp_ros2_interfaces/action/ExecuteTree \
            '{target_tree: $BT_TREE_ID}' 2>&1
    " 2>&1 || true)
    if echo "$GOAL_OUT" | grep -q 'Goal finished with status: SUCCEEDED'; then
        SUCCESS_COUNT=$((SUCCESS_COUNT + 1))
        STATUS="SUCCEEDED"
    else
        ABORT_COUNT=$((ABORT_COUNT + 1))
        STATUS="ABORTED"
    fi
    ELAPSED=$(( $(date +%s) - START_TS ))
    log "  goal#${TOTAL_COUNT} t=${ELAPSED}s status=${STATUS}  (S=${SUCCESS_COUNT} A=${ABORT_COUNT})"
done

POLL_ELAPSED=$(( $(date +%s) - START_TS ))
log "===== Phase 5 done: total=${TOTAL_COUNT} SUCCESS=${SUCCESS_COUNT} ABORTED=${ABORT_COUNT} elapsed=${POLL_ELAPSED}s ====="

# ===== Phase 6: capture last perf_monitor SUM =====
# perf_monitor.py prints SUM every 10s. Wait ~12s for the next tick to
# land, then kill -15. Tail -1 picks the most recent complete SUM line.
log "===== Phase 6: capturing last perf_monitor SUM (wait ${PERF_MONITOR_WIN_SEC}s) ====="
if [ -n "$PERF_PID" ]; then
    sleep "$PERF_MONITOR_WIN_SEC"
    kill -15 "$PERF_PID" 2>/dev/null || true
    sleep 2
    # Remove PERF_PID from HOST_PIDS so cleanup doesn't double-kill it.
    HOST_PIDS=("${HOST_PIDS[@]/$PERF_PID}")
fi

SUM_LINE=$(grep '^SUM' "$PERF_MONITOR_LOG" 2>/dev/null | tail -1 || true)
if [ -z "$SUM_LINE" ]; then
    err "  no SUM line captured from $PERF_MONITOR_LOG"
    SUM_LINE="(none)"
fi
log "  perf_monitor SUM: $SUM_LINE"

# ===== Phase 7: final summary =====
log "===== Phase 7: poll_loop summary ====="
log "  BT goals:    SUCCESS=${SUCCESS_COUNT}  ABORTED=${ABORT_COUNT}  total=${TOTAL_COUNT}  (polling elapsed=${POLL_ELAPSED}s)"
log "  perf_monitor: $SUM_LINE"

# Parse head/arm res from SUM for fps (res/s) — only meaningful if SUM
# was captured.
#
# SUM format (perf_monitor.py:108-114):
#   SUM  head:res=N conf=N none=N err=N det=N det_valid=N ovl=N | arm:res=N conf=N ...
# Only `res` carries the camera prefix; the rest of each side is just
# `key=N` whitespace-separated. Split on whitespace, then index.
if [ "$SUM_LINE" != "(none)" ]; then
    IFS=' ' read -ra SUM_PARTS <<< "$SUM_LINE"
    # Layout: [0]=SUM [1]=head:res=N [2]=conf=N [3]=none=N [4]=err=N
    #         [5]=det=N [6]=det_valid=N [7]=ovl=N [8]=| [9]=arm:res=N
    #         [10..15]=conf/none/err/det/det_valid/ovl
    kv_val() {
        # $1 = token like "head:res=206" or "conf=206"
        echo "${1#*=}"
    }
    HEAD_RES=$(kv_val "${SUM_PARTS[1]}")
    HEAD_CONF=$(kv_val "${SUM_PARTS[2]}")
    HEAD_NONE=$(kv_val "${SUM_PARTS[3]}")
    HEAD_ERR=$(kv_val "${SUM_PARTS[4]}")
    ARM_RES=$(kv_val "${SUM_PARTS[9]}")
    ARM_CONF=$(kv_val "${SUM_PARTS[10]}")
    ARM_NONE=$(kv_val "${SUM_PARTS[11]}")
    ARM_ERR=$(kv_val "${SUM_PARTS[12]}")
    HEAD_HIT="-"
    ARM_HIT="-"
    if [ -n "$HEAD_RES" ] && [ "$HEAD_RES" -gt 0 ] 2>/dev/null; then
        HEAD_HIT=$(awk -v c="$HEAD_CONF" -v t="$HEAD_RES" 'BEGIN{printf "%.1f%%", (c/t)*100}')
    fi
    if [ -n "$ARM_RES" ] && [ "$ARM_RES" -gt 0 ] 2>/dev/null; then
        ARM_HIT=$(awk -v c="$ARM_CONF" -v t="$ARM_RES" 'BEGIN{printf "%.1f%%", (c/t)*100}')
    fi
    # Use perf_monitor effective window (polling + PERF_MONITOR_WIN_SEC)
    # as the denominator; perf_monitor accumulates from Phase 4 start.
    EFFECTIVE_ELAPSED=$(( POLL_ELAPSED + PERF_MONITOR_WIN_SEC ))
    HEAD_FPS="-"
    ARM_FPS="-"
    if [ -n "$HEAD_RES" ] && [ "$HEAD_RES" -gt 0 ] 2>/dev/null && [ "$EFFECTIVE_ELAPSED" -gt 0 ]; then
        HEAD_FPS=$(awk -v r="$HEAD_RES" -v s="$EFFECTIVE_ELAPSED" 'BEGIN{printf "%.2f", r/s}')
    fi
    if [ -n "$ARM_RES" ] && [ "$ARM_RES" -gt 0 ] 2>/dev/null && [ "$EFFECTIVE_ELAPSED" -gt 0 ]; then
        ARM_FPS=$(awk -v r="$ARM_RES" -v s="$EFFECTIVE_ELAPSED" 'BEGIN{printf "%.2f", r/s}')
    fi
    log "  perf_monitor breakdown:"
    log "    head: res=${HEAD_RES:-?} conf=${HEAD_CONF:-?} none=${HEAD_NONE:-?} err=${HEAD_ERR:-?}  hit_rate=${HEAD_HIT}  fps=${HEAD_FPS} res/s"
    log "    arm:  res=${ARM_RES:-?}  conf=${ARM_CONF:-?}  none=${ARM_NONE:-?}  err=${ARM_ERR:-?}   hit_rate=${ARM_HIT}   fps=${ARM_FPS} res/s"
fi

log "===== test complete ====="
log "  logs: ${LOG_FILES[*]}  perf_monitor: $PERF_MONITOR_LOG"
log "  cleanup follows via trap..."