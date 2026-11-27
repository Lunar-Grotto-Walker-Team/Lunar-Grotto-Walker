#!/bin/bash
# test_yoloe_poll_e2e.sh — full pipeline + 60s polling driver.
#
# Combines the phase 1-4 process bring-up from test_yoloe_e2e.sh with
# the new test_yoloe_poll_loop.sh outer driver. Designed for the
# "BT is a finite mission; outer driver polls the goal until the user-
# specified wall-clock window" passes — decided 2026-09-29.
#
# Process tree:
#   T1 (container self_l4t): two_realsense.launch.py
#       -> /camera_head/camera_head
#       -> /camera_arm/camera_arm
#   T2 (host Foxy):           dual_camera_foxy.launch.py
#       -> /perception/two_stage/yoloe_dual_camera_perception
#   T3 (container self_l4t):  lgw_bt_planner.launch.py
#       -> /lgw_bt_execution action server
#
# Then runs test_yoloe_poll_loop.sh with the same wall-clock budget
# (default 60 s). Each individual goal can SUCCEED or ABORT;
# the polling driver tallies the counts.
#
# Environment overrides:
#   POLL_DURATION_SEC  - default 60
#   POLL_GOAL_TIMEOUT  - per-goal timeout, default 15
#   HEAD_CAMERA_SERIAL / ARM_CAMERA_SERIAL / ROS_DOMAIN_ID / RMW_IMPLEMENTATION

set -uo pipefail

POLL_DURATION_SEC="${POLL_DURATION_SEC:-60}"
POLL_GOAL_TIMEOUT="${POLL_GOAL_TIMEOUT:-15}"

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

# ===== Phase 1: start cameras =====
log "===== Phase 1: cameras ====="
CAMERAS_LOG=/tmp/test_yoloe_poll_e2e_cameras.log
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

# ===== Phase 2: start YOLOE on host Foxy =====
log "===== Phase 2: YOLOE host Foxy ====="
YOLOE_LOG=/tmp/test_yoloe_poll_e2e_yoloe.log
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
        log "  YOLOE up after ${attempt} polls"
        yoloe_ready=1
        break
    fi
done
[ "$yoloe_ready" -ne 1 ] && { err "YOLOE didn't come up — see $YOLOE_LOG"; exit 1; }

# ===== Phase 3: start bt_runner in self_l4t =====
log "===== Phase 3: bt_runner in self_l4t ====="
BT_LOG=/tmp/test_yoloe_poll_e2e_bt_runner.log
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

# Sanity: confirm YOLOE has published at least once to BOTH result topics
# before we start the polling driver (else the very first goal will ABORT).
log "  waiting up to 12s for YOLOE to publish to both result topics..."
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

# ===== Phase 4: run polling driver =====
log "===== Phase 4: polling driver (${POLL_DURATION_SEC}s, per-goal timeout ${POLL_GOAL_TIMEOUT}s) ====="
POLL_LOG=/tmp/test_yoloe_poll_e2e_poll.log
LOG_FILES+=("$POLL_LOG")
POLL_DURATION_SEC="$POLL_DURATION_SEC" \
POLL_GOAL_TIMEOUT="$POLL_GOAL_TIMEOUT" \
    bash "$(dirname "$0")/test_yoloe_poll_loop.sh" "$POLL_DURATION_SEC" \
    2>&1 | tee "$POLL_LOG"

log "===== test complete ====="
log "  logs: ${LOG_FILES[*]}"
log "  cleanup follows via trap..."