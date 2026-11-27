#!/bin/bash
# test_yoloe_e2e.sh — End-to-end smoke test for the detection-condition BT
# nodes (IsHead/ArmDetectionConfirmed) against the real YOLOE dual-camera
# pipeline per YOLOE operator's runbook (14 sections).
#
# Spawns 3 process classes in coordinated order, tracks every PID, and
# tears them all down on exit (success OR failure OR Ctrl+C) via per-PID
# kill -15 per CLAUDE.md global constraint #3 (no pkill -f, no killall,
# no kill -9 -1).
#
# Process tree:
#   T1 (container self_l4t): two_realsense.launch.py
#       -> /camera_head/camera_head
#       -> /camera_arm/camera_arm
#   T2 (host Foxy):           dual_camera_foxy.launch.py
#       -> /perception/two_stage/yoloe_dual_camera_perception
#   T3 (container self_l4t):  lgw_bt_planner.launch.py
#       -> /lgw_bt_execution (action server)
#
# Test:
#   - Phase 5 fires DetectionConditionTest Sequence tree.
#   - PASS criterion: feedback shows IsHeadDetectionConfirmed +
#     IsArmDetectionConfirmed both return SUCCESS within 2.5s of their
#     latest CONFIRMED ObservationResult, Sequence returns SUCCESS.
#
# Usage:
#   bash test_yoloe_e2e.sh            # run with default serials
#   bash test_yoloe_e2e.sh --check    # prerequisites only, no launch
#
# Environment overrides (export before running):
#   HEAD_CAMERA_SERIAL, ARM_CAMERA_SERIAL, ROS_DOMAIN_ID, BT_TREE_ID

set -uo pipefail

# ===== Configuration =====
HEAD_CAMERA_SERIAL="${HEAD_CAMERA_SERIAL:-_261622078621}"
ARM_CAMERA_SERIAL="${ARM_CAMERA_SERIAL:-_262422071246}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
BT_TREE_ID="${BT_TREE_ID:-DetectionConditionTest}"
YOLOE_DEPLOY="${YOLOE_DEPLOY:-$HOME/yoloe_deploy}"
# Export so the nohup bash -c subshell in Phase 3 inherits it (otherwise
# $YOLOE_DEPLOY expands to empty inside the subshell and ros2 launch
# rejects `transfer_root:=` as malformed).
export YOLOE_DEPLOY
CONTAINER="self_l4t"
WS_L4T="/my_ws/l4t"
HUMBLE_YOLOE_INSTALL="/opt/yoloe_transfer/humble_ws/install/local_setup.bash"

# Class-1 (CONTAINER) and class-2 (HOST) PIDs tracked separately because
# they need different kill commands: docker exec for container, direct
# kill for host.
CONTAINER_PIDS=()
HOST_PIDS=()
LOG_FILES=()

# ===== CLI parsing =====
CHECK_ONLY=0
CAMERAS_ONLY=0
for arg in "$@"; do
    case "$arg" in
        --check) CHECK_ONLY=1 ;;
        --cameras-only) CAMERAS_ONLY=1 ;;
        -h|--help)
            grep -E '^#( |$)' "$0" | sed 's/^# \?//'
            cat <<EOF

Flags:
  --check         prerequisites only, no launches
  --cameras-only  Phase 0..2 only (cameras + topic verify), skip YOLOE / bt_runner
EOF
            exit 0
            ;;
        *)
            echo "Unknown arg: $arg" >&2
            exit 2
            ;;
    esac
done

# ===== Logging =====
ts() { date +'%H:%M:%S'; }
log() { echo "[$(ts)] $*"; }
err() { echo "[$(ts)] [ERROR] $*" >&2; }

# ===== Cleanup (trap) =====
# Order: host processes first (they depend on cameras in container, so
# killing them first prevents orphan inference results), then container
# children in reverse-spawn order, finally verify clean.
cleanup() {
    local exit_code=$?
    log "===== cleanup (exit=$exit_code) ====="

    # 1) Host processes (YOLOE inference)
    if [ ${#HOST_PIDS[@]} -gt 0 ]; then
        log "killing ${#HOST_PIDS[@]} host PIDs..."
        for ((i=${#HOST_PIDS[@]}-1; i>=0; i--)); do
            local pid="${HOST_PIDS[$i]}"
            log "  host: kill -15 $pid"
            kill -15 "$pid" 2>/dev/null || true
            sleep 1
        done
        sleep 2
    fi

    # 2) Container processes (cameras, bt_runner)
    if [ ${#CONTAINER_PIDS[@]} -gt 0 ]; then
        log "killing ${#CONTAINER_PIDS[@]} container PIDs..."
        for ((i=${#CONTAINER_PIDS[@]}-1; i>=0; i--)); do
            local pid="${CONTAINER_PIDS[$i]}"
            log "  container: kill -15 $pid (via docker exec)"
            docker exec -e LGW_INSTALL="${LGW_INSTALL:-}" "$CONTAINER" bash -c "kill -15 $pid 2>/dev/null || true"
            sleep 1
        done
        sleep 2
    fi

    # 3) Force-kill anything still lingering (PID-by-PID, never blanket).
    #    Only checks processes that were OUR concern; uses individual kills.
    log "verifying no lingering test processes..."
    local lingering_container
    lingering_container=$(docker exec -e LGW_INSTALL="${LGW_INSTALL:-}" "$CONTAINER" bash -c \
        "ps -eo pid,comm,args 2>/dev/null \
          | grep -E 'two_realsense|bt_runner|dual_camera_foxy|ros2 launch' \
          | grep -v grep | awk '{print \$1}'" 2>/dev/null || true)
    if [ -n "$lingering_container" ]; then
        err "lingering container PIDs (force kill -15 one by one):"
        for pid in $lingering_container; do
            err "  forcing: kill -15 $pid"
            docker exec -e LGW_INSTALL="${LGW_INSTALL:-}" "$CONTAINER" bash -c "kill -15 $pid 2>/dev/null || true"
        done
        sleep 2
    fi

    local lingering_host
    lingering_host=$(ps -eo pid,args 2>/dev/null \
        | grep -E 'dual_camera_foxy|ros2 launch yoloe_perception' \
        | grep -v grep | awk '{print $1}' || true)
    if [ -n "$lingering_host" ]; then
        err "lingering host PIDs (force kill -15 one by one):"
        for pid in $lingering_host; do
            err "  forcing: kill -15 $pid"
            kill -15 "$pid" 2>/dev/null || true
        done
    fi

    log "===== cleanup done ====="
    exit "$exit_code"
}
trap cleanup EXIT INT TERM

# ===== Phase 0: Prerequisites =====
phase0_check() {
    log "===== Phase 0: prerequisites ====="
    local failures=0

    # Container running?
    if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER}$"; then
        err "container $CONTAINER not running"
        failures=$((failures+1))
    else
        log "  container $CONTAINER is up"
    fi

    # USB cameras attached?
    local cam_count
    cam_count=$(ls /dev/video* 2>/dev/null | wc -l)
    log "  /dev/video* count: $cam_count"
    if [ "$cam_count" -lt 2 ]; then
        err "expected >=2 video devices, found $cam_count"
        failures=$((failures+1))
    fi

    # yoloe_deploy installed?
    if [ ! -f "$YOLOE_DEPLOY/ros2_ws/install/local_setup.bash" ]; then
        err "yoloe_deploy ros2_ws not built: $YOLOE_DEPLOY/ros2_ws/install/local_setup.bash missing"
        failures=$((failures+1))
    else
        log "  yoloe_deploy ros2_ws install present"
    fi

    # Humble OLED container install (for cameras + orchestration)
    if ! docker exec -e LGW_INSTALL="${LGW_INSTALL:-}" "$CONTAINER" bash -c "[ -f $HUMBLE_YOLOE_INSTALL ]" 2>/dev/null; then
        err "humble_yoloe_install not in container at $HUMBLE_YOLOE_INSTALL"
        err "  (the runbook expects the YOLOE Humble container to be installed)"
        failures=$((failures+1))
    else
        log "  YOLOE Humble install present in container"
    fi

    # Model + prompt assets
    if [ ! -f "$YOLOE_DEPLOY/models/yoloe-26n-seg.pt" ]; then
        err "model missing: $YOLOE_DEPLOY/models/yoloe-26n-seg.pt"
        failures=$((failures+1))
    else
        log "  model present ($(du -h "$YOLOE_DEPLOY/models/yoloe-26n-seg.pt" | awk '{print $1}'))"
    fi
    if [ ! -f "$YOLOE_DEPLOY/prompt_assets/red_black_c2.jpg" ]; then
        err "prompt asset missing"
        failures=$((failures+1))
    else
        log "  prompt asset present"
    fi

    # lgw_bt_planner + yoloe_interfaces built in container
    if ! docker exec -e LGW_INSTALL="${LGW_INSTALL:-}" "$CONTAINER" bash -c "[ -f /my_ws/l4t/lg_walker_ws/install/setup.bash ]" 2>/dev/null; then
        err "lgw_bt_planner not built in container (run colcon build first)"
        failures=$((failures+1))
    else
        log "  lgw_bt_planner built in container"
    fi

    # Foxy on host
    if [ ! -f /opt/ros/foxy/setup.bash ]; then
        err "host Foxy not installed"
        failures=$((failures+1))
    else
        log "  host Foxy present"
    fi

    if [ $failures -gt 0 ]; then
        err "prerequisites failed ($failures issues)"
        return 1
    fi
    log "===== Phase 0 PASS ====="
}

phase0_check || exit 1
[ "$CHECK_ONLY" -eq 1 ] && { log "--check only, exiting"; exit 0; }

# ===== Phase 1: Start cameras in container =====
log "===== Phase 1: start RealSense dual cameras in $CONTAINER ====="
CAMERAS_LOG=/tmp/test_yoloe_e2e_cameras.log
LOG_FILES+=("$CAMERAS_LOG")
docker exec -e LGW_INSTALL="${LGW_INSTALL:-}" -d "$CONTAINER" bash -c "
    set +u
    export HEAD_CAMERA_SERIAL='$HEAD_CAMERA_SERIAL'
    export ARM_CAMERA_SERIAL='$ARM_CAMERA_SERIAL'
    export ROS_DOMAIN_ID='$ROS_DOMAIN_ID'
    export ROS_LOCALHOST_ONLY='$ROS_LOCALHOST_ONLY'
    export RMW_IMPLEMENTATION='$RMW_IMPLEMENTATION'
    source /opt/ros/humble/setup.bash
    source $WS_L4T/ros2_ws/install/local_setup.bash
    source $HUMBLE_YOLOE_INSTALL
    ros2 launch yoloe_humble_orchestration two_realsense.launch.py \
        head_serial:=\"\$HEAD_CAMERA_SERIAL\" \
        arm_serial:=\"\$ARM_CAMERA_SERIAL\"
" > "$CAMERAS_LOG" 2>&1 &
CAMERAS_LAUNCH_PID=$!
log "  camera launch wrapper PID (host): $CAMERAS_LAUNCH_PID"

# The actual ros2 launch process lives inside the container. Wait for
# the camera nodes to appear, which is the real signal that it's ready.
log "  waiting for /camera_head/camera_head + /camera_arm/camera_arm..."
cameras_ready=0
for attempt in $(seq 1 30); do
    sleep 2
    node_count=$(docker exec -e LGW_INSTALL="${LGW_INSTALL:-}" "$CONTAINER" bash -c "
        source /opt/ros/humble/setup.bash >/dev/null 2>&1
        source $HUMBLE_YOLOE_INSTALL >/dev/null 2>&1
        ros2 node list 2>/dev/null \
          | grep -E '^/camera_(head|arm)/camera_(head|arm)$' \
          | sort -u | wc -l
    " 2>/dev/null || echo 0)
    if [ "$node_count" = "2" ]; then
        log "  cameras up after ${attempt} polls (~$((attempt*2))s)"
        cameras_ready=1
        break
    fi
done
if [ "$cameras_ready" -ne 1 ]; then
    err "cameras did not come up within 60s — see $CAMERAS_LOG"
    err "  tail: $(tail -20 "$CAMERAS_LOG")"
    exit 1
fi

# Track the in-container PID by name match (best effort — actual PIDs are
# grandchildren of $CAMERAS_LAUNCH_PID and reparented to init if launch dies).
# Use ps to capture and re-verify on cleanup.
CAMERAS_IN_CONTAINER_PID=$(docker exec -e LGW_INSTALL="${LGW_INSTALL:-}" "$CONTAINER" bash -c \
    "ps -eo pid,args 2>/dev/null | grep 'ros2 launch yoloe_humble_orchestration' | grep -v grep | awk '{print \$1}' | head -1" \
    2>/dev/null || echo "")
if [ -n "$CAMERAS_IN_CONTAINER_PID" ]; then
    CONTAINER_PIDS+=("$CAMERAS_IN_CONTAINER_PID")
    log "  camera in-container PID: $CAMERAS_IN_CONTAINER_PID"
fi

# ===== Phase 2: Verify camera data =====
log "===== Phase 2: verify camera image topics ====="
for topic in \
    /camera_head/camera_head/color/image_raw \
    /camera_head/camera_head/aligned_depth_to_color/image_raw \
    /camera_arm/camera_arm/color/image_raw \
    /camera_arm/camera_arm/aligned_depth_to_color/image_raw
do
    pub_count=$(docker exec -e LGW_INSTALL="${LGW_INSTALL:-}" "$CONTAINER" bash -c "
        source /opt/ros/humble/setup.bash >/dev/null 2>&1
        source $HUMBLE_YOLOE_INSTALL >/dev/null 2>&1
        ros2 topic info $topic 2>/dev/null | grep 'Publisher count' | awk '{print \$NF}'
    " 2>/dev/null || echo "?")
    log "  $topic: Publisher count = $pub_count"
    if ! [[ "$pub_count" =~ ^[0-9]+$ ]] || [ "$pub_count" -lt 1 ]; then
        err "expected Publisher count >=1 for $topic, got $pub_count"
    fi
done

# Sample 10s of head color frame rate as a sanity check.
log "  10s sample of /camera_head/camera_head/color/image_raw..."
timeout 10 docker exec -e LGW_INSTALL="${LGW_INSTALL:-}" "$CONTAINER" bash -c "
    source /opt/ros/humble/setup.bash 2>/dev/null
    source $HUMBLE_YOLOE_INSTALL 2>/dev/null
    ros2 topic hz /camera_head/camera_head/color/image_raw 2>&1
" | tail -10 || log "  (hz sample ended)"

if [ "$CAMERAS_ONLY" -eq 1 ]; then
    log "===== --cameras-only: skipping YOLOE / bt_runner / action trigger ====="
    log "===== cameras-only test PASS ====="
    exit 0
fi

# ===== Phase 3: Start YOLOE inference on host Foxy =====
log "===== Phase 3: start YOLOE dual-camera inference on host Foxy ====="
YOLOE_LOG=/tmp/test_yoloe_e2e_yoloe.log
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
YOLOE_LAUNCH_PID=$!
HOST_PIDS+=("$YOLOE_LAUNCH_PID")
log "  YOLOE launch wrapper PID (host): $YOLOE_LAUNCH_PID"

# Wait for YOLOE node to register.
log "  waiting for /perception/two_stage/yoloe_dual_camera_perception..."
yoloe_ready=0
for attempt in $(seq 1 30); do   # YOLOE warmup is 8-16s + DDS discovery, 60s covers it
    sleep 2
    if bash -c "
        source $YOLOE_DEPLOY/ros2_ws/install/local_setup.bash 2>/dev/null
        ros2 node list 2>/dev/null | grep -q yoloe_dual_camera_perception
    " 2>/dev/null; then
        log "  YOLOE up after ${attempt} polls (~$((attempt*2))s)"
        yoloe_ready=1
        break
    fi
done
if [ "$yoloe_ready" -ne 1 ]; then
    err "YOLOE did not come up within 60s — see $YOLOE_LOG"
    err "  tail: $(tail -30 "$YOLOE_LOG")"
    exit 1
fi

# Track the actual ros2 launch + yoloe process PIDs on host.
yoloe_node_pid=$(pgrep -f 'dual_camera_foxy.launch.py' | head -1 || true)
if [ -n "$yoloe_node_pid" ]; then
    HOST_PIDS+=("$yoloe_node_pid")
    log "  YOLOE ros2 launch PID (host): $yoloe_node_pid"
fi

# ===== Phase 4: Start bt_runner in container =====
log "===== Phase 4: start bt_runner in $CONTAINER ====="
BT_LOG=/tmp/test_yoloe_e2e_bt_runner.log
LOG_FILES+=("$BT_LOG")
# Redirect INSIDE the container via nohup + &, because docker exec -d does
# not capture stdout back to host (the wrapper exits immediately, severing
# the pipe). Without this, $BT_LOG ends up 0 bytes and we can't see why
# the BT tree returned FAILURE.
docker exec -e LGW_INSTALL="${LGW_INSTALL:-}" "$CONTAINER" bash -c "
    source $WS_L4T/l4t_setup.sh
    export PYTHONUNBUFFERED=1
    nohup ros2 launch lgw_bt_planner lgw_bt_planner.launch.py \
        > /tmp/bt_runner_inside.log 2>&1 &
    echo \$!
" > "$BT_LOG" 2>&1
BT_IN_CONTAINER_PID_FROM_LAUNCH=$(tail -1 "$BT_LOG" | tr -d '[:space:]')
BT_LAUNCH_PID=""
log "  bt_runner inside-container PID: $BT_IN_CONTAINER_PID_FROM_LAUNCH"

log "  waiting for /lgw_bt_execution action server..."
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
if [ "$bt_ready" -ne 1 ]; then
    err "bt_runner did not come up within 30s — see $BT_LOG"
    err "  tail: $(tail -30 "$BT_LOG")"
    exit 1
fi

if [ -n "$BT_IN_CONTAINER_PID_FROM_LAUNCH" ] && [ "$BT_IN_CONTAINER_PID_FROM_LAUNCH" != "0" ]; then
    CONTAINER_PIDS+=("$BT_IN_CONTAINER_PID_FROM_LAUNCH")
fi
bt_in_container_pid=$(docker exec -e LGW_INSTALL="${LGW_INSTALL:-}" "$CONTAINER" bash -c \
    "ps -eo pid,args 2>/dev/null | grep 'ros2 launch lgw_bt_planner' | grep -v grep | awk '{print \$1}' | head -1" \
    2>/dev/null || echo "")
if [ -n "$bt_in_container_pid" ]; then
    CONTAINER_PIDS+=("$bt_in_container_pid")
    log "  bt_runner in-container PID: $bt_in_container_pid"
fi

# ===== Phase 5: Trigger DetectionConditionTest =====
log "===== Phase 5: send action goal $BT_TREE_ID ====="
log "  (YOLOE inference period ~2.1s/round, max_age_sec=2.5, expect SUCCESS)"

# Warm-up: DDS discovery between the BT subscriber (created on goal receipt)
# and the YOLOE publisher needs ~500ms+. Without this, the first tick fires
# before any msg lands, evaluate_detection() sees nullptr → FAILURE in
# ~100ms. 6s gives 3+ YOLOE cycles of slack so the Sequence finds both
# topics with cached msgs when it ticks.
log "  warming up 6s for DDS discovery + 3+ YOLOE publish cycles..."
sleep 6

# Sanity: confirm YOLOE has published at least once to BOTH topics in the
# last 10s. If not, the BT will fail deterministically — fail fast.
for topic in \
    /perception/two_stage/camera_head/result \
    /perception/two_stage/camera_arm/result
do
    pub_count=$(docker exec -e LGW_INSTALL="${LGW_INSTALL:-}" "$CONTAINER" bash -c "
        source /opt/ros/humble/setup.bash >/dev/null 2>&1
        source $WS_L4T/l4t_setup.sh >/dev/null 2>&1
        ros2 topic info $topic 2>/dev/null | grep 'Publisher count' | awk '{print \$NF}'
    " 2>/dev/null || echo "?")
    log "  $topic: Publisher count = $pub_count (>=1 expected)"
    if ! [[ "$pub_count" =~ ^[0-9]+$ ]] || [ "$pub_count" -lt 1 ]; then
        err "YOLOE not publishing on $topic — BT tree will fail; aborting"
        exit 1
    fi
done

# Use 30s timeout — gives ~14 inference cycles, plenty of margin for
# first-cycle YOLOE warmup.
ACTION_OUTPUT=$(timeout 30 docker exec -e LGW_INSTALL="${LGW_INSTALL:-}" "$CONTAINER" bash -c "
    source $WS_L4T/l4t_setup.sh 2>/dev/null
    ros2 action send_goal /lgw_bt_execution \
        btcpp_ros2_interfaces/action/ExecuteTree \
        '{target_tree: $BT_TREE_ID}' \
        --feedback 2>&1
" || true)

echo "$ACTION_OUTPUT" | tail -40

if echo "$ACTION_OUTPUT" | grep -qE 'SUCCEEDED.*DetectionConditionTest|status.*SUCCEEDED|GoalReply.*SUCCEEDED'; then
    log "===== Phase 5 PASS: SUCCESS path verified ====="
    PHASE5_RESULT=SUCCESS
else
    err "===== Phase 5 FAIL: SUCCESS not detected ====="
    err "  full output saved to /tmp/test_yoloe_e2e_action.log"
    echo "$ACTION_OUTPUT" > /tmp/test_yoloe_e2e_action.log
    PHASE5_RESULT=FAILURE
fi

# ===== Phase 6: Frequency check =====
log "===== Phase 6: rate check on both result topics ====="
log "  (per runbook: ~0.4-0.5 Hz each, ~2.1s/round)"
timeout 30 docker exec -e LGW_INSTALL="${LGW_INSTALL:-}" "$CONTAINER" bash -c "
    source $WS_L4T/l4t_setup.sh 2>/dev/null
    ros2 topic hz /perception/two_stage/camera_head/result 2>&1
" | tail -10 || true
log "  ---"
timeout 30 docker exec -e LGW_INSTALL="${LGW_INSTALL:-}" "$CONTAINER" bash -c "
    source $WS_L4T/l4t_setup.sh 2>/dev/null
    ros2 topic hz /perception/two_stage/camera_arm/result 2>&1
" | tail -10 || true

# ===== Done =====
log "===== test complete: Phase 5 = $PHASE5_RESULT ====="
log "  logs: ${LOG_FILES[*]}"
log "  cleanup follows via trap..."

# Exit triggers trap.
exit 0