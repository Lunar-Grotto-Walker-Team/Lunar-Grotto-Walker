#!/usr/bin/env bash
# shellcheck source=../_common.sh
source "$(dirname "${BASH_SOURCE[0]}")/../_common.sh"
# Guided REAL-ARM verification for HOLD recovery (spec 2026-11-03 §15.1).
#
#   bash scripts/bench_hold_recovery_real.sh <baseline|target_lost|release|saturated>
#
# Safety model:
#   - refuses unless agx_arm_ctrl is running and /feedback/joint_states is live;
#   - refuses if the mock/full stack is already running or if anything is
#     publishing on /perception/tracks/unified (YOLOE/tracker must be stopped);
#   - asks for an explicit YES before EACH motion (takeover preposition, then
#     the recovery move);
#   - never starts the hardware driver and never publishes static TFs;
#   - conservative Servo caps (v_max 0.20, w_max 0.5, scaling 0.5);
#   - recovery speed stays at recovery_velocity_scaling (0.10).
#
# The fake target is the ONLY publisher on /perception/tracks/unified while
# this script runs. Targets are in arm1/base_link (arm base), x forward.
set -euo pipefail

SCENARIO="${1:-}"
case "${SCENARIO}" in
  baseline|target_lost|release|saturated) ;;
  *)
    echo "usage: $0 <baseline|target_lost|release|saturated>" >&2
    exit 2
    ;;
esac

CT="self_l4t"
WS="${WS_FORK_CT}"
LOG="/tmp/real_recovery_${SCENARIO}_$(date +%H%M%S).log"
TARGET_LOG="/tmp/real_recovery_target_$(date +%H%M%S).log"

confirm() {
  local prompt="$1" answer
  read -r -p "${prompt} [YES/no]: " answer
  [ "${answer}" = "YES" ]
}

ODOM_PID=""
cleanup() {
  echo "[real] cleanup: release + stop the bench stack (driver untouched)"
  if [ -n "${ODOM_PID}" ]; then
    docker exec "${CT}" bash -c "kill -15 ${ODOM_PID} 2>/dev/null" || true
  fi
  docker exec -w "${WS}" "${CT}" bash -lc "
    source /my_ws/l4t/.l4t_env.sh
    timeout 5 ros2 service call /lgw/servo/release std_srvs/srv/Trigger >/dev/null 2>&1 || true" || true
  docker exec -w "${WS}" "${CT}" bash tools/lgw/bench/bench_cleanup.sh >/dev/null 2>&1 || true
  echo "[real] stack log: ${LOG}"
}
trap cleanup EXIT

echo "[real] 1/6 pre-flight"
if ! docker exec "${CT}" bash -c "pgrep -f '[a]gx_arm_ctrl_single' >/dev/null"; then
  echo "[real] ABORT: agx_arm_ctrl_single is not running. Start the driver first:"
  echo "  ros2 launch agx_arm_ctrl start_single_agx_arm.launch.py \\"
  echo "    effector_type:=agx_gripper tcp_offset:='[0.0, 0.0, 0.12, 0.0, 0.0, 0.0]' \\"
  echo "    speed_percent:=80 fast_mode:=false"
  exit 1
fi
if docker exec "${CT}" bash -c "pgrep -f '[l]gw_full.launch.py' >/dev/null"; then
  echo "[real] ABORT: lgw_full.launch.py is already running. Stop it by PID first."
  exit 1
fi
if ! docker exec "${CT}" bash -lc "
    source /my_ws/l4t/.l4t_env.sh
    timeout 4 ros2 topic echo /feedback/joint_states --once >/dev/null 2>&1"; then
  echo "[real] ABORT: /feedback/joint_states is not publishing."
  exit 1
fi
PUBLISHERS=$(docker exec "${CT}" bash -lc "
  source /my_ws/l4t/.l4t_env.sh
  timeout 6 ros2 topic info /perception/tracks/unified -v 2>/dev/null \
    | awk '/Publisher count/{print \$3; exit}'" || true)
if [ -n "${PUBLISHERS}" ] && [ "${PUBLISHERS}" != "0" ]; then
  echo "[real] ABORT: ${PUBLISHERS} publisher(s) on /perception/tracks/unified."
  echo "[real] Stop YOLOE / the 3D tracker first; only the fake target may publish."
  exit 1
fi

echo "[real] 2/6 launching the real stack (follow:=true) -> ${LOG}"
PARAMS="follow:=true use_bt:=false use_rviz:=false lookat_mode:=pinned_posture \
  servo_v_max:=0.20 servo_w_max:=0.5 servo_velocity_scaling:=0.5"
if [ "${SCENARIO}" = "baseline" ]; then
  PARAMS="${PARAMS} hold_recovery_enable:=false recovery_allow_target_lost:=false"
else
  PARAMS="${PARAMS} hold_recovery_enable:=true recovery_allow_target_lost:=true"
fi
docker exec -d -w "${WS}" "${CT}" bash -lc "
  source /my_ws/l4t/.l4t_env.sh
  ros2 launch lgw_bt_planner lgw_full.launch.py ${PARAMS} > '${LOG}' 2>&1"

if [ "${SCENARIO}" = "saturated" ]; then
  TX=1.10; TY=0.0; TZ=0.45
else
  TX=0.75; TY=0.0; TZ=-0.37
fi
echo "[real] 3/6 odom -> base_link bridge (the tracker normally provides it)"
docker exec -d -w "${WS}" "${CT}" bash -lc "
  source /my_ws/l4t/.l4t_env.sh
  python3 scripts/odom_tf_bridge.py > /tmp/real_recovery_odom_tf.log 2>&1"
sleep 1.5
ODOM_PID=$(docker exec "${CT}" bash -c "pgrep -f '^python3 scripts/odom_tf_bridge.py' | head -1" || true)
if [ -z "${ODOM_PID}" ]; then
  echo "[real] ABORT: odom_tf_bridge did not start."
  exit 1
fi
echo "[real]   odom bridge pid ${ODOM_PID}"

echo "[real] 3/6 fake target A at arm_base (${TX}, ${TY}, ${TZ})"
docker exec -d -w "${WS}" "${CT}" bash -lc "
  source /my_ws/l4t/.l4t_env.sh
  python3 scripts/fake_track_publisher.py --x ${TX} --y ${TY} --z ${TZ} \
    --frame arm_base > '${TARGET_LOG}' 2>&1"
sleep 1.5
TARGET_PID=$(docker exec "${CT}" bash -c "pgrep -f '^python3 scripts/fake_track_publisher.py' | head -1" || true)
if [ -z "${TARGET_PID}" ]; then
  echo "[real] ABORT: fake target did not start."
  exit 1
fi
echo "[real]   target pid ${TARGET_PID}"

echo
echo "================ SAFETY CHECK #1: takeover ================"
echo "The node will PRE-POSITION the arm with MoveGroup, then start visual"
echo "servoing. Two motions can occur. Keep the e-stop in hand."
confirm "Robot standing, arm workspace clear, e-stop ready?" || { echo "[real] aborted"; exit 1; }

echo "[real] 4/6 prepare (takeover + preposition + settle)"
docker exec -w "${WS}" "${CT}" bash -lc "
  source /my_ws/l4t/.l4t_env.sh
  timeout 120 python3 tools/lgw/bench/real_recovery_run.py \
    --scenario ${SCENARIO} --phase prepare --joint-topic /feedback/joint_states"

echo
echo "================ SAFETY CHECK #2: recovery ================"
case "${SCENARIO}" in
  baseline)
    echo "The target stream will stop. Recovery is DISABLED; the arm must"
    echo "NOT move. This proves the baseline."
    ;;
  target_lost)
    echo "The target will switch to B and the stream will stop. The node"
    echo "will MOVE the arm to B's posture family."
    ;;
  release)
    echo "The target will switch to B and the stream will stop. As soon as"
    echo "an active recovery goal is seen the script releases; the bounded"
    echo "move may finish at the family pose, but Servo must never unpause."
    ;;
  saturated)
    echo "The target is unreachable. The arm may MOVE toward a best-effort"
    echo "pose, or the node may refuse and stay in HOLD."
    ;;
esac
confirm "Arm workspace clear and you are watching the arm?" || { echo "[real] aborted"; exit 1; }

echo "[real] 5/6 trigger"
MONITOR=8
[ "${SCENARIO}" = "saturated" ] && MONITOR=10
set +e
docker exec -w "${WS}" "${CT}" bash -lc "
  source /my_ws/l4t/.l4t_env.sh
  timeout 120 python3 tools/lgw/bench/real_recovery_run.py \
    --scenario ${SCENARIO} --phase trigger --joint-topic /feedback/joint_states \
    --target-pid ${TARGET_PID} --monitor-seconds ${MONITOR}"
RC=$?
set -e

echo "[real] 6/6 done (exit ${RC}); stack log ${LOG}"
exit "${RC}"
