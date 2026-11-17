#!/usr/bin/env bash
# Hardware-in-the-loop (HIL) validation of the return-home failsafe against a
# live ArduPilot SITL autopilot via mavros.
#
#   ArduRover SITL  <--MAVLink/TCP 5760-->  mavros  <--ROS-->  rth_node (use_mavros:=true)
#                                                    <--ROS-->  sitl_operator.py
#
# The scripted operator arms the rover, drives it out, cuts the heartbeat and
# then checks that rth_node brings the vehicle back. Exit status 0 = it did.
#
# Requirements:
#   * mavros / mavros_msgs  (in the dev image)
#   * ugv_return_home built with HAVE_MAVROS (automatic when mavros_msgs is present)
#   * ArduPilot Rover SITL  -> image `ugv-nav-stand:sitl` (docker/Dockerfile.sitl),
#     or a local ArduPilot tree with ARDUPILOT_HOME pointing at it
#
# Usage: ./scripts/run_sitl_failsafe.sh [outbound_drive_seconds]
#
# NOTE: ROS setup.bash reads unset variables (AMENT_TRACE_SETUP_FILES), so
# nounset (`set -u`) must stay off while sourcing it.
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DRIVE_TIME="${1:-25}"
LOG_DIR="${SITL_LOG_DIR:-/tmp/ugv_sitl}"
# The SITL binary serves MAVLink on TCP 5760 (serial0); no MAVProxy in between.
FCU_URL="tcp://127.0.0.1:5760"

source /opt/ros/jazzy/setup.bash
if [[ -f "${REPO_ROOT}/ros2_ws/install/setup.bash" ]]; then
  source "${REPO_ROOT}/ros2_ws/install/setup.bash"
else
  echo "[sitl] workspace not built; run ./scripts/build.sh first" >&2
  exit 1
fi
[[ -d /opt/venv ]] && source /opt/venv/bin/activate

if ! ros2 pkg prefix mavros >/dev/null 2>&1; then
  cat >&2 <<'EOF'
[sitl] mavros is not installed. Use the dev image (it installs ros-jazzy-mavros*)
       or: sudo apt-get install -y ros-jazzy-mavros ros-jazzy-mavros-msgs
EOF
  exit 1
fi

# Locate the ArduPilot tree: ARDUPILOT_HOME, or wherever sim_vehicle.py lives.
if [[ -z "${ARDUPILOT_HOME:-}" ]]; then
  SIM_VEHICLE="$(command -v sim_vehicle.py || true)"
  [[ -n "${SIM_VEHICLE}" ]] && ARDUPILOT_HOME="$(cd "$(dirname "${SIM_VEHICLE}")/../.." && pwd)"
fi
ROVER_BIN="${ARDUPILOT_HOME:+${ARDUPILOT_HOME}/build/sitl/bin/ardurover}"
if [[ -z "${ROVER_BIN}" || ! -x "${ROVER_BIN}" ]]; then
  cat >&2 <<'EOF'
[sitl] ArduPilot Rover SITL not found. Either use the add-on image:

  docker build -t ugv-nav-stand:sitl -f docker/Dockerfile.sitl docker
  docker compose run --rm sitl ./scripts/run_sitl_failsafe.sh

or build it once and point ARDUPILOT_HOME at the tree:

  git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
  cd ardupilot && ./waf configure --board sitl && ./waf rover
  export ARDUPILOT_HOME="$PWD"
EOF
  exit 1
fi

mkdir -p "${LOG_DIR}"
PIDS=()
cleanup() {
  for pid in "${PIDS[@]}"; do kill -INT "${pid}" 2>/dev/null || true; done
  sleep 1
  for pid in "${PIDS[@]}"; do kill -KILL "${pid}" 2>/dev/null || true; done
}
trap cleanup EXIT

echo "[sitl] launching ArduRover SITL (${ROVER_BIN})..."
(
  cd "${LOG_DIR}"   # SITL writes eeprom.bin / logs into its working directory
  exec "${ROVER_BIN}" --model rover --speedup 1 -I0 --wipe \
    --defaults "${ARDUPILOT_HOME}/Tools/autotest/default_params/rover.parm" \
    >"${LOG_DIR}/sitl.log" 2>&1
) &
PIDS+=($!)

echo "[sitl] waiting for the autopilot's MAVLink port..."
for _ in $(seq 1 60); do
  if (exec 3<>/dev/tcp/127.0.0.1/5760) 2>/dev/null; then break; fi
  sleep 1
done

echo "[sitl] launching mavros (fcu_url=${FCU_URL})..."
# system_id 255 = ArduPilot's default SYSID_MYGCS: RC overrides from any other
# MAVLink system id are silently ignored.
ros2 run mavros mavros_node --ros-args -p fcu_url:="${FCU_URL}" -p system_id:=255 \
  >"${LOG_DIR}/mavros.log" 2>&1 &
PIDS+=($!)

echo "[sitl] launching rth_node (mavros backend)..."
ros2 run ugv_return_home rth_node --ros-args \
  -p use_mavros:=true \
  -p position_topic:=/mavros/global_position/global \
  -p heartbeat_topic:=/operator/heartbeat \
  -p link_timeout:=3.0 -p min_record_distance:=2.0 -p arrival_radius:=3.0 \
  >"${LOG_DIR}/rth.log" 2>&1 &
PIDS+=($!)

echo "[sitl] running the scripted operator (drive ${DRIVE_TIME}s, then drop the link)..."
echo "[sitl] logs: ${LOG_DIR}/{sitl,mavros,rth}.log"
status=0
python3 "${REPO_ROOT}/scripts/sitl_operator.py" --drive-time "${DRIVE_TIME}" || status=$?

echo "[sitl] autopilot commands issued by rth_node:"
grep -E "\[mavros\]|mission mode" "${LOG_DIR}/rth.log" | sed 's/^\[[A-Z]*\] \[[0-9.]*\] //' || true
exit "${status}"
