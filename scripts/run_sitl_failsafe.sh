#!/usr/bin/env bash
# Hardware-in-the-loop (HIL) validation of the return-home failsafe against a
# live ArduPilot SITL autopilot via mavros.
#
#   ArduRover SITL  <--MAVLink/UDP-->  mavros  <--ROS-->  rth_node (use_mavros:=true)
#
# Requirements (installed into the image on rebuild, except ArduPilot SITL):
#   * mavros / mavros_msgs         (Dockerfile: ros-jazzy-mavros*)
#   * ugv_return_home built with HAVE_MAVROS (auto when mavros_msgs is present)
#   * ArduPilot SITL (sim_vehicle.py)  -> see install hint below
#
# Usage: ./scripts/run_sitl_failsafe.sh [drop_after_seconds]
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DROP_AFTER="${1:-25}"
FCU_URL="udp://:14551@127.0.0.1:14555"

source /opt/ros/jazzy/setup.bash
if [[ -f "${REPO_ROOT}/ros2_ws/install/setup.bash" ]]; then
  source "${REPO_ROOT}/ros2_ws/install/setup.bash"
else
  echo "[sitl] workspace not built; run ./scripts/build.sh first" >&2
  exit 1
fi

if ! ros2 pkg prefix mavros >/dev/null 2>&1; then
  cat >&2 <<'EOF'
[sitl] mavros is not installed. Rebuild the dev image (Dockerfile now installs
       ros-jazzy-mavros*) or: sudo apt-get install -y ros-jazzy-mavros ros-jazzy-mavros-msgs
EOF
  exit 1
fi

SIM_VEHICLE="$(command -v sim_vehicle.py || true)"
if [[ -z "${SIM_VEHICLE}" ]]; then
  cat >&2 <<'EOF'
[sitl] ArduPilot SITL (sim_vehicle.py) not found. Install once:

  git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
  cd ardupilot && Tools/environment_install/install-prereqs-ubuntu.sh -y
  . ~/.profile && ./waf configure --board sitl && ./waf rover
  export PATH="$PWD/Tools/autotest:$PATH"

Then re-run this script.
EOF
  exit 1
fi

cleanup() { pkill -P $$ >/dev/null 2>&1 || true; }
trap cleanup EXIT

echo "[sitl] launching ArduRover SITL..."
"${SIM_VEHICLE}" -v Rover -f rover --no-mavproxy --out=udp:127.0.0.1:14551 >/tmp/sitl.log 2>&1 &
sleep 15

echo "[sitl] launching mavros (fcu_url=${FCU_URL})..."
ros2 run mavros mavros_node --ros-args -p fcu_url:="${FCU_URL}" >/tmp/mavros.log 2>&1 &
sleep 10

echo "[sitl] launching rth_node (mavros backend)..."
ros2 run ugv_return_home rth_node --ros-args \
  -p use_mavros:=true \
  -p position_topic:=/mavros/global_position/global \
  -p heartbeat_topic:=/operator/heartbeat \
  -p link_timeout:=3.0 -p min_record_distance:=2.0 -p arrival_radius:=3.0 >/tmp/rth.log 2>&1 &
sleep 3

echo "[sitl] operator heartbeat for ${DROP_AFTER}s, then dropping the link..."
ros2 run ugv_bringup operator_link --ros-args \
  -p drop_after:="${DROP_AFTER}" >/tmp/operator.log 2>&1 &

echo "[sitl] running. Tail logs: /tmp/{sitl,mavros,rth,operator}.log"
echo "[sitl] after the link drops, rth_node should SET_MODE GUIDED/AUTO and push the mission."
echo "[sitl] Press Ctrl-C to stop."
wait
