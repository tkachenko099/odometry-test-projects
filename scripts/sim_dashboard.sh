#!/usr/bin/env bash
# Launch the interactive GNSS/IMU navigation simulator dashboard.
# No ROS/Gazebo needed - pure Python simulation. http://localhost:8051
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
[[ -d /opt/venv ]] && source /opt/venv/bin/activate
export PYTHONPATH="${REPO_ROOT}/ros2_ws/src/ugv_dashboard:${PYTHONPATH:-}"
exec python3 -m ugv_dashboard.interactive "$@"
