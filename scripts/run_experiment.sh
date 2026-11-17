#!/usr/bin/env bash
# Top-level one-command experiment entry point (Definition of Done, Sec. 24).
#
#   ./scripts/run_experiment.sh configs/E05_gnss_outage_30s.yaml
#
# Accepts an absolute path, a path relative to experiments/, or a bare config
# name resolved against experiments/configs/.
# NOTE: ROS setup.bash reads unset variables (AMENT_TRACE_SETUP_FILES), so
# nounset (`set -u`) must stay off while sourcing it.
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ARG="${1:-configs/E01_nominal.yaml}"; shift || true

resolve() {
  local a="$1"
  if [[ -f "$a" ]]; then echo "$a"; return; fi
  if [[ -f "${REPO_ROOT}/experiments/${a}" ]]; then echo "${REPO_ROOT}/experiments/${a}"; return; fi
  if [[ -f "${REPO_ROOT}/experiments/configs/${a}" ]]; then echo "${REPO_ROOT}/experiments/configs/${a}"; return; fi
  echo "config not found: ${a}" >&2; exit 1
}
CONFIG="$(resolve "${ARG}")"

source /opt/ros/jazzy/setup.bash
if [[ -f "${REPO_ROOT}/ros2_ws/install/setup.bash" ]]; then
  source "${REPO_ROOT}/ros2_ws/install/setup.bash"
else
  echo "[run] workspace not built; building now..."
  "${REPO_ROOT}/scripts/build.sh"
  source "${REPO_ROOT}/ros2_ws/install/setup.bash"
fi
[[ -d /opt/venv ]] && source /opt/venv/bin/activate

python3 "${REPO_ROOT}/experiments/run_experiment.py" "${CONFIG}" "$@"
