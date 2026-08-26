#!/usr/bin/env bash
# Generate the return-home failsafe report: run the standalone demo to emit a
# JSON run log, then render the route + metrics figure (PNG).
#
#   scripts/rth_report.sh [output.png]
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-${REPO_ROOT}/docs/rth_metrics.png}"
RUN_JSON="$(mktemp --suffix=.json)"
trap 'rm -f "${RUN_JSON}"' EXIT

# Locate the rth_demo binary (installed or built).
DEMO="$(command -v rth_demo || true)"
if [[ -z "${DEMO}" ]]; then
    DEMO="$(find "${REPO_ROOT}/ros2_ws" -type f -name rth_demo -path '*install*' 2>/dev/null | head -n1)"
fi
if [[ -z "${DEMO}" ]]; then
    echo "rth_demo not found; build ugv_return_home first (colcon build)." >&2
    exit 1
fi

echo ">> running ${DEMO}"
"${DEMO}" "${RUN_JSON}"

[[ -d /opt/venv ]] && source /opt/venv/bin/activate
export PYTHONPATH="${REPO_ROOT}/ros2_ws/src/ugv_dashboard:${PYTHONPATH:-}"
echo ">> rendering ${OUT}"
python3 -m ugv_dashboard.rth_plots --in "${RUN_JSON}" --out "${OUT}"
