#!/usr/bin/env bash
# Launch the Plotly/Dash dashboard on http://localhost:8050
# Usage: ./scripts/dashboard.sh [RESULT_DIR_OR_RESULTS_ROOT]
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
[[ -d /opt/venv ]] && source /opt/venv/bin/activate
RESULTS="${1:-${REPO_ROOT}/experiments/results}"
python3 "${REPO_ROOT}/ros2_ws/src/ugv_dashboard/ugv_dashboard/app.py" --results "${RESULTS}"
