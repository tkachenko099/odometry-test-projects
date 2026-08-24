#!/usr/bin/env bash
# Run the full experiment matrix E01..E12 sequentially.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
for cfg in "${REPO_ROOT}"/experiments/configs/E*.yaml; do
  echo "==================== $(basename "${cfg}") ===================="
  "${REPO_ROOT}/scripts/run_experiment.sh" "${cfg}" "$@"
done
echo "[all] experiment matrix complete; see experiments/results/"
