#!/usr/bin/env bash
# Milestone 1: build the reference KF-GINS and run its demo dataset (Sec. 12).
# KF-GINS is expected under third_party/KF-GINS (cloned by the devcontainer
# bootstrap; cloned here on demand otherwise).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KFGINS="${REPO_ROOT}/third_party/KF-GINS"

if [[ ! -d "${KFGINS}/.git" ]]; then
  echo "[kfgins] cloning reference implementation..."
  git clone --depth 1 https://github.com/i2Nav-WHU/KF-GINS.git "${KFGINS}"
fi

echo "[kfgins] configuring & building (Clang + Ninja, Release)..."
cmake -S "${KFGINS}" -B "${KFGINS}/build" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++
cmake --build "${KFGINS}/build" -j"$(nproc)"

BIN="$(find "${KFGINS}" -maxdepth 3 -type f -name 'KF-GINS' -executable | head -n1)"
CONF="${KFGINS}/dataset/kf-gins.yaml"
if [[ -x "${BIN}" && -f "${CONF}" ]]; then
  echo "[kfgins] running demo dataset..."
  ( cd "${KFGINS}" && "${BIN}" "${CONF}" )
  mkdir -p "${REPO_ROOT}/datasets/reference"
  cp -v "${KFGINS}"/dataset/*_result*.nav "${REPO_ROOT}/datasets/reference/" 2>/dev/null || true
  echo "[kfgins] demo complete; reference outputs copied to datasets/reference/"
else
  echo "[kfgins] built, but demo config/binary not found where expected:" >&2
  echo "         bin=${BIN} conf=${CONF}" >&2
  exit 1
fi
