#!/usr/bin/env bash
# Build the ROS 2 workspace with Clang + Ninja and C++23.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash

cd "${REPO_ROOT}/ros2_ws"
export CC=clang CXX=clang++

colcon build \
  --symlink-install \
  --cmake-args \
    -G Ninja \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DCMAKE_CXX_STANDARD=23 \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  "$@"

echo "[build] done. Source it with: source ${REPO_ROOT}/ros2_ws/install/setup.bash"
