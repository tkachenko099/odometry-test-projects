#!/usr/bin/env bash
###############################################################################
# One-shot workspace bootstrap (runs on devcontainer creation).
#   1. Scaffold the project tree defined in "План диплома.pdf" (Section 4).
#   2. Clone the KF-GINS reference implementation into third_party/ (kept
#      pristine, Section 2).
#   3. Resolve ROS 2 dependencies via rosdep.
# Idempotent: safe to re-run.
###############################################################################
# NOTE: ROS setup.bash is not `set -u` safe, so we avoid nounset here.
set -eo pipefail

WS_ROOT="/workspace"
KFGINS_URL="https://github.com/i2Nav-WHU/KF-GINS.git"
KFGINS_DIR="${WS_ROOT}/third_party/KF-GINS"

log() { printf '\033[1;32m[post-create]\033[0m %s\n' "$*"; }

cd "${WS_ROOT}"

# --- 1. Scaffold the reproducible project tree -------------------------------
log "Scaffolding project directories..."
mkdir -p \
  docs \
  third_party \
  ros2_ws/src/{ugv_description,ugv_gazebo,ugv_bringup,ugv_sensor_processing,ugv_fault_injector,ugv_localization,ugv_metrics,ugv_dashboard} \
  experiments/{configs,results,scripts} \
  datasets/{reference,simulated,rosbags} \
  scripts

# Keep otherwise-empty result/data directories under version control.
find datasets experiments/results -type d -empty -exec touch {}/.gitkeep \; 2>/dev/null || true

# --- 2. Clone the KF-GINS reference (do not modify) --------------------------
if [[ ! -d "${KFGINS_DIR}/.git" ]]; then
  log "Cloning KF-GINS reference into third_party/..."
  git clone --depth 1 "${KFGINS_URL}" "${KFGINS_DIR}"
else
  log "KF-GINS already present — skipping clone."
fi

# --- 3. Resolve ROS 2 dependencies -------------------------------------------
source /opt/ros/jazzy/setup.bash
if [[ -n "$(find ros2_ws/src -mindepth 2 -name package.xml 2>/dev/null)" ]]; then
  log "Running rosdep install for ros2_ws/src..."
  sudo rosdep init 2>/dev/null || true
  rosdep update --rosdistro jazzy || true
  rosdep install --from-paths ros2_ws/src --ignore-src -r -y || true
else
  log "No ROS packages defined yet — skipping rosdep install."
fi

log "Bootstrap complete."
log "KF-GINS reference build:"
log "  cd third_party/KF-GINS && cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release && cmake --build build -j"
log "  ./bin/KF-GINS ./dataset/kf-gins.yaml"
log "ROS 2 workspace build:"
log "  cd ros2_ws && colcon build --symlink-install --cmake-args -GNinja -DCMAKE_CXX_STANDARD=23"
