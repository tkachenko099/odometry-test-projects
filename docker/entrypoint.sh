#!/usr/bin/env bash
# Container entrypoint: layer ROS 2, the local overlay and the Python venv onto
# the environment before handing control to the requested command.
#
# NOTE: ROS setup.bash is not `set -u` safe, so we deliberately avoid nounset.
set -e
set -o pipefail

source /opt/ros/"${ROS_DISTRO:-jazzy}"/setup.bash

if [[ -f /workspace/ros2_ws/install/setup.bash ]]; then
    source /workspace/ros2_ws/install/setup.bash
fi

if [[ -d /opt/venv ]]; then
    # shellcheck disable=SC1091
    source /opt/venv/bin/activate
fi

exec "$@"
