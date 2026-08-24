# syntax=docker/dockerfile:1.7
###############################################################################
# UGV GNSS/INS Navigation Stand — reproducible development image
#
# Builder-agnostic: works with both the classic Docker builder and BuildKit
# (no --mount / heredoc features required).
#
# Stack (pinned by base image / apt where possible):
#   * Ubuntu 24.04 (Noble)      — Tier-1 platform for ROS 2 Jazzy
#   * ROS 2 Jazzy Jalisco       — desktop (RViz2, demos, rosbag2, tf2, ...)
#   * Gazebo Harmonic           — via ros-jazzy-ros-gz vendor packages
#   * Clang 19 + libc++ / libstdc++, CMake (Kitware), Ninja
#     -> C++20 / C++23 with named modules, concepts, ranges, <format>, etc.
#   * GoogleTest / GoogleMock   — unit + fixture testing
#   * Eigen3, abseil, yaml-cpp  — KF-GINS reference build dependencies
#   * robot_localization        — ROS 2 EKF/UKF baseline
#   * Python 3.12 + Dash/Plotly — analysis dashboard
###############################################################################
FROM osrf/ros:jazzy-desktop AS base

# ---- Build-time configuration ------------------------------------------------
ARG LLVM_VERSION=19
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000
ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ENV LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    ROS_DISTRO=jazzy \
    GZ_VERSION=harmonic \
    TZ=Etc/UTC

# ---- Base OS tooling ---------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        locales ca-certificates curl wget gnupg lsb-release software-properties-common \
        build-essential pkg-config git git-lfs openssh-client \
        ninja-build gdb lldb valgrind \
        python3 python3-pip python3-venv python3-dev \
        libeigen3-dev libyaml-cpp-dev \
        bash-completion sudo vim less zip unzip \
    && locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# ---- Modern CMake from the official Kitware APT repository -------------------
RUN curl -fsSL https://apt.kitware.com/keys/kitware-archive-latest.asc \
        | gpg --dearmor -o /usr/share/keyrings/kitware-archive-keyring.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ noble main" \
        > /etc/apt/sources.list.d/kitware.list \
    && apt-get update && apt-get install -y --no-install-recommends cmake cmake-curses-gui \
    && rm -rf /var/lib/apt/lists/*

# ---- Clang / LLVM toolchain (C++20 modules + C++23) --------------------------
# libc++ is used for standalone modules experiments; ROS nodes stay on
# libstdc++ (GCC ABI) so they remain link-compatible with the Jazzy debs.
RUN curl -fsSL https://apt.llvm.org/llvm-snapshot.gpg.key \
        | gpg --dearmor -o /usr/share/keyrings/llvm-archive-keyring.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/llvm-archive-keyring.gpg] http://apt.llvm.org/noble/ llvm-toolchain-noble-${LLVM_VERSION} main" \
        > /etc/apt/sources.list.d/llvm.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        clang-${LLVM_VERSION} clangd-${LLVM_VERSION} clang-tidy-${LLVM_VERSION} \
        clang-format-${LLVM_VERSION} clang-tools-${LLVM_VERSION} lld-${LLVM_VERSION} lldb-${LLVM_VERSION} \
        llvm-${LLVM_VERSION} llvm-${LLVM_VERSION}-dev \
        libc++-${LLVM_VERSION}-dev libc++abi-${LLVM_VERSION}-dev libomp-${LLVM_VERSION}-dev \
    && update-alternatives --install /usr/bin/clang        clang        /usr/bin/clang-${LLVM_VERSION}        100 \
    && update-alternatives --install /usr/bin/clang++      clang++      /usr/bin/clang++-${LLVM_VERSION}      100 \
    && update-alternatives --install /usr/bin/clangd       clangd       /usr/bin/clangd-${LLVM_VERSION}       100 \
    && update-alternatives --install /usr/bin/clang-tidy   clang-tidy   /usr/bin/clang-tidy-${LLVM_VERSION}   100 \
    && update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-${LLVM_VERSION} 100 \
    && update-alternatives --install /usr/bin/lld          lld          /usr/bin/lld-${LLVM_VERSION}          100 \
    && update-alternatives --install /usr/bin/ld.lld       ld.lld       /usr/bin/ld.lld-${LLVM_VERSION}       100 \
    && rm -rf /var/lib/apt/lists/*

# ---- GoogleTest / GoogleMock, Eigen, abseil (KF-GINS deps) -------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        libgtest-dev libgmock-dev \
        libabsl-dev libbenchmark-dev libspdlog-dev nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

# ---- Gazebo Harmonic + ROS<->GZ bridge + navigation stack --------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-gz \
        ros-jazzy-ros-gz-sim \
        ros-jazzy-ros-gz-bridge \
        ros-jazzy-ros-gz-image \
        ros-jazzy-ros-gz-interfaces \
        ros-jazzy-robot-localization \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-joint-state-publisher \
        ros-jazzy-xacro \
        ros-jazzy-tf2-ros ros-jazzy-tf2-tools ros-jazzy-tf-transformations \
        ros-jazzy-rmw-cyclonedds-cpp \
        ros-jazzy-nav-msgs ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs \
        ros-jazzy-rviz2 \
        ros-dev-tools python3-colcon-common-extensions python3-colcon-mixin \
        python3-rosdep python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# ---- Non-root developer user -------------------------------------------------
# The Noble base already ships a uid=1000 "ubuntu" account; rename/relocate it
# to the requested USERNAME so bind-mounted files keep host ownership.
RUN if getent passwd "${USER_UID}" >/dev/null; then \
        existing="$(getent passwd ${USER_UID} | cut -d: -f1)"; \
        [ "${existing}" != "${USERNAME}" ] && usermod -l "${USERNAME}" -d "/home/${USERNAME}" -m "${existing}" || true; \
        groupmod -n "${USERNAME}" "$(getent group ${USER_GID} | cut -d: -f1)" 2>/dev/null || true; \
    else \
        groupadd --gid "${USER_GID}" "${USERNAME}"; \
        useradd  --uid "${USER_UID}" --gid "${USER_GID}" -m "${USERNAME}"; \
    fi \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# ---- Python dashboard / experiment tooling -----------------------------------
# PEP 668: Noble's system Python is externally managed, so use an isolated venv
# that every shell auto-activates.
# --system-site-packages so the venv can still import ROS's Python modules
# (catkin_pkg, ament_package, colcon, empy, ...) needed by ament/colcon builds,
# while our pinned dashboard deps layer on top.
ENV VENV=/opt/venv
RUN python3 -m venv --system-site-packages "${VENV}" \
    && "${VENV}/bin/pip" install --no-cache-dir --upgrade pip setuptools wheel
COPY requirements.txt /tmp/requirements.txt
RUN "${VENV}/bin/pip" install --no-cache-dir -r /tmp/requirements.txt

# ---- rosdep bootstrap --------------------------------------------------------
RUN rosdep update --rosdistro "${ROS_DISTRO}" || true

# ---- Environment defaults for interactive shells -----------------------------
ENV CC=clang \
    CXX=clang++ \
    CMAKE_GENERATOR=Ninja \
    CMAKE_EXPORT_COMPILE_COMMANDS=ON \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    GZ_SIM_RESOURCE_PATH=/workspace/ros2_ws/install/share \
    PATH="/opt/venv/bin:${PATH}"

COPY docker/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# Source ROS + venv automatically for the dev user (no heredoc: classic-builder safe).
RUN printf '%s\n' \
      '' \
      '# ---- UGV navigation stand shell setup ----' \
      'source /opt/ros/jazzy/setup.bash' \
      '[ -f /workspace/ros2_ws/install/setup.bash ] && source /workspace/ros2_ws/install/setup.bash' \
      '[ -d /opt/venv ] && source /opt/venv/bin/activate' \
      'export CC=clang CXX=clang++ CMAKE_GENERATOR=Ninja' \
      >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /workspace

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
