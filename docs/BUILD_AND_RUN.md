# Build, Run & Observe — Step‑by‑Step Guide

This guide takes you from a clean checkout to observing simulation results, both
in the **interactive simulator** (no ROS needed) and in the **full ROS 2 +
Gazebo** navigation stack.

> All commands assume you run them **inside the dev container**, where the
> toolchain (ROS 2 Jazzy, Gazebo Harmonic, Clang 19, CMake, Ninja, the Python
> `venv`) is pre‑installed and pinned.

---

## 0. Prerequisites (host machine)

| Requirement | Notes |
|---|---|
| Docker | Engine ≥ 24. `docker --version` |
| VS Code + Dev Containers extension | Recommended entry point |
| X11 server | Only for GUI (Gazebo / RViz). Linux native works out of the box |
| NVIDIA Container Toolkit | *Optional*, only for NVIDIA GPU acceleration |

Allow local X11 clients (once per host login, Linux):

```bash
xhost +local:docker
```

---

## 1. Open the environment

### Option A — VS Code Dev Container (recommended)

1. Open the project folder in VS Code.
2. `F1 → Dev Containers: Reopen in Container`.
3. First launch builds the image and runs `.devcontainer/post-create.sh`
   (scaffolds folders, clones **KF‑GINS**, resolves `rosdep`).

### Option B — Headless via docker‑compose

```bash
docker compose build          # builds the image (first time only)
docker compose run --rm dev bash
```

You now have an interactive shell at `/workspace` with everything sourced.

---

## 2. Build the C++/ROS 2 workspace

```bash
./scripts/build.sh                       # Clang + Ninja + C++23, RelWithDebInfo
source ros2_ws/install/setup.bash        # overlay the freshly built workspace
```

`build.sh` runs `colcon build --symlink-install` and exports
`compile_commands.json` for `clangd`.

### Run the unit tests (GoogleTest)

```bash
cd ros2_ws
colcon test --packages-select ugv_nav_core
colcon test-result --verbose             # 26 tests: math/geodesy, ESKF, sensors, metrics
cd ..
```

---

## 3. Fast path — interactive GNSS/IMU simulator (no ROS/Gazebo)

Best way to *see the algorithm working* immediately.

```bash
./scripts/sim_dashboard.sh               # → http://localhost:8051
```

Open the URL in a browser and:

1. Pick a **route shape** (`figure8`, `circle`, `s_curve`, `square`, `spiral`).
2. Drag the **GNSS noise / rate / outage** and **IMU noise / bias** sliders —
   plots update instantly.
3. Press **▶ Play** to animate the robot along all three routes.

What you observe:

| Colour | Trajectory | Meaning |
|---|---|---|
| 🟢 green | **Ideal** | ground truth |
| 🔴 red (dotted) | **Raw** | IMU‑only dead‑reckoning → drifts |
| 🔵 blue (dashed) | **Corrected** | EKF fusion of GNSS + IMU (the algorithm) |
| 🟡 amber ✕ | GNSS fixes | noisy position measurements |

Infographics update live: **position error**, **heading**, **speed**, **EKF 1σ
uncertainty**, and metric cards (RMSE raw vs. corrected, improvement %, final
error, distance).

Render a static PNG instead of the live server:

```bash
python -m ugv_dashboard.render_preview --out docs/sim_preview.png --trajectory figure8
```

---

## 4. Full stack — Gazebo simulation experiments

Each experiment drives the UGV in Gazebo Harmonic, injects faults, runs the
online ESKF, logs ground truth vs. estimate and computes metrics.

### Run a single scenario

```bash
./scripts/run_experiment.sh configs/E05_gnss_outage_30s.yaml
```

Accepts an absolute path, a path relative to `experiments/`, or a bare config
name. If the workspace is not built yet, the script builds it first.

### Run the whole matrix (E01–E12)

```bash
./scripts/run_all_experiments.sh
```

| ID | Scenario |
|---|---|
| E01 | Nominal |
| E02 / E03 | GNSS noise low / high |
| E04 / E05 / E06 | GNSS outage 10 s / 30 s / 60 s |
| E07 / E08 | IMU noise / bias |
| E09 / E10 | Odometry noise / scale error |
| E11 | Wheel slip |
| E12 | GNSS outage + wheel slip (combined) |

### Watch it live (GUI)

```bash
ros2 launch ugv_bringup bringup.launch.py rviz:=true gui:=true
```

Requires X11 (Section 0). Gazebo shows the robot; RViz shows TF, ground‑truth
odometry, ESKF odometry and the estimated path.

---

## 5. Observe the results

Every run writes a timestamped directory under `experiments/results/`:

```
experiments/results/E05_gnss_outage_30s_YYYYMMDD_HHMMSS/
├── config.yaml          # exact scenario used
├── data.csv             # time‑aligned gt_x/y/yaw vs est_x/y/yaw
├── metrics.json         # ATE, RMSE (pos/vel/yaw), max error, RPE, distance
├── trajectory.png       # static plot
├── *_error.png          # error‑over‑time plots
└── rosbag/              # (optional) recorded topics
```

### Results dashboard

```bash
./scripts/dashboard.sh                                   # newest result, → http://localhost:8050
./scripts/dashboard.sh experiments/results/E05_gnss_outage_30s_YYYYMMDD_HHMMSS   # a specific run
```

The dashboard auto‑refreshes, so it can also be opened **while an experiment is
running** to watch metrics converge live.

### Inspect metrics from the shell

```bash
cat experiments/results/*/metrics.json | python3 -m json.tool
```

---

## 6. Reproduce the KF‑GINS reference (milestone 1)

```bash
./scripts/reproduce_kfgins.sh    # clones, builds (Clang+Ninja) and runs the demo dataset
```

Reference outputs are copied to `datasets/reference/` for comparison.

---

## 7. Troubleshooting

| Symptom | Fix |
|---|---|
| Gazebo/RViz window doesn't open | `xhost +local:docker` on host; check `echo $DISPLAY` inside container |
| `catkin_pkg not found` during build | venv must be created with `--system-site-packages` (already handled in the image) |
| Port 8050/8051 already in use | `./scripts/sim_dashboard.sh --port 8061` or free the port |
| NVIDIA GPU not used | add `"--gpus=all"` to `runArgs` in `.devcontainer/devcontainer.json` and install the NVIDIA Container Toolkit |
| `colcon build` picks GCC | ensure `CC=clang CXX=clang++` (set by `build.sh`) |

---

## Command cheat‑sheet

```bash
./scripts/build.sh                                    # build workspace
source ros2_ws/install/setup.bash                     # source overlay
./scripts/sim_dashboard.sh                            # interactive simulator  :8051
./scripts/run_experiment.sh configs/E05_gnss_outage_30s.yaml
./scripts/run_all_experiments.sh                      # full E01–E12 matrix
./scripts/dashboard.sh                                # results dashboard      :8050
./scripts/reproduce_kfgins.sh                         # KF‑GINS reference
```
