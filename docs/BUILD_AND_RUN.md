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
`compile_commands.json` for `clangd`. All C++ packages compile with
`-Wall -Wextra -Wpedantic -Wshadow -Wconversion`; add `-DUGV_WERROR=ON` to turn
warnings into errors, as CI does:

```bash
./scripts/build.sh -DUGV_WERROR=ON
```

### Run the unit tests (GoogleTest)

```bash
cd ros2_ws
colcon test --packages-select ugv_nav_core
colcon test-result --verbose             # 29 tests: math/geodesy, ESKF, sensors, metrics
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

Useful flags (passed through to `experiments/run_experiment.py`):

| Flag | Effect |
|---|---|
| `--headless` | Gazebo server only, off‑screen rendering — for containers/CI without a display |
| `--no-bag` | skip the rosbag recording |
| `--max-position-rmse M` | exit non‑zero unless the run's position RMSE is ≤ `M` metres (regression gate) |

```bash
./scripts/run_experiment.sh configs/E01_nominal.yaml --headless --no-bag --max-position-rmse 1.0
```

**Estimator configuration** lives in `ugv_localization/config/eskf.yaml`
(override with `eskf_params_file:=…` on `bringup.launch.py`). The heading
parameters matter most: `init_yaw_deg` is the start heading as a ROS/ENU yaw
(0 = East, the Gazebo spawn pose) and `init_yaw_std_deg` how well it is known.
Set the std to 45° or more to declare the heading unknown — the node then aligns
to the GNSS course over ground after `align_distance` metres of travel.

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

## 7. Return-home-on-link-loss failsafe (`ugv_return_home`)

Onboard C++23 module that records the driven trail and, on operator-datalink loss,
backtracks the vehicle home via an ArduPilot autopilot over MAVLink.

### Standalone demo (no ROS/Gazebo)

```bash
source ros2_ws/install/setup.bash
ros2 run ugv_return_home rth_demo            # trace only
ros2 run ugv_return_home rth_demo run.json   # also write a JSON run log for plotting
```

Expected trace: the vehicle drives an L-route (recording breadcrumbs), the link goes
silent, the failsafe trips, a simplified backtrack mission is "uploaded" over a
simulated MAVLink, and the vehicle is driven home — ending in `RESULT: vehicle
returned home successfully.`

### Route + metrics figure

```bash
./scripts/rth_report.sh docs/rth_metrics.png   # runs rth_demo + renders the figure
```

Produces `docs/rth_metrics.png`: the route map (recorded trail vs. uploaded backtrack
vs. actual driven path) plus distance-to-home, speed, cross-track error, and the
mission-state timeline. Requires the Python deps (`plotly`, `kaleido`, `numpy`).

### ROS 2 node (wired to mavros topics)

```bash
ros2 launch ugv_return_home rth.launch.py           # params: config/rth.yaml
# In another shell, publish an operator heartbeat; stop it to trip the failsafe:
ros2 topic pub -r 2 /operator/heartbeat std_msgs/msg/Empty {}
# Watch the backtrack mission (metric ENU path for RViz), mode and mission state:
ros2 topic echo /rth/return_path
ros2 topic echo /rth/autopilot_mode      # commanded autopilot mode (GUIDED/AUTO/HOLD)
ros2 topic echo /rth/mission_state       # IDLE/RECORDING/RETURNING/HOME
ros2 topic echo /rth/recorded_path       # recorded breadcrumb trail (RViz)
# Or force a manual return at any time:
ros2 service call /rth/force std_srvs/srv/Trigger {}
# HOME is a terminal state (the vehicle is held there). Re-arm for a new sortie -
# the next position fix becomes the new home:
ros2 service call /rth/reset std_srvs/srv/Trigger {}
```

Key parameters (`config/rth.yaml`): `link_timeout`, `min_record_distance`,
`arrival_radius`, `simplify`/`simplify_epsilon`, `resume_on_recovery`,
`position_topic` (default `/mavros/global_position/global`), `heartbeat_topic`,
`use_mavros` (select the real MAVLink backend).

### Inside the full Gazebo stack

The failsafe is wired into `ugv_bringup` behind a toggle. It records the robot's
`/gps/fix`, and an `operator_link` node publishes a heartbeat that stops after
`rth_drop_after` seconds to simulate a comms loss:

```bash
ros2 launch ugv_bringup bringup.launch.py return_home:=true rviz:=true rth_drop_after:=30.0
```

RViz shows the recorded trail (green, `/rth/recorded_path`) and the backtrack path
(blue, `/rth/return_path`). When the heartbeat stops, `mission_state` flips to
`RETURNING`, `trajectory_commander` yields `/cmd_vel`, and the `return_follower`
(pure-pursuit) drives the robot back along its route until `HOME`.

### Hardware-in-the-loop against ArduPilot SITL

The real MAVLink path is exercised end-to-end against ArduPilot Rover SITL. The
autopilot lives in an add-on image (`docker/Dockerfile.sitl`, ArduPilot
Rover 4.6.3 on top of the dev image; several GB, ~15 min to build):

```bash
docker compose build dev sitl
docker compose run --rm sitl bash -c './scripts/build.sh && ./scripts/run_sitl_failsafe.sh 25'
```

`run_sitl_failsafe.sh` starts the SITL rover, mavros (TCP 5760, MAVLink system
id 255) and `rth_node` with `use_mavros:=true`, then runs a scripted operator
(`scripts/sitl_operator.py`) that arms the rover, drives it out for the given
number of seconds under RC override, cuts the operator heartbeat and watches the
result. On link loss the node issues `SET_MODE GUIDED`, clears and uploads the
backtrack mission, and switches to `AUTO` once the autopilot has accepted it;
on arrival it commands `HOLD`. The script exits 0 only if the vehicle ends
within 5 m of the launch point with the mission in `HOME`:

```
[operator] drove out to 163.3 m from the launch point (mission=RECORDING)
[operator] DATALINK LOST: heartbeat stopped, sticks released
[rth_node]: [mavros] mission accepted (22 items)
[operator] final: mission=HOME mode=HOLD distance home=1.8 m (max 185.3 m)
[operator] RESULT: PASS - vehicle backtracked home.
```

Logs go to `/tmp/ugv_sitl/{sitl,mavros,rth}.log` (override with `SITL_LOG_DIR`).
Outside the image, build ArduPilot yourself and point `ARDUPILOT_HOME` at the
tree; the script prints the steps if it cannot find the `ardurover` binary.

### Tests

```bash
cd ros2_ws
colcon test --packages-select ugv_return_home
colcon test-result --test-result-base build/ugv_return_home --verbose   # 26 tests
cd ..
```

---

## 8. Troubleshooting

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
ros2 run ugv_return_home rth_demo                     # return-home failsafe demo
ros2 launch ugv_return_home rth.launch.py             # return-home failsafe node
ros2 launch ugv_bringup bringup.launch.py return_home:=true rviz:=true   # failsafe in Gazebo
./scripts/run_sitl_failsafe.sh                        # HIL: ArduPilot SITL + mavros
./scripts/rth_report.sh docs/rth_metrics.png          # return-home route + metrics figure
```
