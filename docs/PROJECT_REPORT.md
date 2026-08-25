# NavStand — Analytic Project Report

**A Reproducible Simulation & Evaluation Stand for Odometry and GNSS/INS
Navigation of Ground Robotic Complexes**

| | |
|---|---|
| **Domain** | Mobile‑robot state estimation / sensor fusion |
| **Stack** | ROS 2 Jazzy · Gazebo Harmonic · C++20/23 · Python (Dash/Plotly) |
| **Toolchain** | Clang 19 · CMake · Ninja · GoogleTest · Docker / Dev Containers |
| **Reference** | KF‑GINS (i2Nav‑WHU) |
| **Scope** | Stage 1 — classical estimation (ML explicitly deferred to Stage 2) |

---

## 1. Abstract

NavStand is a fully containerised, reproducible software stand for **modelling,
researching and visualising** the odometry and navigation of a ground robotic
complex (GRC/UGV). It couples a physically‑simulated robot (Gazebo Harmonic)
carrying **IMU**, **GNSS** and **wheel‑odometry** sensors with a modern C++23
estimation core implementing an **Error‑State Kalman Filter (ESKF)** for
loosely‑coupled GNSS/INS fusion. A deterministic **fault injector** corrupts the
sensor streams with configurable noise, bias, outages and wheel slip; a
**metrics** node computes navigation‑accuracy statistics against ground truth;
and two **Plotly/Dash dashboards** visualise the ideal, raw and corrected
trajectories together with the supporting infographics.

The headline result: across all tested motion profiles, the estimation
algorithm reduces position RMSE by **93–98 %** relative to raw inertial
dead‑reckoning, and maintains bounded error even through multi‑second GNSS
outages.

---

## 2. Objectives

1. Build a **reproducible** experimentation environment (single Docker image,
   pinned toolchain and dependencies).
2. Simulate a UGV with realistic, corruptible **IMU/GNSS/wheel** sensors.
3. Implement a **classical GNSS/INS fusion** estimator using modern C++
   (modules, namespaces, advanced STL, OOP) validated by unit tests.
4. Provide **automatic metric computation** and **interactive visualisation** of
   the ideal, raw and corrected robot routes.
5. Cross‑check the estimator concept against the published **KF‑GINS**
   reference.

---

## 3. System architecture

Nine ROS 2 packages plus a standalone Python simulator:

```
Gazebo Harmonic ──(ros_gz bridge)──► /imu /gps /wheel /ground_truth
        │                                   │
        ▼                                   ▼
 ugv_fault_injector ──► /sensors/* ──► ugv_localization (ESKF) ──► /localization/odom, /path
                                            │                          │
                       ugv_bringup (baseline: robot_localization)      ▼
                                            └──────────────► ugv_metrics ──► data.csv + metrics.json
                                                                              │
                                                                              ▼
                                                                      ugv_dashboard (Dash)
```

| Package | Responsibility |
|---|---|
| `ugv_nav_core` | **C++23 module** core: types, attitude/geodesy math, ESKF, sensor models, metrics + GoogleTest suite |
| `ugv_description` | UGV URDF/xacro with IMU + GNSS links |
| `ugv_gazebo` | Gazebo world, `ros_gz` bridge, spawn/launch |
| `ugv_sensor_processing` | Wheel‑odometry (differential‑drive) node |
| `ugv_fault_injector` | Deterministic sensor corruption (noise/bias/outage/slip) |
| `ugv_localization` | Online **ESKF** estimator node |
| `ugv_metrics` | Time‑alignment + accuracy metrics (CSV/JSON) |
| `ugv_bringup` | Trajectory commander, operator-heartbeat link, `robot_localization` baseline, top‑level launch |
| `ugv_dashboard` | Results viewer **+ interactive GNSS/IMU simulator** |
| `ugv_return_home` | **C++23 return‑home‑on‑link‑loss failsafe** (ArduPilot/MAVLink) |

### 3.1 Modern‑C++ design highlight

The core is written as a **C++23 named module** `ugv.nav.core` (partitions:
`types`, `math`, `sensors`, `eskf`, `metrics`). Because ROS 2 links against
`libstdc++` while the modules are cleanest under `libc++`, the core is exposed to
ROS nodes through an **ABI‑stable header facade** (`ugv_nav_core.hpp`) backed by
a shared library — the modules are compiled once into a static archive and hidden
behind the facade. This lets the project use bleeding‑edge language features
without breaking ROS interoperability.

---

## 4. Methodology

### 4.1 Sensor models

* **IMU** — body specific force `[a_forward, a_lateral] = [dv/dt, v·ω]` and
  yaw‑rate `ω`, each augmented with a constant **bias** and Gaussian **noise**.
* **GNSS** — geodetic fix converted to a local NED/ENU frame, with Gaussian
  position noise and configurable **outage** windows.
* **Wheel odometry** — differential‑drive forward kinematics from joint states.

### 4.2 Estimator — Error‑State Kalman Filter

The online node runs a **15‑state ESKF** (position, velocity, attitude error,
accel bias, gyro bias) with IMU‑driven propagation and loosely‑coupled GNSS
position updates plus optional wheel‑speed updates — the standard KF‑GINS‑style
architecture.

For the interactive simulator (planar, real‑time in the browser) the same design
is expressed as a compact **8‑state EKF**
`[pₓ, p_y, θ, vₓ, v_y, b_aₓ, b_a_y, b_gz]`:

* **Predict** (every IMU sample): rotate bias‑corrected body acceleration into
  the navigation frame, integrate velocity and position, propagate the yaw with
  the bias‑corrected gyro, and update the covariance with the analytic Jacobian.
* **Update** (every GNSS fix): loosely‑coupled position correction
  `H = [I₂ | 0]`, standard Kalman gain.

The **raw** trajectory uses identical IMU integration **without** bias estimation
or GNSS updates, isolating the value added by the filter.

### 4.3 Fault injection & experiment matrix

A deterministic, seed‑controlled injector reproduces twelve scenarios (E01–E12)
spanning nominal operation, GNSS noise/outage, IMU noise/bias, odometry
noise/scale error, wheel slip, and a combined worst case.

### 4.4 Metrics

`ate` (absolute trajectory error), position/velocity/yaw **RMSE**, **max error**,
**RPE** (relative pose error) and travelled distance — computed in C++ and
emitted as `metrics.json`, and re‑derived in Python for the simulator cards.

---

## 5. Results

### 5.1 Estimator performance across motion profiles

Interactive simulator, 40 s runs, GNSS outage 15–23 s, seed 42:

| Route | RMSE — raw (m) | RMSE — corrected (m) | Improvement |
|---|---:|---:|---:|
| figure‑8 | 40.15 | 1.76 | **95.6 %** |
| circle | 32.86 | 0.64 | **98.0 %** |
| square | 15.88 | 0.73 | **95.4 %** |
| spiral | 12.66 | 0.82 | **93.5 %** |
| s‑curve | 40.14 | 1.07 | **97.3 %** |

### 5.2 Qualitative view (figure‑8, GNSS outage 25–35 s)

![Ideal vs. raw vs. EKF‑corrected route](sim_preview.png)

* **Left** — the green ideal route, the red raw dead‑reckoning drifting away, and
  the blue EKF track locked onto ground truth using the amber GNSS fixes.
* **Right** — raw error grows unbounded (~35 m by the end) while the corrected
  error stays under ~2 m; the small bump around 25–35 s is the GNSS outage, after
  which the filter re‑converges once fixes resume.

### 5.3 Interpretation

* **Unbounded inertial drift.** Even modest IMU bias/noise makes raw
  dead‑reckoning diverge quadratically — confirming the necessity of aiding.
* **Bounded fused error.** GNSS updates make the estimate observable; the ESKF
  also **estimates the biases online**, so accuracy stays sub‑metre.
* **Graceful outage handling.** During GNSS loss the filter coasts on inertial
  prediction with error growing slowly (bias already partially estimated) and
  snaps back on fix resumption — the behaviour the E04–E06 matrix targets.

---

## 5b. Return-home-on-link-loss failsafe (`ugv_return_home`)

A second onboard subsystem, built to the same engineering standard as the estimation
core, addresses vehicle safety: if the operator datalink is lost, the flight computer
must autonomously bring the machine home.

### Design

* **C++23 named module** `ugv.rth` with partitions `types`, `geo`, `route`, `simplify`,
  `link`, `mission`, exposed to ROS through an **ABI-stable facade** (`ugv::rth::api`,
  PIMPL) — mirroring the `ugv_nav_core` architecture.
* **MISRA-friendly control path**: fixed-capacity storage (`StaticVector`, no dynamic
  allocation), no exceptions (status by value), strong `enum class` with fixed
  underlying type, `[[nodiscard]]`/`noexcept` queries, recursion-free Douglas-Peucker
  (explicit bounded work-stack), and dependency-free geodesy (no Eigen on the vehicle).
* **Dependency inversion**: the controller talks to an injected `IAutopilotLink`
  abstraction — the **MAVLink/ArduPilot seam**. Three interchangeable backends exist:
  a **simulator** (`rth_demo`), a **diagnostic ROS bridge** (`RosBridgeLink`, latched
  `/rth/return_path` + `/rth/autopilot_mode` for RViz), and a **real mavros client**
  (`MavrosLink`: `SetMode` / `WaypointPush` / `WaypointClear`, compiled when
  `mavros_msgs` is present).
* **Closed-loop actuation in sim**: `rth_node` additionally publishes the mission
  state (`/rth/mission_state`) and recorded trail (`/rth/recorded_path`); a pure-pursuit
  `return_follower` takes over `/cmd_vel` while `Returning` (with `trajectory_commander`
  yielding), so the Gazebo vehicle physically retraces its route home.

### Behaviour

The `MissionController` is a four-state machine — `Idle → Recording → Returning →
Home`. While recording, positions (from MAVLink `GLOBAL_POSITION_INT`) are decimated
into a bounded breadcrumb trail; operator heartbeats feed a `LinkMonitor`. When the
heartbeat ages past `link_timeout`, the controller:

1. reverses the trail (current → … → home) and simplifies it (Douglas-Peucker) to fit
   the autopilot mission cap,
2. commands `SET_MODE → GUIDED`, uploads the mission (`MISSION_COUNT` +
   `MISSION_ITEM_INT`), then `SET_MODE → AUTO`,
3. monitors arrival and finally commands `HOLD` at home.

Heartbeat recovery optionally aborts the return (`resume_on_recovery`).

### Verification

* **22 GoogleTest** cases (`geo`, `route`/`StaticVector`, `simplify`, `link`,
  `mission`) — including the full failsafe scenario (drive out → link loss → backtrack
  upload → arrive home), recovery-abort, manual force, and reset.
* **Deterministic demo** (`rth_demo`): an L-shaped 100 m route recorded as 30
  breadcrumbs is simplified to a **3-waypoint** backtrack; on a 2 s link timeout the
  simulated autopilot returns the vehicle to within **0.0 m** of home. The run is
  exported to JSON and rendered by `ugv_dashboard.rth_plots` into a route + metrics
  figure (`docs/rth_metrics.png`): recorded vs. backtrack vs. actual path,
  distance-to-home (peak ≈ 71 m → 0 m), backtrack speed (≈ 5 m/s), cross-track error
  (≤ 0.7 m), and the mission-state timeline.

![Return-home route and metrics](rth_metrics.png)
* **Live ROS integration**: with a moving `/gps/fix` and an `operator_link` heartbeat
  that drops, `rth_node` transitions `RECORDING → RETURNING`, issues `SET_MODE GUIDED`,
  uploads the backtrack mission and `SET_MODE AUTO`, publishing the latched mission on
  `/rth/return_path`. Wired into `ugv_bringup` behind `return_home:=true`.
* **Hardware-in-the-loop**: `scripts/run_sitl_failsafe.sh` drives the `MavrosLink`
  backend against ArduPilot SITL over MAVLink (mavros).

---

## 6. Reproducibility

* **Single image** (`osrf/ros:jazzy-desktop` base) with **pinned** Clang 19,
  Kitware CMake, Ninja, GoogleTest, Eigen, and a pinned Python `venv`
  (`requirements.txt`).
* **Dev Container** + **docker‑compose** for both IDE and headless/CI use.
* Deterministic seeds throughout the fault injector and simulator.
* One‑command entry points (`scripts/*.sh`) for build, experiments, dashboards
  and the KF‑GINS reference.

---

## 7. Verification & testing

* **26 GoogleTest** cases in `ugv_nav_core` covering attitude algebra, WGS‑84
  geodesy conversions, ESKF predict/update, sensor kinematics and metrics.
* **22 GoogleTest** cases in `ugv_return_home` covering the failsafe module.
* Full workspace `colcon build` (10 packages) and `colcon test` pass.
* Interactive simulator numerics assert **corrected RMSE < raw RMSE** for every
  route shape; Ruff‑clean Python.
* Dashboards validated to boot and serve a valid callback dependency graph.

---

## 8. Limitations & future work

* **Planar simulator** — the browser EKF is 2‑D for interactivity; the full 3‑D
  ESKF lives in the ROS node/Gazebo pipeline.
* **Loosely‑coupled GNSS** — a tightly‑coupled (pseudorange/Doppler) variant
  would improve outage/urban‑canyon behaviour.
* **Stage 2 (ML)** — learned motion/measurement models, outlier rejection and
  adaptive noise are deferred as planned.
* **Quantitative KF‑GINS cross‑check** — currently a reproduction script; a
  numeric A/B against the reference dataset is a natural next milestone.
* **Real MAVLink backend** — implemented (`MavrosLink`) and integrated via
  `scripts/run_sitl_failsafe.sh`; a full ArduPilot SITL soak test depends on the host
  having `sim_vehicle.py` (the image now ships mavros), and closing the loop so the
  Gazebo vehicle physically drives the backtrack (rather than only visualising it) is a
  follow-up.

---

## 9. Conclusion

NavStand delivers a reproducible, end‑to‑end stand for GRC navigation research:
modern C++23 estimation, a physics‑backed simulation, deterministic fault
injection, automatic metrics and interactive visualisation. The estimator
consistently converts diverging inertial dead‑reckoning into sub‑metre,
outage‑robust navigation (93–98 % RMSE reduction), providing a solid classical
baseline for the machine‑learning extensions planned in Stage 2.

---

## 10. References

1. KF‑GINS — i2Nav‑WHU: <https://github.com/i2Nav-WHU/KF-GINS>
2. ROS 2 Jazzy documentation.
3. Gazebo Harmonic + `ros_gz` integration.
4. J. Sola, *Quaternion kinematics for the error‑state Kalman filter*, 2017.
5. `robot_localization` (baseline EKF).
