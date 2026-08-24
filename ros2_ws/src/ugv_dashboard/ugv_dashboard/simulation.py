#!/usr/bin/env python3
"""Self-contained 2-D GNSS/IMU navigation simulation for the interactive
dashboard.

It generates a *ground-truth* (ideal) trajectory, synthesises noisy **IMU**
(body accelerometer + gyroscope, with bias) and **GNSS** (position, with noise
and optional outage) measurements, then reconstructs the path two ways:

* **raw**       – pure IMU strapdown dead-reckoning (drifts with bias/noise);
* **corrected** – an 8-state Extended Kalman Filter that fuses IMU prediction
                  with GNSS position updates (the "algorithm").

The EKF mirrors the C++ ``ugv.nav.core`` ESKF design (predict on IMU, loosely
coupled GNSS position update) in a compact planar form so the dashboard runs
instantly without ROS or Gazebo.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass

import numpy as np

TRAJECTORIES = ("figure8", "circle", "s_curve", "square", "spiral")


@dataclass
class SimConfig:
    """All knobs exposed by the dashboard controls."""

    trajectory: str = "figure8"
    duration: float = 60.0          # [s]
    dt: float = 0.02                # [s] IMU period (50 Hz)
    speed: float = 1.5              # [m/s] nominal forward speed
    gnss_rate: float = 5.0          # [Hz]
    gnss_noise: float = 1.0         # [m] 1-sigma position noise
    gnss_outage_start: float = -1.0  # [s] (<0 disables)
    gnss_outage_duration: float = 0.0  # [s]
    accel_noise: float = 0.08       # [m/s^2] 1-sigma
    gyro_noise: float = 0.01        # [rad/s] 1-sigma
    accel_bias: float = 0.15        # [m/s^2] constant bias (body-x)
    gyro_bias: float = 0.012        # [rad/s] constant bias
    seed: int = 42


# --------------------------------------------------------------------------- #
# Ground-truth trajectory (unicycle kinematics)
# --------------------------------------------------------------------------- #
def _omega_profile(cfg: SimConfig, t: np.ndarray) -> np.ndarray:
    """Yaw-rate profile [rad/s] for the selected trajectory shape."""
    T = cfg.duration
    if cfg.trajectory == "circle":
        return np.full_like(t, 2 * np.pi / T)                     # one loop
    if cfg.trajectory == "figure8":
        return 0.6 * np.sin(2 * np.pi * t / (T / 2))
    if cfg.trajectory == "s_curve":
        return 0.5 * np.sin(2 * np.pi * t / (T / 3))
    if cfg.trajectory == "spiral":
        return np.full_like(t, 0.35)
    if cfg.trajectory == "square":                                # turn in bursts
        omega = np.zeros_like(t)
        quarter = T / 4.0
        turn = np.pi / 2 / 1.5                                     # 1.5 s turns
        for k in range(1, 4):
            mask = (t >= k * quarter) & (t < k * quarter + 1.5)
            omega[mask] = turn
        return omega
    return np.zeros_like(t)


def ground_truth(cfg: SimConfig) -> dict[str, np.ndarray]:
    """Integrate the unicycle model to produce the ideal trajectory + the true
    body-frame specific force / yaw-rate that a perfect IMU would read."""
    t = np.arange(0.0, cfg.duration, cfg.dt)
    n = t.size
    omega = _omega_profile(cfg, t)

    # Spiral gently ramps speed to open the loops; others use constant speed.
    v = np.full(n, cfg.speed)
    if cfg.trajectory == "spiral":
        v = cfg.speed * (0.3 + 0.7 * t / cfg.duration)

    theta = np.cumsum(omega) * cfg.dt
    vx = v * np.cos(theta)
    vy = v * np.sin(theta)
    x = np.cumsum(vx) * cfg.dt
    y = np.cumsum(vy) * cfg.dt

    a_forward = np.gradient(v, cfg.dt)          # tangential acceleration
    a_lateral = v * omega                        # centripetal (body-y)
    return {
        "t": t, "x": x, "y": y, "theta": theta,
        "vx": vx, "vy": vy, "v": v, "omega": omega,
        "a_forward": a_forward, "a_lateral": a_lateral,
    }


# --------------------------------------------------------------------------- #
# Sensor synthesis
# --------------------------------------------------------------------------- #
def synthesise_sensors(cfg: SimConfig, gt: dict[str, np.ndarray]) -> dict:
    """Create noisy IMU (every dt) and GNSS (every 1/gnss_rate, outage-aware)."""
    rng = np.random.default_rng(cfg.seed)
    n = gt["t"].size

    accel = np.column_stack([gt["a_forward"], gt["a_lateral"]])
    accel[:, 0] += cfg.accel_bias
    accel += rng.normal(0.0, cfg.accel_noise, size=accel.shape)
    gyro = gt["omega"] + cfg.gyro_bias + rng.normal(0.0, cfg.gyro_noise, size=n)

    gnss_step = max(1, int(round(1.0 / (cfg.gnss_rate * cfg.dt))))
    outage_end = cfg.gnss_outage_start + cfg.gnss_outage_duration
    gnss_mask = np.zeros(n, dtype=bool)
    gnss_xy = np.full((n, 2), np.nan)
    for i in range(0, n, gnss_step):
        ti = gt["t"][i]
        if cfg.gnss_outage_start >= 0.0 and cfg.gnss_outage_start <= ti < outage_end:
            continue
        gnss_mask[i] = True
        gnss_xy[i, 0] = gt["x"][i] + rng.normal(0.0, cfg.gnss_noise)
        gnss_xy[i, 1] = gt["y"][i] + rng.normal(0.0, cfg.gnss_noise)
    return {"accel": accel, "gyro": gyro, "gnss_mask": gnss_mask, "gnss_xy": gnss_xy}


# --------------------------------------------------------------------------- #
# Reconstruction 1: raw IMU dead-reckoning
# --------------------------------------------------------------------------- #
def dead_reckoning(cfg: SimConfig, sensors: dict) -> dict[str, np.ndarray]:
    """Strapdown integration of the raw IMU only (no bias estimation, no GNSS)."""
    n = sensors["gyro"].size
    x, y, theta = np.zeros(n), np.zeros(n), np.zeros(n)
    vx, vy = np.zeros(n), np.zeros(n)
    for k in range(1, n):
        th = theta[k - 1] + sensors["gyro"][k] * cfg.dt
        c, s = np.cos(th), np.sin(th)
        ax, ay = sensors["accel"][k]
        anx = c * ax - s * ay
        any_ = s * ax + c * ay
        vxk = vx[k - 1] + anx * cfg.dt
        vyk = vy[k - 1] + any_ * cfg.dt
        x[k] = x[k - 1] + vxk * cfg.dt
        y[k] = y[k - 1] + vyk * cfg.dt
        vx[k], vy[k], theta[k] = vxk, vyk, th
    return {"x": x, "y": y, "theta": theta, "vx": vx, "vy": vy}


# --------------------------------------------------------------------------- #
# Reconstruction 2: 8-state INS/GNSS EKF (the algorithm)
# --------------------------------------------------------------------------- #
def ekf_fuse(cfg: SimConfig, sensors: dict) -> dict[str, np.ndarray]:
    """State = [px, py, theta, vx, vy, b_ax, b_ay, b_gz].

    Predict with the IMU, correct with GNSS position fixes.
    """
    n = sensors["gyro"].size
    s = np.zeros(8)
    P = np.diag([4.0, 4.0, (10 * np.pi / 180) ** 2, 1.0, 1.0, 0.25, 0.25, 0.01])

    q_a, q_g = cfg.accel_noise ** 2, cfg.gyro_noise ** 2
    Q = np.diag([0, 0, q_g * cfg.dt, q_a * cfg.dt, q_a * cfg.dt,
                 1e-6, 1e-6, 1e-7])
    R = np.diag([cfg.gnss_noise ** 2, cfg.gnss_noise ** 2])
    H = np.zeros((2, 8))
    H[0, 0] = H[1, 1] = 1.0

    out = {k: np.zeros(n) for k in ("x", "y", "theta", "vx", "vy")}
    trace = np.zeros(n)
    for k in range(n):
        if k > 0:
            th = s[2]
            ax, ay = sensors["accel"][k] - s[5:7]
            w = sensors["gyro"][k] - s[7]
            c, sn = np.cos(th), np.sin(th)
            Rm = np.array([[c, -sn], [sn, c]])
            a_nav = Rm @ np.array([ax, ay])

            s[0] += s[3] * cfg.dt
            s[1] += s[4] * cfg.dt
            s[3] += a_nav[0] * cfg.dt
            s[4] += a_nav[1] * cfg.dt
            s[2] += w * cfg.dt

            F = np.eye(8)
            F[0, 3] = F[1, 4] = cfg.dt
            dR_dth = np.array([[-sn, -c], [c, -sn]]) @ np.array([ax, ay])
            F[3, 2] = dR_dth[0] * cfg.dt
            F[4, 2] = dR_dth[1] * cfg.dt
            F[3, 5:7] = -Rm[0, :] * cfg.dt
            F[4, 5:7] = -Rm[1, :] * cfg.dt
            F[2, 7] = -cfg.dt
            P = F @ P @ F.T + Q

        if sensors["gnss_mask"][k]:
            z = sensors["gnss_xy"][k]
            y_innov = z - s[0:2]
            S = H @ P @ H.T + R
            K = P @ H.T @ np.linalg.inv(S)
            s = s + K @ y_innov
            P = (np.eye(8) - K @ H) @ P

        out["x"][k], out["y"][k], out["theta"][k] = s[0], s[1], s[2]
        out["vx"][k], out["vy"][k] = s[3], s[4]
        trace[k] = np.sqrt(P[0, 0] + P[1, 1])
    out["pos_std"] = trace
    return out


# --------------------------------------------------------------------------- #
# Metrics
# --------------------------------------------------------------------------- #
def _rmse(ax, ay, bx, by) -> float:
    return float(np.sqrt(np.mean((ax - bx) ** 2 + (ay - by) ** 2)))


def compute_metrics(gt, raw, cor) -> dict[str, float]:
    err_cor = np.hypot(cor["x"] - gt["x"], cor["y"] - gt["y"])
    err_raw = np.hypot(raw["x"] - gt["x"], raw["y"] - gt["y"])
    dist = float(np.sum(np.hypot(np.diff(gt["x"]), np.diff(gt["y"]))))
    rmse_raw = _rmse(raw["x"], raw["y"], gt["x"], gt["y"])
    rmse_cor = _rmse(cor["x"], cor["y"], gt["x"], gt["y"])
    return {
        "rmse_raw": rmse_raw,
        "rmse_corrected": rmse_cor,
        "max_error_corrected": float(err_cor.max()),
        "max_error_raw": float(err_raw.max()),
        "final_error_raw": float(err_raw[-1]),
        "final_error_corrected": float(err_cor[-1]),
        "improvement_pct": float(100.0 * (1.0 - rmse_cor / rmse_raw)) if rmse_raw > 1e-9 else 0.0,
        "travelled_distance": dist,
        "drift_raw_pct": float(100.0 * err_raw[-1] / dist) if dist > 1e-6 else 0.0,
    }


def run_simulation(cfg: SimConfig) -> dict:
    """One-shot: ground truth -> sensors -> raw + corrected -> metrics."""
    gt = ground_truth(cfg)
    sensors = synthesise_sensors(cfg, gt)
    raw = dead_reckoning(cfg, sensors)
    cor = ekf_fuse(cfg, sensors)
    metrics = compute_metrics(gt, raw, cor)
    return {"config": asdict(cfg), "gt": gt, "sensors": sensors,
            "raw": raw, "corrected": cor, "metrics": metrics}
