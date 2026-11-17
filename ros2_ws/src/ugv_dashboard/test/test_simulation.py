"""Unit tests for the planar GNSS/IMU simulator behind the dashboard."""
from __future__ import annotations

import numpy as np
import pytest
from ugv_dashboard import simulation as sim


def test_ground_truth_is_consistent_unicycle_motion():
    cfg = sim.SimConfig(trajectory="circle", duration=40.0, speed=1.0)
    gt = sim.ground_truth(cfg)
    n = gt["t"].size
    assert n == round(cfg.duration / cfg.dt)
    assert all(gt[k].shape == (n,) for k in ("x", "y", "theta", "v", "omega"))
    # One full loop: the heading advances by 2*pi and the path closes.
    assert gt["theta"][-1] == pytest.approx(2 * np.pi, abs=0.02)
    assert np.hypot(gt["x"][-1] - gt["x"][0], gt["y"][-1] - gt["y"][0]) < 0.1
    # Constant speed along the heading.
    assert np.allclose(np.hypot(gt["vx"], gt["vy"]), cfg.speed)


@pytest.mark.parametrize("trajectory", sim.TRAJECTORIES)
def test_fusion_beats_dead_reckoning(trajectory):
    cfg = sim.SimConfig(trajectory=trajectory)
    gt = sim.ground_truth(cfg)
    sensors = sim.synthesise_sensors(cfg, gt)
    raw = sim.dead_reckoning(cfg, sensors)
    cor = sim.ekf_fuse(cfg, sensors)
    raw_rmse = sim._rmse(raw["x"], raw["y"], gt["x"], gt["y"])
    cor_rmse = sim._rmse(cor["x"], cor["y"], gt["x"], gt["y"])
    assert cor_rmse < 1.5, f"EKF RMSE {cor_rmse:.2f} m"
    assert cor_rmse < 0.25 * raw_rmse, f"EKF {cor_rmse:.2f} m vs raw {raw_rmse:.2f} m"


def test_same_seed_is_reproducible_and_outage_is_honoured():
    cfg = sim.SimConfig(gnss_outage_start=20.0, gnss_outage_duration=10.0)
    gt = sim.ground_truth(cfg)
    a = sim.synthesise_sensors(cfg, gt)
    b = sim.synthesise_sensors(cfg, gt)
    assert np.array_equal(a["accel"], b["accel"])
    assert np.array_equal(a["gnss_mask"], b["gnss_mask"])
    in_outage = (gt["t"] >= 20.0) & (gt["t"] < 30.0)
    assert not a["gnss_mask"][in_outage].any()
    assert a["gnss_mask"][~in_outage].any()
