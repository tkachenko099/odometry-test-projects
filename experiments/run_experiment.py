#!/usr/bin/env python3
"""Automated experiment runner (Sections 16 & 20).

Given an experiment YAML, it:
  1. derives a unique experiment id,
  2. launches the full simulation + estimator + metrics stack,
  3. optionally records a rosbag,
  4. waits for the route to finish,
  5. shuts everything down gracefully (so the metrics node flushes
     ``data.csv`` / ``metrics.json``),
  6. renders trajectory / error plots,
  7. saves a self-contained result directory.

Usage::

    python experiments/run_experiment.py experiments/configs/E04_gnss_outage_10s.yaml
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RESULTS = REPO_ROOT / "experiments" / "results"
BAG_TOPICS = [
    "/sensors/imu", "/sensors/gnss", "/sensors/wheel_odom",
    "/ground_truth/odom", "/localization/odom", "/localization/path",
    "/cmd_vel", "/tf", "/tf_static",
]


def _load_config(path: Path) -> dict[str, Any]:
    with path.open() as fh:
        return yaml.safe_load(fh) or {}


def _write_fault_params(cfg: dict[str, Any], seed: int, dst: Path) -> None:
    """Extract the fault_injector_node section into a standalone params file."""
    section = cfg.get("fault_injector_node", {"ros__parameters": {}})
    section.setdefault("ros__parameters", {})["seed"] = seed
    with dst.open("w") as fh:
        yaml.safe_dump({"fault_injector_node": section}, fh)


def _git_rev() -> str:
    try:
        return subprocess.check_output(
            ["git", "-C", str(REPO_ROOT), "rev-parse", "--short", "HEAD"],
            stderr=subprocess.DEVNULL,
        ).decode().strip()
    except Exception:  # noqa: BLE001 - best-effort provenance
        return "unknown"


def run(config_path: Path, results_root: Path, record_bag: bool, extra_time: float) -> Path:
    cfg = _load_config(config_path)
    exp = cfg.get("experiment", {})
    exp_id = exp.get("id") or config_path.stem
    seed = int(exp.get("seed", 42))
    duration = float(exp.get("duration", 60.0))
    baseline = bool(exp.get("baseline", True))

    stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    result_dir = results_root / f"{exp_id}_{stamp}"
    result_dir.mkdir(parents=True, exist_ok=True)

    shutil.copy(config_path, result_dir / "config.yaml")
    (result_dir / "metadata.json").write_text(
        json.dumps(
            {
                "experiment_id": exp_id,
                "seed": seed,
                "duration_s": duration,
                "baseline": baseline,
                "git_rev": _git_rev(),
                "created_utc": stamp,
                "description": exp.get("description", ""),
            },
            indent=2,
        )
    )

    fault_params = result_dir / "fault_injector.yaml"
    _write_fault_params(cfg, seed, fault_params)

    data_csv = result_dir / "data.csv"
    metrics_json = result_dir / "metrics.json"

    launch_cmd = [
        "ros2", "launch", "ugv_bringup", "bringup.launch.py",
        f"scenario_file:={fault_params}",
        f"output_csv:={data_csv}",
        f"output_json:={metrics_json}",
        f"baseline:={'true' if baseline else 'false'}",
    ]
    print(f"[run] launching: {' '.join(launch_cmd)}", flush=True)
    launch = subprocess.Popen(launch_cmd, preexec_fn=os.setsid)

    bag_proc = None
    if record_bag:
        bag_dir = result_dir / "bag"
        bag_proc = subprocess.Popen(
            ["ros2", "bag", "record", "-o", str(bag_dir), *BAG_TOPICS],
            preexec_fn=os.setsid,
        )

    try:
        time.sleep(duration + extra_time)
    except KeyboardInterrupt:
        print("[run] interrupted; shutting down early", flush=True)

    # Graceful shutdown so the metrics node flushes its files.
    for proc in (bag_proc, launch):
        if proc and proc.poll() is None:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    for proc in (bag_proc, launch):
        if proc:
            try:
                proc.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)

    _render_plots(result_dir)
    print(f"[run] artifacts written to {result_dir}", flush=True)
    return result_dir


def _render_plots(result_dir: Path) -> None:
    data_csv = result_dir / "data.csv"
    if not data_csv.exists():
        print("[run] no data.csv - skipping plots", flush=True)
        return
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    import pandas as pd

    df = pd.read_csv(data_csv)
    if df.empty:
        return

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(df.gt_x, df.gt_y, label="Ground Truth", linewidth=2)
    ax.plot(df.est_x, df.est_y, "--", label="ESKF")
    ax.set_xlabel("North [m]"); ax.set_ylabel("East [m]"); ax.axis("equal")
    ax.legend(); ax.set_title("Trajectory"); fig.tight_layout()
    fig.savefig(result_dir / "trajectory.png", dpi=120); plt.close(fig)

    err = np.sqrt((df.est_x - df.gt_x) ** 2 + (df.est_y - df.gt_y) ** 2
                  + (df.est_z - df.gt_z) ** 2)
    fig, ax = plt.subplots(figsize=(8, 3))
    ax.plot(df.t, err); ax.set_xlabel("time [s]"); ax.set_ylabel("error [m]")
    ax.set_title("Position error"); fig.tight_layout()
    fig.savefig(result_dir / "position_error.png", dpi=120); plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 3))
    ax.plot(df.t, df.gt_vx, label="GT"); ax.plot(df.t, df.est_vx, "--", label="ESKF")
    ax.set_xlabel("time [s]"); ax.set_ylabel("v [m/s]"); ax.legend()
    ax.set_title("Forward velocity"); fig.tight_layout()
    fig.savefig(result_dir / "velocity_error.png", dpi=120); plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 3))
    ax.plot(df.t, np.degrees(df.gt_yaw), label="GT")
    ax.plot(df.t, np.degrees(df.est_yaw), "--", label="ESKF")
    ax.set_xlabel("time [s]"); ax.set_ylabel("yaw [deg]"); ax.legend()
    ax.set_title("Yaw"); fig.tight_layout()
    fig.savefig(result_dir / "yaw_error.png", dpi=120); plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="UGV experiment runner")
    parser.add_argument("config", type=Path, help="experiment YAML")
    parser.add_argument("--results-root", type=Path, default=DEFAULT_RESULTS)
    parser.add_argument("--no-bag", action="store_true", help="skip rosbag recording")
    parser.add_argument("--extra-time", type=float, default=8.0,
                        help="settle/shutdown margin [s]")
    args = parser.parse_args()

    if not args.config.exists():
        sys.exit(f"config not found: {args.config}")
    run(args.config, args.results_root, not args.no_bag, args.extra_time)


if __name__ == "__main__":
    main()
