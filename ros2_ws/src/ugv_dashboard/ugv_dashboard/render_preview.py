#!/usr/bin/env python3
"""Render a static PNG preview of the ideal / raw / corrected routes.

Handy for docs and quick visual checks without launching the live dashboard::

    python -m ugv_dashboard.render_preview --out docs/sim_preview.png
"""
from __future__ import annotations

import argparse

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

from ugv_dashboard.simulation import SimConfig, run_simulation

IDEAL_C, RAW_C, COR_C, GNSS_C = "#2ecc71", "#e74c3c", "#3498db", "#f1c40f"


def render(cfg: SimConfig, out: str) -> None:
    r = run_simulation(cfg)
    gt, raw, cor, s, m = r["gt"], r["raw"], r["corrected"], r["sensors"], r["metrics"]
    mask = s["gnss_mask"]
    t = gt["t"]
    err_raw = np.hypot(raw["x"] - gt["x"], raw["y"] - gt["y"])
    err_cor = np.hypot(cor["x"] - gt["x"], cor["y"] - gt["y"])

    fig = make_subplots(
        rows=1, cols=2, column_widths=[0.6, 0.4],
        subplot_titles=("Robot route", "Position error vs. time"),
    )
    fig.add_trace(go.Scatter(x=gt["x"], y=gt["y"], name="Ideal (ground truth)",
                             line=dict(color=IDEAL_C, width=4)), 1, 1)
    fig.add_trace(go.Scatter(x=raw["x"], y=raw["y"], name="Raw (IMU dead-reckoning)",
                             line=dict(color=RAW_C, width=2, dash="dot")), 1, 1)
    fig.add_trace(go.Scatter(x=cor["x"], y=cor["y"], name="Corrected (EKF)",
                             line=dict(color=COR_C, width=2.5, dash="dash")), 1, 1)
    fig.add_trace(go.Scatter(x=s["gnss_xy"][mask, 0], y=s["gnss_xy"][mask, 1],
                             mode="markers", name="GNSS fixes",
                             marker=dict(color=GNSS_C, size=5, symbol="x", opacity=0.5)), 1, 1)
    fig.add_trace(go.Scatter(x=t, y=err_raw, name="raw error",
                             line=dict(color=RAW_C)), 1, 2)
    fig.add_trace(go.Scatter(x=t, y=err_cor, name="corrected error",
                             line=dict(color=COR_C)), 1, 2)

    fig.update_xaxes(title_text="East X [m]", row=1, col=1)
    fig.update_yaxes(title_text="North Y [m]", scaleanchor="x", scaleratio=1, row=1, col=1)
    fig.update_xaxes(title_text="t [s]", row=1, col=2)
    fig.update_yaxes(title_text="error [m]", row=1, col=2)
    fig.update_layout(
        template="plotly_white", width=1280, height=560,
        title=(f"UGV navigation — {cfg.trajectory} · "
               f"RMSE raw {m['rmse_raw']:.1f} m → corrected {m['rmse_corrected']:.2f} m "
               f"({m['improvement_pct']:.0f}% better)"),
        legend=dict(orientation="h", yanchor="bottom", y=1.06, x=0),
    )
    fig.write_image(out, scale=2)
    print(f"wrote {out}")


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--out", default="sim_preview.png")
    p.add_argument("--trajectory", default="figure8")
    p.add_argument("--outage-start", type=float, default=25.0)
    p.add_argument("--outage-duration", type=float, default=10.0)
    a = p.parse_args()
    render(
        SimConfig(trajectory=a.trajectory, duration=60.0,
                  gnss_outage_start=a.outage_start, gnss_outage_duration=a.outage_duration),
        a.out,
    )


if __name__ == "__main__":
    main()
