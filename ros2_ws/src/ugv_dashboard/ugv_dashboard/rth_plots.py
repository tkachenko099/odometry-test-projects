#!/usr/bin/env python3
"""Route + metrics visualisation for the return-home failsafe.

Consumes the JSON run log emitted by ``rth_demo run.json`` and renders a
multi-panel figure: the route map (recorded outbound trail, uploaded backtrack
mission, and the actual driven path) alongside distance-to-home, speed,
cross-track error, and the mission-state timeline::

    python -m ugv_dashboard.rth_plots --in run.json --out docs/rth_metrics.png

All coordinates are local ENU metres relative to the launch point (home ≈ 0,0).
"""
from __future__ import annotations

import argparse
import json

import numpy as np
import plotly.graph_objects as go
from plotly.subplots import make_subplots

REC_C, WPT_C, ACT_C, HOME_C, DROP_C = "#2ecc71", "#3498db", "#e67e22", "#111111", "#e74c3c"
STATES = ("IDLE", "RECORDING", "RETURNING", "HOME")
STATE_LVL = {s: i for i, s in enumerate(STATES)}


def _point_to_polyline(pts: np.ndarray, poly: np.ndarray) -> np.ndarray:
    """Minimum Euclidean distance from each point to a polyline.

    Parameters
    ----------
    pts : np.ndarray, shape (N, 2)
        Query points.
    poly : np.ndarray, shape (M, 2)
        Ordered polyline vertices.

    Returns
    -------
    np.ndarray, shape (N,)
        Distance from every query point to the nearest polyline segment.
    """
    if len(poly) < 2:
        return np.full(len(pts), np.nan)
    a, b = poly[:-1], poly[1:]                      # segment endpoints (M-1, 2)
    seg = b - a                                     # (M-1, 2)
    seg_len2 = np.einsum("ij,ij->i", seg, seg)      # (M-1,)
    seg_len2[seg_len2 == 0.0] = 1e-12
    d = pts[:, None, :] - a[None, :, :]             # (N, M-1, 2)
    tproj = np.clip(np.einsum("nij,ij->ni", d, seg) / seg_len2, 0.0, 1.0)  # (N, M-1)
    proj = a[None, :, :] + tproj[:, :, None] * seg[None, :, :]             # (N, M-1, 2)
    return np.linalg.norm(pts[:, None, :] - proj, axis=2).min(axis=1)      # (N,)


def build_figure(run: dict) -> go.Figure:
    """Assemble the return-home route + metrics dashboard figure."""
    s = run["samples"]
    t = np.array([p["t"] for p in s])
    north = np.array([p["north"] for p in s])
    east = np.array([p["east"] for p in s])
    state = np.array([p["state"] for p in s])
    trail = np.array(run["trail"]) if run["trail"] else np.empty((0, 2))
    wpts = np.array(run["waypoints"]) if run["waypoints"] else np.empty((0, 2))
    drop_t = run["link_loss_t"]

    xy = np.column_stack([east, north])             # plot east on X, north on Y
    dist_home = np.hypot(north, east)
    dt = np.diff(t, prepend=t[0] - 1e-3)
    speed = np.hypot(np.diff(east, prepend=east[0]), np.diff(north, prepend=north[0])) / dt

    returning = state == "RETURNING"
    xtrack = np.full(len(s), np.nan)
    if returning.any() and len(wpts) >= 2:
        xtrack[returning] = _point_to_polyline(xy[returning], wpts[:, ::-1])  # wpts are (n,e)
    lvl = np.array([STATE_LVL.get(x, 0) for x in state])

    fig = make_subplots(
        rows=2, cols=3, column_widths=[0.5, 0.25, 0.25],
        specs=[[{"rowspan": 2}, {}, {}], [None, {}, {}]],
        subplot_titles=(
            "Route: recorded vs. backtrack vs. actual",
            "Distance to home [m]", "Speed [m/s]",
            "Cross-track error (return) [m]", "Mission state",
        ),
        horizontal_spacing=0.08, vertical_spacing=0.12,
    )

    # --- Route map (col 1) --------------------------------------------------
    if len(trail):
        fig.add_trace(go.Scatter(x=trail[:, 1], y=trail[:, 0], name="Recorded trail",
                                 line=dict(color=REC_C, width=5), opacity=0.7), 1, 1)
    if len(wpts):
        fig.add_trace(go.Scatter(x=wpts[:, 1], y=wpts[:, 0], name="Backtrack mission",
                                 mode="lines+markers", line=dict(color=WPT_C, width=2, dash="dash"),
                                 marker=dict(size=7, symbol="triangle-up")), 1, 1)
    fig.add_trace(go.Scatter(x=east, y=north, name="Actual path",
                             line=dict(color=ACT_C, width=2)), 1, 1)
    fig.add_trace(go.Scatter(x=[0], y=[0], name="Home", mode="markers",
                             marker=dict(color=HOME_C, size=13, symbol="star")), 1, 1)
    di = int(np.searchsorted(t, drop_t))
    if 0 <= di < len(east):
        fig.add_trace(go.Scatter(x=[east[di]], y=[north[di]], name="Link loss",
                                 mode="markers",
                                 marker=dict(color=DROP_C, size=12, symbol="x")), 1, 1)

    # --- Metrics (cols 2-3) -------------------------------------------------
    fig.add_trace(go.Scatter(x=t, y=dist_home, line=dict(color=WPT_C), showlegend=False), 1, 2)
    fig.add_trace(go.Scatter(x=t, y=speed, line=dict(color=ACT_C), showlegend=False), 1, 3)
    fig.add_trace(go.Scatter(x=t, y=xtrack, line=dict(color=DROP_C), showlegend=False,
                             connectgaps=False), 2, 2)
    fig.add_trace(go.Scatter(x=t, y=lvl, line=dict(color="#8e44ad", shape="hv"),
                             showlegend=False), 2, 3)

    for r, c in ((1, 2), (1, 3), (2, 2), (2, 3)):  # link-loss marker on time plots
        fig.add_vline(x=drop_t, line=dict(color=DROP_C, dash="dot"), row=r, col=c)

    fig.update_xaxes(title_text="East [m]", row=1, col=1)
    fig.update_yaxes(title_text="North [m]", scaleanchor="x", scaleratio=1, row=1, col=1)
    for c in (2, 3):
        fig.update_xaxes(title_text="t [s]", row=2, col=c)
    fig.update_yaxes(rangemode="tozero", tickformat=".2f", row=2, col=2)
    fig.update_yaxes(tickmode="array", tickvals=list(STATE_LVL.values()),
                     ticktext=list(STATE_LVL.keys()), row=2, col=3)

    peak = float(dist_home.max()) if len(dist_home) else 0.0
    final = float(dist_home[-1]) if len(dist_home) else 0.0
    max_xt = float(np.nanmax(xtrack)) if np.isfinite(xtrack).any() else 0.0
    fig.update_layout(
        template="plotly_white", width=1400, height=680,
        title=(f"Return-home failsafe — trail {len(trail)} pts → "
               f"backtrack {len(wpts)} wpts · peak {peak:.0f} m, "
               f"returned to {final:.2f} m (max cross-track {max_xt:.2f} m)"),
        legend=dict(orientation="h", yanchor="bottom", y=1.04, x=0),
    )
    return fig


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--in", dest="inp", default="run.json", help="rth_demo JSON run log")
    p.add_argument("--out", default="rth_metrics.png", help="output PNG (or .html)")
    a = p.parse_args()
    with open(a.inp, encoding="utf-8") as f:
        run = json.load(f)
    fig = build_figure(run)
    if a.out.endswith(".html"):
        fig.write_html(a.out)
    else:
        fig.write_image(a.out, scale=2)
    print(f"wrote {a.out}")


if __name__ == "__main__":
    main()
