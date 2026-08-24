#!/usr/bin/env python3
"""Plotly/Dash dashboard for the UGV navigation stand (Section 18).

Reads an experiment result directory (``data.csv`` + ``metrics.json``) and
renders trajectory, position/velocity/yaw errors, GNSS scatter, sensor status
and metric cards. A refresh interval makes it usable *online* (the metrics
node streams ``data.csv`` live) as well as *offline*.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

import dash
import dash_bootstrap_components as dbc
import numpy as np
import pandas as pd
import plotly.graph_objects as go
from dash import Input, Output, dcc, html

CSV_COLUMNS = [
    "t", "gt_x", "gt_y", "gt_z", "gt_yaw", "gt_vx",
    "est_x", "est_y", "est_z", "est_yaw", "est_vx",
]


def _latest_result(root: Path) -> Path:
    """Return the most recently modified experiment directory under ``root``."""
    candidates = [p for p in root.glob("*") if (p / "data.csv").exists()]
    if not candidates:
        return root
    return max(candidates, key=lambda p: p.stat().st_mtime)


def _load(result_dir: Path) -> tuple[pd.DataFrame, dict]:
    csv = result_dir / "data.csv"
    df = pd.read_csv(csv) if csv.exists() else pd.DataFrame(columns=CSV_COLUMNS)
    metrics_path = result_dir / "metrics.json"
    metrics = json.loads(metrics_path.read_text()) if metrics_path.exists() else {}
    return df, metrics


def _metric_card(title: str, value: str, unit: str = "") -> dbc.Card:
    return dbc.Card(
        dbc.CardBody(
            [
                html.H6(title, className="text-muted"),
                html.H4(f"{value}{unit}", className="card-title"),
            ]
        ),
        className="m-1 text-center",
    )


def create_app(result_dir: Path) -> dash.Dash:
    app = dash.Dash(__name__, external_stylesheets=[dbc.themes.FLATLY])
    app.title = "UGV Navigation Dashboard"

    app.layout = dbc.Container(
        fluid=True,
        children=[
            html.H2("UGV GNSS/INS Navigation Dashboard", className="my-3"),
            html.Div(id="result-path", className="text-muted mb-2"),
            dbc.Row(id="metric-cards", className="mb-3"),
            dbc.Row(
                [
                    dbc.Col(dcc.Graph(id="trajectory"), md=6),
                    dbc.Col(dcc.Graph(id="position-error"), md=6),
                ]
            ),
            dbc.Row(
                [
                    dbc.Col(dcc.Graph(id="velocity"), md=6),
                    dbc.Col(dcc.Graph(id="yaw"), md=6),
                ]
            ),
            dcc.Interval(id="tick", interval=2000, n_intervals=0),
            dcc.Store(id="result-dir", data=str(result_dir)),
        ],
    )

    @app.callback(
        Output("trajectory", "figure"),
        Output("position-error", "figure"),
        Output("velocity", "figure"),
        Output("yaw", "figure"),
        Output("metric-cards", "children"),
        Output("result-path", "children"),
        Input("tick", "n_intervals"),
        Input("result-dir", "data"),
    )
    def _update(_n: int, result_dir_str: str):
        rdir = Path(result_dir_str)
        df, metrics = _load(rdir)

        traj = go.Figure()
        pos_err = go.Figure()
        vel = go.Figure()
        yaw = go.Figure()
        if not df.empty:
            traj.add_trace(go.Scatter(x=df.gt_x, y=df.gt_y, name="Ground Truth",
                                      mode="lines", line=dict(width=3)))
            traj.add_trace(go.Scatter(x=df.est_x, y=df.est_y, name="ESKF",
                                      mode="lines", line=dict(dash="dash")))
            traj.update_layout(title="Trajectory (NED x-y)", xaxis_title="North [m]",
                               yaxis_title="East [m]", yaxis_scaleanchor="x")

            err = np.sqrt((df.est_x - df.gt_x) ** 2 + (df.est_y - df.gt_y) ** 2
                          + (df.est_z - df.gt_z) ** 2)
            pos_err.add_trace(go.Scatter(x=df.t, y=err, name="‖p_est − p_gt‖"))
            pos_err.update_layout(title="Position error", xaxis_title="time [s]",
                                  yaxis_title="error [m]")

            vel.add_trace(go.Scatter(x=df.t, y=df.gt_vx, name="GT vx"))
            vel.add_trace(go.Scatter(x=df.t, y=df.est_vx, name="ESKF vx"))
            vel.update_layout(title="Forward velocity", xaxis_title="time [s]",
                              yaxis_title="v [m/s]")

            yaw.add_trace(go.Scatter(x=df.t, y=np.degrees(df.gt_yaw), name="GT yaw"))
            yaw.add_trace(go.Scatter(x=df.t, y=np.degrees(df.est_yaw), name="ESKF yaw"))
            yaw.update_layout(title="Yaw", xaxis_title="time [s]", yaxis_title="yaw [deg]")

        def fmt(key: str, scale: float = 1.0, nd: int = 3) -> str:
            v = metrics.get(key)
            return f"{v * scale:.{nd}f}" if isinstance(v, (int, float)) else "—"

        cards = [
            _metric_card("Position RMSE", fmt("position_rmse"), " m"),
            _metric_card("ATE", fmt("ate"), " m"),
            _metric_card("RPE", fmt("rpe"), " m"),
            _metric_card("Yaw RMSE", fmt("yaw_rmse", 57.2958, 2), "°"),
            _metric_card("Max Error", fmt("max_error"), " m"),
            _metric_card("Drift", fmt("relative_drift_pct", 1.0, 2), " %"),
            _metric_card("Distance", fmt("travelled_distance", 1.0, 1), " m"),
            _metric_card("Duration", fmt("duration", 1.0, 1), " s"),
        ]
        cards = [dbc.Col(c, xs=6, md=3, lg="auto") for c in cards]
        return traj, pos_err, vel, yaw, cards, f"Result: {rdir}"

    return app


def main() -> None:
    parser = argparse.ArgumentParser(description="UGV navigation dashboard")
    default_root = os.environ.get(
        "UGV_RESULTS", str(Path.cwd() / "experiments" / "results")
    )
    parser.add_argument("--results", default=default_root,
                        help="experiment result dir, or a results root (uses latest)")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8050)
    args = parser.parse_args()

    result_dir = Path(args.results)
    if not (result_dir / "data.csv").exists():
        result_dir = _latest_result(result_dir)

    app = create_app(result_dir)
    app.run(host=args.host, port=args.port, debug=False)


if __name__ == "__main__":
    main()
