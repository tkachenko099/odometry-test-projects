#!/usr/bin/env python3
"""Interactive GNSS/IMU navigation dashboard.

Simulates a ground robot driving a chosen route, synthesises noisy IMU + GNSS
data and shows three trajectories live:

* **Ideal** – ground-truth route,
* **Raw (IMU dead-reckoning)** – what you get without correction (drifts),
* **Corrected (EKF)** – GNSS/IMU fusion (the algorithm).

Every control (route shape, noise, bias, GNSS rate, outage window, seed) instantly
re-runs the simulation; a Play button animates the robot along the routes.

Run:  python -m ugv_dashboard.interactive        (or scripts/sim_dashboard.sh)
"""
from __future__ import annotations

import argparse

import dash
import dash_bootstrap_components as dbc
import numpy as np
import plotly.graph_objects as go
from dash import Input, Output, State, dcc, html

from ugv_dashboard.simulation import SimConfig, TRAJECTORIES, run_simulation

IDEAL_C = "#2ecc71"     # green
RAW_C = "#e74c3c"       # red
COR_C = "#3498db"       # blue
GNSS_C = "#f1c40f"      # amber


# --------------------------------------------------------------------------- #
# Controls
# --------------------------------------------------------------------------- #
def _slider(id_, lo, hi, step, val, marks_every=None):
    marks = None
    if marks_every:
        marks = {int(x) if float(x).is_integer() else x: str(x)
                 for x in np.arange(lo, hi + 1e-9, marks_every)}
    return dcc.Slider(id=id_, min=lo, max=hi, step=step, value=val,
                      marks=marks, tooltip={"placement": "bottom", "always_visible": False})


def _controls() -> dbc.Card:
    d = SimConfig()
    return dbc.Card(
        dbc.CardBody(
            [
                html.H5("Scenario controls", className="mb-3"),
                dbc.Label("Route shape"),
                dcc.Dropdown(id="trajectory", value=d.trajectory, clearable=False,
                             options=[{"label": t.replace("_", " ").title(), "value": t}
                                      for t in TRAJECTORIES]),
                html.Br(),
                dbc.Label("Duration [s]"), _slider("duration", 20, 120, 5, d.duration, 20),
                dbc.Label("Speed [m/s]"), _slider("speed", 0.5, 4.0, 0.1, d.speed, 0.5),
                html.Hr(),
                html.Small("GNSS", className="text-muted"),
                dbc.Label("GNSS rate [Hz]"), _slider("gnss_rate", 1, 10, 1, d.gnss_rate, 1),
                dbc.Label("GNSS noise σ [m]"), _slider("gnss_noise", 0.0, 5.0, 0.1, d.gnss_noise, 1),
                dbc.Label("Outage start [s]  (−1 = none)"),
                _slider("outage_start", -1, 100, 1, d.gnss_outage_start, 20),
                dbc.Label("Outage duration [s]"),
                _slider("outage_dur", 0, 60, 1, d.gnss_outage_duration, 10),
                html.Hr(),
                html.Small("IMU", className="text-muted"),
                dbc.Label("Accel noise σ [m/s²]"), _slider("accel_noise", 0.0, 0.5, 0.01, d.accel_noise),
                dbc.Label("Gyro noise σ [rad/s]"), _slider("gyro_noise", 0.0, 0.1, 0.002, d.gyro_noise),
                dbc.Label("Accel bias [m/s²]"), _slider("accel_bias", 0.0, 0.6, 0.02, d.accel_bias),
                dbc.Label("Gyro bias [rad/s]"), _slider("gyro_bias", 0.0, 0.05, 0.002, d.gyro_bias),
                html.Hr(),
                dbc.Label("Seed"),
                dbc.Input(id="seed", type="number", value=d.seed, min=0, step=1),
                html.Br(),
                dbc.ButtonGroup(
                    [
                        dbc.Button("▶ Play", id="play", color="primary", n_clicks=0),
                        dbc.Button("⏸ Pause", id="pause", color="secondary", n_clicks=0),
                        dbc.Button("↺ Reset", id="reset", color="light", n_clicks=0),
                    ],
                    className="w-100",
                ),
                html.Br(), html.Br(),
                dbc.Label("Playback speed"),
                _slider("play_speed", 1, 20, 1, 6),
            ]
        ),
        className="shadow-sm",
    )


def _metric_card(title, cid, unit=""):
    return dbc.Card(
        dbc.CardBody([html.Small(title, className="text-muted d-block"),
                      html.H4([html.Span(id=cid), html.Small(unit, className="text-muted")],
                              className="mb-0")]),
        className="text-center shadow-sm",
    )


# --------------------------------------------------------------------------- #
# App
# --------------------------------------------------------------------------- #
def create_app() -> dash.Dash:
    app = dash.Dash(__name__, external_stylesheets=[dbc.themes.FLATLY],
                    title="UGV Navigation Simulator")

    app.layout = dbc.Container(
        fluid=True,
        children=[
            html.H3("UGV GNSS/INS Navigation Simulator", className="my-3"),
            html.P("Ideal vs. raw dead-reckoning vs. EKF-corrected route, "
                   "from live-simulated GNSS + IMU data.", className="text-muted"),
            dbc.Row(
                [
                    dbc.Col(_controls(), lg=3, md=4),
                    dbc.Col(
                        [
                            dbc.Row(
                                [
                                    dbc.Col(_metric_card("RMSE — raw", "m_rmse_raw", " m"), md=2),
                                    dbc.Col(_metric_card("RMSE — corrected", "m_rmse_cor", " m"), md=2),
                                    dbc.Col(_metric_card("Improvement", "m_impr", " %"), md=2),
                                    dbc.Col(_metric_card("Final error — raw", "m_fin_raw", " m"), md=2),
                                    dbc.Col(_metric_card("Final error — corr.", "m_fin_cor", " m"), md=2),
                                    dbc.Col(_metric_card("Distance", "m_dist", " m"), md=2),
                                ],
                                className="g-2 mb-2",
                            ),
                            dcc.Graph(id="route", style={"height": "62vh"}),
                        ],
                        lg=9, md=8,
                    ),
                ]
            ),
            dbc.Row(
                [
                    dbc.Col(dcc.Graph(id="pos_error"), md=6),
                    dbc.Col(dcc.Graph(id="heading"), md=6),
                ]
            ),
            dbc.Row(
                [
                    dbc.Col(dcc.Graph(id="speed_plot"), md=6),
                    dbc.Col(dcc.Graph(id="uncertainty"), md=6),
                ]
            ),
            dcc.Store(id="sim"),
            dcc.Interval(id="tick", interval=120, n_intervals=0, disabled=True),
        ],
    )

    _register_callbacks(app)
    return app


def _register_callbacks(app: dash.Dash) -> None:
    control_inputs = [
        Input("trajectory", "value"), Input("duration", "value"), Input("speed", "value"),
        Input("gnss_rate", "value"), Input("gnss_noise", "value"),
        Input("outage_start", "value"), Input("outage_dur", "value"),
        Input("accel_noise", "value"), Input("gyro_noise", "value"),
        Input("accel_bias", "value"), Input("gyro_bias", "value"), Input("seed", "value"),
    ]

    @app.callback(
        Output("sim", "data"), Output("tick", "n_intervals"),
        *control_inputs,
    )
    def _simulate(trajectory, duration, speed, gnss_rate, gnss_noise, outage_start,
                  outage_dur, accel_noise, gyro_noise, accel_bias, gyro_bias, seed):
        cfg = SimConfig(
            trajectory=trajectory, duration=float(duration), speed=float(speed),
            gnss_rate=float(gnss_rate), gnss_noise=float(gnss_noise),
            gnss_outage_start=float(outage_start), gnss_outage_duration=float(outage_dur),
            accel_noise=float(accel_noise), gyro_noise=float(gyro_noise),
            accel_bias=float(accel_bias), gyro_bias=float(gyro_bias),
            seed=int(seed if seed is not None else 42),
        )
        r = run_simulation(cfg)
        gt, raw, cor, s = r["gt"], r["raw"], r["corrected"], r["sensors"]
        mask = s["gnss_mask"]
        data = {
            "t": gt["t"].tolist(),
            "gt_x": gt["x"].tolist(), "gt_y": gt["y"].tolist(),
            "gt_theta": gt["theta"].tolist(), "gt_v": gt["v"].tolist(),
            "raw_x": raw["x"].tolist(), "raw_y": raw["y"].tolist(),
            "raw_theta": raw["theta"].tolist(),
            "cor_x": cor["x"].tolist(), "cor_y": cor["y"].tolist(),
            "cor_theta": cor["theta"].tolist(), "pos_std": cor["pos_std"].tolist(),
            "cor_v": np.hypot(cor["vx"], cor["vy"]).tolist(),
            "gnss_t": gt["t"][mask].tolist(),
            "gnss_x": s["gnss_xy"][mask, 0].tolist(),
            "gnss_y": s["gnss_xy"][mask, 1].tolist(),
            "metrics": r["metrics"], "n": int(gt["t"].size),
        }
        return data, 0

    # Playback controls ----------------------------------------------------- #
    @app.callback(
        Output("tick", "disabled"), Output("tick", "interval"),
        Input("play", "n_clicks"), Input("pause", "n_clicks"),
        Input("reset", "n_clicks"), Input("play_speed", "value"),
        prevent_initial_call=True,
    )
    def _playback(_p, _pa, _r, play_speed):
        trigger = dash.ctx.triggered_id
        interval = max(40, int(300 / max(1, play_speed)))
        if trigger == "play":
            return False, interval
        if trigger in ("pause", "reset"):
            return True, interval
        return dash.no_update, interval

    # Route view (animated) ------------------------------------------------- #
    @app.callback(
        Output("route", "figure"),
        Input("sim", "data"), Input("tick", "n_intervals"), State("play_speed", "value"),
    )
    def _route(data, n_intervals, play_speed):
        if not data:
            return go.Figure()
        n = data["n"]
        step = max(1, n // 130) * max(1, int(play_speed or 6) // 3)
        idx = min(n - 1, (n_intervals * step) % (n + step))
        idx = min(idx, n - 1)

        fig = go.Figure()
        fig.add_trace(go.Scatter(x=data["gt_x"], y=data["gt_y"], mode="lines",
                                 name="Ideal (ground truth)",
                                 line=dict(color=IDEAL_C, width=4)))
        fig.add_trace(go.Scatter(x=data["raw_x"], y=data["raw_y"], mode="lines",
                                 name="Raw (IMU dead-reckoning)",
                                 line=dict(color=RAW_C, width=2, dash="dot")))
        fig.add_trace(go.Scatter(x=data["cor_x"], y=data["cor_y"], mode="lines",
                                 name="Corrected (EKF)",
                                 line=dict(color=COR_C, width=2.5, dash="dash")))
        # GNSS fixes revealed up to current time.
        gt_now = data["t"][idx]
        gx = [x for x, tt in zip(data["gnss_x"], data["gnss_t"]) if tt <= gt_now]
        gy = [y for y, tt in zip(data["gnss_y"], data["gnss_t"]) if tt <= gt_now]
        fig.add_trace(go.Scatter(x=gx, y=gy, mode="markers", name="GNSS fixes",
                                 marker=dict(color=GNSS_C, size=6, symbol="x", opacity=0.6)))
        # Moving robot markers.
        for key, color, nm in (("gt", IDEAL_C, "robot (ideal)"),
                               ("cor", COR_C, "robot (EKF)")):
            fig.add_trace(go.Scatter(
                x=[data[f"{key}_x"][idx]], y=[data[f"{key}_y"][idx]],
                mode="markers", showlegend=False,
                marker=dict(color=color, size=15, symbol="circle",
                            line=dict(color="white", width=2)),
                hovertext=nm))
        fig.update_layout(
            title=f"Robot route  ·  t = {gt_now:5.1f}s",
            xaxis_title="East X [m]", yaxis_title="North Y [m]",
            yaxis=dict(scaleanchor="x", scaleratio=1),
            legend=dict(orientation="h", yanchor="bottom", y=1.02, x=0),
            margin=dict(l=40, r=20, t=60, b=40), template="plotly_white",
        )
        return fig

    # Infographics ---------------------------------------------------------- #
    @app.callback(
        Output("pos_error", "figure"), Output("heading", "figure"),
        Output("speed_plot", "figure"), Output("uncertainty", "figure"),
        Output("m_rmse_raw", "children"), Output("m_rmse_cor", "children"),
        Output("m_impr", "children"), Output("m_fin_raw", "children"),
        Output("m_fin_cor", "children"), Output("m_dist", "children"),
        Input("sim", "data"),
    )
    def _infographics(data):
        if not data:
            e = go.Figure()
            return e, e, e, e, "—", "—", "—", "—", "—", "—"
        t = np.array(data["t"])
        gx, gy = np.array(data["gt_x"]), np.array(data["gt_y"])
        err_raw = np.hypot(np.array(data["raw_x"]) - gx, np.array(data["raw_y"]) - gy)
        err_cor = np.hypot(np.array(data["cor_x"]) - gx, np.array(data["cor_y"]) - gy)

        perr = go.Figure()
        perr.add_trace(go.Scatter(x=t, y=err_raw, name="raw", line=dict(color=RAW_C)))
        perr.add_trace(go.Scatter(x=t, y=err_cor, name="corrected", line=dict(color=COR_C)))
        perr.update_layout(title="Position error vs. time", xaxis_title="t [s]",
                           yaxis_title="error [m]", template="plotly_white",
                           margin=dict(l=40, r=20, t=50, b=40))

        def wrap(a):
            return (np.degrees(np.array(a)) + 180) % 360 - 180

        head = go.Figure()
        head.add_trace(go.Scatter(x=t, y=wrap(data["gt_theta"]), name="ideal", line=dict(color=IDEAL_C)))
        head.add_trace(go.Scatter(x=t, y=wrap(data["raw_theta"]), name="raw", line=dict(color=RAW_C, dash="dot")))
        head.add_trace(go.Scatter(x=t, y=wrap(data["cor_theta"]), name="corrected", line=dict(color=COR_C, dash="dash")))
        head.update_layout(title="Heading (yaw)", xaxis_title="t [s]", yaxis_title="deg",
                           template="plotly_white", margin=dict(l=40, r=20, t=50, b=40))

        spd = go.Figure()
        spd.add_trace(go.Scatter(x=t, y=data["gt_v"], name="ideal", line=dict(color=IDEAL_C)))
        spd.add_trace(go.Scatter(x=t, y=data["cor_v"], name="corrected", line=dict(color=COR_C, dash="dash")))
        spd.update_layout(title="Speed", xaxis_title="t [s]", yaxis_title="v [m/s]",
                          template="plotly_white", margin=dict(l=40, r=20, t=50, b=40))

        unc = go.Figure()
        unc.add_trace(go.Scatter(x=t, y=data["pos_std"], name="EKF 1σ",
                                 line=dict(color=COR_C), fill="tozeroy"))
        unc.update_layout(title="EKF position uncertainty (1σ)", xaxis_title="t [s]",
                          yaxis_title="σ [m]", template="plotly_white",
                          margin=dict(l=40, r=20, t=50, b=40))

        m = data["metrics"]
        return (perr, head, spd, unc,
                f"{m['rmse_raw']:.2f}", f"{m['rmse_corrected']:.2f}",
                f"{m['improvement_pct']:.0f}", f"{m['final_error_raw']:.2f}",
                f"{m['final_error_corrected']:.2f}", f"{m['travelled_distance']:.0f}")


def main() -> None:
    parser = argparse.ArgumentParser(description="UGV interactive navigation simulator")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8051)
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()
    create_app().run(host=args.host, port=args.port, debug=args.debug)


if __name__ == "__main__":
    main()
