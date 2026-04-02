from pathlib import Path

import plotly.express as px
import plotly.graph_objects as go
from plotly.subplots import make_subplots
import pandas as pd

from analysis.utils.data_loader import load_simulation_data


def plot_trajectories(df: pd.DataFrame, output_html: str | None = None):
    """2D trajectories of UGV and UAV (true + estimated positions)."""
    fig = go.Figure()

    # Simulated ground truth trajectories
    if all(c in df.columns for c in ["x_eg", "x_ng"]):
        fig.add_trace(
            go.Scatter(
                x=df["x_eg"], y=df["x_ng"], mode="lines", name="UGV Truth", line=dict(color="blue")
            )
        )
    if all(c in df.columns for c in ["x_ea", "x_na"]):
        fig.add_trace(
            go.Scatter(
                x=df["x_ea"], y=df["x_na"], mode="lines", name="UAV Truth", line=dict(color="red")
            )
        )

    # Estimated trajectories
    if all(c in df.columns for c in ["xhat_eg", "xhat_ng"]):
        fig.add_trace(
            go.Scatter(
                x=df["xhat_eg"],
                y=df["xhat_ng"],
                mode="lines",
                name="UGV Estimated",
                line=dict(dash="dash", color="blue"),
            )
        )
    if all(c in df.columns for c in ["xhat_ea", "xhat_na"]):
        fig.add_trace(
            go.Scatter(
                x=df["xhat_ea"],
                y=df["xhat_na"],
                mode="lines",
                name="UAV Estimated",
                line=dict(dash="dash", color="red"),
            )
        )

    fig.update_layout(
        title="UAV & UGV Trajectories (Truth vs Estimated)",
        xaxis_title="East (m)",
        yaxis_title="North (m)",
        legend_title="Legend",
        height=600,
    )
    fig.show()
    if output_html:
        fig.write_html(output_html)
    return fig


def plot_states_vs_time(df: pd.DataFrame, output_html: str | None = None):
    """Interactive time-history plots of true states, estimates, and errors."""
    state_cols = [
        c
        for c in df.columns
        if any(k in c for k in ["x_", "xhat_", "ey_"]) and not c.startswith("p_")
    ]
    fig = make_subplots(
        rows=len(state_cols), cols=1, subplot_titles=state_cols, vertical_spacing=0.05
    )

    for i, col in enumerate(state_cols, 1):
        fig.add_trace(go.Scatter(x=df["time"], y=df[col], mode="lines", name=col), row=i, col=1)

    fig.update_layout(
        title="States, Estimates & Errors vs Time", height=200 * len(state_cols), showlegend=False
    )
    fig.update_xaxes(title_text="Time (s)", row=len(state_cols), col=1)
    fig.show()
    if output_html:
        fig.write_html(output_html)
    return fig


def plot_covariances(df: pd.DataFrame, output_html: str | None = None):
    """Covariance diagonal traces (2-sigma uncertainty bounds)."""
    cov_cols = [c for c in df.columns if c.startswith("p_")]
    if not cov_cols:
        print("No covariance columns found.")
        return None

    fig = go.Figure()
    for col in cov_cols:
        sigma = df[col] ** 0.5 * 2
        fig.add_trace(go.Scatter(x=df["time"], y=sigma, mode="lines", name=f"2-sigma {col}"))
    fig.update_layout(
        title="Covariance Diagonals (2-sigma Bounds)",
        xaxis_title="Time (s)",
        yaxis_title="2-sigma Uncertainty",
        height=500,
    )
    fig.show()
    if output_html:
        fig.write_html(output_html)
    return fig


def plot_all(output_dir: str, save_html: bool = True):
    """Convenience: run all plots from a simulation output directory."""
    df, _ = load_simulation_data(output_dir)
    plots = {
        "trajectories.html": plot_trajectories(df),
        "states_vs_time.html": plot_states_vs_time(df),
        "covariances.html": plot_covariances(df),
    }
    if save_html:
        for fname, fig in plots.items():
            if fig:
                fig.write_html(str(Path(output_dir) / fname))
        print(f"HTML plots saved in {output_dir}")
