"""
VANET Collision Warning & Traffic Management - Reusable Plotly Chart Components
"""

import pandas as pd
import numpy as np
import plotly.express as px
import plotly.graph_objects as go
from typing import Optional


def plot_speed_trajectories(df: pd.DataFrame, title: str = "Vehicle Speed Over Time") -> go.Figure:
    """Plots speed variation across vehicles with unified hover."""
    fig = px.line(
        df,
        x="timestamp",
        y="speed_kmh",
        color="vehicle_id",
        title=title,
        labels={"timestamp": "Simulation Time (s)", "speed_kmh": "Speed (km/h)", "vehicle_id": "Vehicle"},
        template="plotly_dark",
        color_discrete_sequence=px.colors.qualitative.Plotly
    )
    fig.update_layout(hovermode="x unified", legend_title="Vehicle ID")
    return fig


def plot_position_trajectories(df: pd.DataFrame, title: str = "Longitudinal Vehicle Positions") -> go.Figure:
    """Plots vehicle longitudinal displacement along highway corridor."""
    fig = px.line(
        df,
        x="timestamp",
        y="pos_x",
        color="vehicle_id",
        title=title,
        labels={"timestamp": "Simulation Time (s)", "pos_x": "Position X (m)", "vehicle_id": "Vehicle"},
        template="plotly_dark"
    )
    fig.update_layout(hovermode="x unified")
    return fig


def plot_gap_preservation_comparison(df_comp: pd.DataFrame) -> go.Figure:
    """Contrasts Minimum Gap between Baseline (OFF) and Closed-Loop (ON)."""
    fig = go.Figure()
    fig.add_trace(go.Bar(
        x=df_comp["Scenario"],
        y=df_comp["Baseline_MinGap_m"],
        name="Open-Loop Baseline (Response OFF)",
        marker_color="#EF4444"
    ))
    fig.add_trace(go.Bar(
        x=df_comp["Scenario"],
        y=df_comp["ClosedLoop_MinGap_m"],
        name="Closed-Loop System (Response ON)",
        marker_color="#10B981"
    ))
    # Add 2.5m collision threshold line
    fig.add_hline(y=2.5, line_dash="dash", line_color="darkred", annotation_text="Physical Collision Limit (2.5m)")

    fig.update_layout(
        title="Minimum Bumper-to-Bumper Gap Preservation (Response ON vs. OFF)",
        xaxis_title="Traffic Scenario",
        yaxis_title="Minimum Headway Gap (m)",
        barmode="group",
        template="plotly_dark",
        legend=dict(orientation="h", yanchor="bottom", y=1.02, xanchor="right", x=1)
    )
    return fig


def plot_speed_reduction_bars(df_comp: pd.DataFrame) -> go.Figure:
    """Visualizes speed reduction (Δv) achieved by responding follower vehicles."""
    fig = px.bar(
        df_comp,
        x="Scenario",
        y="Follower_Speed_Reduction_kmh",
        color="Follower_Speed_Reduction_kmh",
        color_continuous_scale="Viridis",
        title="Follower Speed Reduction After Warning Receipt (Δv)",
        labels={"Follower_Speed_Reduction_kmh": "Speed Reduction (km/h)", "Scenario": "Traffic Scenario"},
        template="plotly_dark"
    )
    fig.update_layout(coloraxis_showscale=False)
    return fig


def plot_causal_sequence_timeline(df_causal: pd.DataFrame) -> go.Figure:
    """Visualizes the 8-step causal sequence progression."""
    fig = px.scatter(
        df_causal,
        x="timestamp",
        y="step_name",
        color="vehicle_id",
        size="step_number",
        hover_data=["details", "speed_mps", "gap_m"],
        title="Chronological 8-Step Causal Sequence (Hazard to Safe Recovery)",
        labels={"timestamp": "Simulation Time (s)", "step_name": "Causal Sequence Step", "vehicle_id": "Vehicle"},
        template="plotly_dark"
    )
    fig.update_traces(marker=dict(size=14, line=dict(width=2, color="white")))
    fig.update_layout(yaxis=dict(categoryorder="total ascending"))
    return fig


def plot_latency_distribution(df_runs: pd.DataFrame) -> go.Figure:
    """Plots distribution of End-to-End AODV delivery latencies across scenarios."""
    clean = df_runs[df_runs["AvgE2EDelay_ms"] > 0]
    fig = px.box(
        clean,
        x="Scenario",
        y="AvgE2EDelay_ms",
        color="Scenario",
        title="AODV End-to-End Delivery Latency Distribution (RFC 3561 Model)",
        labels={"AvgE2EDelay_ms": "Latency (ms)", "Scenario": "Scenario"},
        template="plotly_dark"
    )
    fig.update_layout(showlegend=False)
    return fig
