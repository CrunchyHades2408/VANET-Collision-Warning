"""
VANET Collision Warning & Traffic Management - Phase 7
Publication-Quality Plot Generator

Generates high-resolution, presentation-ready analytical figures:
1. scenario_detection_performance.png: Precision, Recall, F1 with N/A labels.
2. minimum_gap_preservation.png: Gap preservation comparing Response ON vs OFF.
3. network_pdr_and_latency.png: AODV Packet Delivery Ratio and Latency.
4. braking_response_and_speed_reduction.png: Reaction times & speed reduction.
5. comparative_overview_panel.png: Multi-panel comparative evaluation.
6. threshold_sensitivity.png: TTC threshold sensitivity analysis.
"""

import os
import math
from typing import Optional, List, Dict, Tuple, Any
import numpy as np
import pandas as pd

# Set Matplotlib cache directory
os.environ["MPLCONFIGDIR"] = os.path.join(os.path.dirname(__file__), "results", ".matplotlib_cache")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def generate_all_phase7_plots(
    df_runs: pd.DataFrame,
    df_summary: pd.DataFrame,
    df_comp: pd.DataFrame,
    df_sens: Optional[pd.DataFrame],
    output_dir: str
):
    os.makedirs(output_dir, exist_ok=True)
    plt.style.use("seaborn-v0_8-whitegrid" if "seaborn-v0_8-whitegrid" in plt.style.available else "default")

    # -----------------------------------------------------------------------
    # PLOT 1: Scenario Collision Detection Performance (Precision, Recall, F1)
    # -----------------------------------------------------------------------
    plt.figure(figsize=(11, 5.5), dpi=300)
    scenarios = [
        "sudden_braking", "high_speed_approach", "risk_recovery",
        "normal_traffic", "safe_following", "different_lane"
    ]
    labels = [
        "Sudden\nBraking", "High-Speed\nApproach", "Risk\nRecovery",
        "Normal\nTraffic", "Safe\nFollowing", "Different\nLane"
    ]

    # Aggregate detection metrics for Closed-Loop runs
    closed_runs = df_runs[df_runs["ResponseEnabled"] == True]
    prec_vals = []
    rec_vals = []
    f1_vals = []
    prec_annot = []

    for sc in scenarios:
        sub = closed_runs[closed_runs["Scenario"] == sc]
        p = sub["Precision"].dropna().mean() if not sub["Precision"].dropna().empty else float("nan")
        r = sub["Recall"].dropna().mean() if not sub["Recall"].dropna().empty else float("nan")
        f = sub["F1"].dropna().mean() if not sub["F1"].dropna().empty else float("nan")

        prec_vals.append(p * 100 if not math.isnan(p) else 0.0)
        rec_vals.append(r * 100 if not math.isnan(r) else 0.0)
        f1_vals.append(f * 100 if not math.isnan(f) else 0.0)
        prec_annot.append("N/A" if math.isnan(p) else f"{p*100:.1f}%")

    x = np.arange(len(scenarios))
    width = 0.25

    plt.bar(x - width, prec_vals, width, label="Precision", color="#2563EB", alpha=0.9)
    plt.bar(x, rec_vals, width, label="Recall (Sensitivity)", color="#10B981", alpha=0.9)
    plt.bar(x + width, f1_vals, width, label="F1-Score", color="#F59E0B", alpha=0.9)

    # Annotate N/A for undefined precision
    for i, annot in enumerate(prec_annot):
        if annot == "N/A":
            plt.text(x[i] - width, 5.0, "N/A*", ha="center", va="bottom", fontsize=9, fontweight="bold", color="darkred")

    plt.xticks(x, labels, fontsize=10, fontweight="bold")
    plt.ylabel("Performance Score (%)", fontsize=11, fontweight="bold")
    plt.ylim(0, 115)
    plt.title("Phase 7: Collision Detection Performance Across 6 Traffic Scenarios", fontsize=12, fontweight="bold")
    plt.legend(loc="upper right", framealpha=0.9)
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.figtext(0.15, 0.02, "* Note: In normal_traffic, safe_following, and different_lane, zero hazards occur (TP=0, FP=0); Precision is mathematically N/A.", fontsize=8, color="#4B5563")

    p1_path = os.path.join(output_dir, "phase7_scenario_detection_performance.png")
    plt.savefig(p1_path, bbox_inches="tight")
    plt.close()

    # -----------------------------------------------------------------------
    # PLOT 2: Minimum Bumper-to-Bumper Gap Preservation (ON vs OFF)
    # -----------------------------------------------------------------------
    plt.figure(figsize=(10, 5), dpi=300)
    risk_scenarios = ["sudden_braking", "high_speed_approach", "risk_recovery"]
    risk_labels = ["Sudden Braking", "High-Speed Approach", "Risk Recovery"]

    off_gaps = []
    on_gaps = []
    for sc in risk_scenarios:
        s_off = df_runs[(df_runs["Scenario"] == sc) & (df_runs["ResponseEnabled"] == False)]["MinGap_m"]
        s_on = df_runs[(df_runs["Scenario"] == sc) & (df_runs["ResponseEnabled"] == True)]["MinGap_m"]
        off_gaps.append(s_off[s_off >= 0].mean())
        on_gaps.append(s_on[s_on >= 0].mean())

    x = np.arange(len(risk_scenarios))
    width = 0.35

    plt.bar(x - width/2, off_gaps, width, label="Open-Loop Baseline (Response Disabled)", color="#EF4444", alpha=0.85)
    plt.bar(x + width/2, on_gaps, width, label="Closed-Loop System (Autonomous Response Enabled)", color="#10B981", alpha=0.9)

    plt.axhline(2.5, color='darkred', linestyle='--', linewidth=1.8, label="Physical Collision Boundary (2.5m)")

    for i in range(len(risk_scenarios)):
        diff = on_gaps[i] - off_gaps[i]
        plt.text(x[i] + width/2, on_gaps[i] + 0.8, f"+{diff:.1f}m\nBuffer", ha="center", fontsize=9, fontweight="bold", color="#047857")

    plt.xticks(x, risk_labels, fontsize=11, fontweight="bold")
    plt.ylabel("Minimum Bumper-to-Bumper Gap (m)", fontsize=11, fontweight="bold")
    plt.title("Phase 7: Headway Margin Preservation Under Hazardous Scenarios", fontsize=12, fontweight="bold")
    plt.legend(loc="upper right", framealpha=0.9)
    plt.grid(True, linestyle="--", alpha=0.5)

    p2_path = os.path.join(output_dir, "phase7_minimum_gap_preservation.png")
    plt.savefig(p2_path, bbox_inches="tight")
    plt.close()

    # -----------------------------------------------------------------------
    # PLOT 3: Network Performance (PDR & End-to-End Latency)
    # -----------------------------------------------------------------------
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5), dpi=300)

    # Subplot A: PDR
    all_pdr = [
        df_runs[df_runs["Scenario"] == sc]["PDR_Percent"].mean()
        for sc in risk_scenarios
    ]
    ax1.bar(risk_labels, all_pdr, color="#3B82F6", width=0.45, alpha=0.9)
    ax1.set_ylim(0, 110)
    ax1.set_ylabel("Packet Delivery Ratio (%)", fontweight="bold", fontsize=10)
    ax1.set_title("AODV Packet Delivery Ratio", fontweight="bold", fontsize=11)
    ax1.grid(True, linestyle="--", alpha=0.5)
    for i, v in enumerate(all_pdr):
        ax1.text(i, v + 2, f"{v:.1f}%", ha="center", fontweight="bold")

    # Subplot B: End-to-End Latency
    delays = [
        df_runs[df_runs["Scenario"] == sc]["AvgE2EDelay_ms"].replace(-1.0, np.nan).dropna().mean()
        for sc in risk_scenarios
    ]
    delays_std = [
        df_runs[df_runs["Scenario"] == sc]["AvgE2EDelay_ms"].replace(-1.0, np.nan).dropna().std()
        for sc in risk_scenarios
    ]
    ax2.bar(risk_labels, delays, yerr=delays_std, capsize=5, color="#8B5CF6", width=0.45, alpha=0.9)
    ax2.set_ylabel("End-to-End Latency (ms)", fontweight="bold", fontsize=10)
    ax2.set_title("Average Warning Dissemination Latency", fontweight="bold", fontsize=11)
    ax2.grid(True, linestyle="--", alpha=0.5)
    for i, v in enumerate(delays):
        ax2.text(i, v + 0.15, f"{v:.2f} ms", ha="center", fontweight="bold")

    plt.suptitle("Phase 7: AODV Network Communication Performance (RFC 3561 Model)", fontsize=13, fontweight="bold")
    p3_path = os.path.join(output_dir, "phase7_network_pdr_and_latency.png")
    plt.savefig(p3_path, bbox_inches="tight")
    plt.close()

    # -----------------------------------------------------------------------
    # PLOT 4: Follower Speed Reduction & Braking Actions
    # -----------------------------------------------------------------------
    plt.figure(figsize=(9, 4.5), dpi=300)
    speed_reds = [
        df_runs[(df_runs["Scenario"] == sc) & (df_runs["ResponseEnabled"] == True)]["MaxSpeedReduction_kmh"].mean()
        for sc in risk_scenarios
    ]
    speed_stds = [
        df_runs[(df_runs["Scenario"] == sc) & (df_runs["ResponseEnabled"] == True)]["MaxSpeedReduction_kmh"].std()
        for sc in risk_scenarios
    ]

    plt.bar(risk_labels, speed_reds, yerr=speed_stds, capsize=5, color="#D97706", width=0.45, alpha=0.9)
    plt.ylabel("Maximum Speed Reduction (km/h)", fontweight="bold", fontsize=11)
    plt.title("Phase 7: Autonomous Follower Deceleration Response", fontsize=12, fontweight="bold")
    plt.grid(True, linestyle="--", alpha=0.5)
    for i, v in enumerate(speed_reds):
        plt.text(i, v + 0.4, f"Δv = {v:.1f} km/h", ha="center", fontweight="bold")

    p4_path = os.path.join(output_dir, "phase7_braking_response_and_speed_reduction.png")
    plt.savefig(p4_path, bbox_inches="tight")
    plt.close()

    # -----------------------------------------------------------------------
    # PLOT 5: Threshold Sensitivity Curves (if available)
    # -----------------------------------------------------------------------
    if df_sens is not None and not df_sens.empty:
        plt.figure(figsize=(9, 4.5), dpi=300)
        th_grouped = df_sens.groupby("TTCThreshold_s").mean(numeric_only=True).reset_index()

        plt.plot(th_grouped["TTCThreshold_s"], th_grouped["DetectionLatency_s"], 'ro-', label="Detection Latency (s)", linewidth=2.2)
        plt.plot(th_grouped["TTCThreshold_s"], th_grouped["WarningsGenerated"], 'bs--', label="Warnings Generated (count)", linewidth=2.0)

        plt.xlabel("TTC Warning Threshold (seconds)", fontweight="bold", fontsize=11)
        plt.ylabel("Metric Magnitude", fontweight="bold", fontsize=11)
        plt.title("Phase 7: Sensitivity Analysis Across Warning Thresholds (sudden_braking)", fontsize=12, fontweight="bold")
        plt.legend(loc="upper left", framealpha=0.9)
        plt.grid(True, linestyle="--", alpha=0.5)

        p5_path = os.path.join(output_dir, "phase7_threshold_sensitivity_curves.png")
        plt.savefig(p5_path, bbox_inches="tight")
        plt.close()

    print(f"[PLOT GENERATION] Successfully created presentation-ready figures in: {output_dir}")
