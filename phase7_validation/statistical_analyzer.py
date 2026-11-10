"""
VANET Collision Warning & Traffic Management - Phase 7
Statistical Analyzer & Metrics Aggregator

Computes:
- Mean, Standard Deviation, Min, Max, and 95% Confidence Intervals (CI95)
  across repeated runs (per-event, per-run, and aggregated).
- Direct comparative matrices contrasting Autonomous Response Enabled vs. Disabled.
- Zero-denominator handling reporting "N/A" for undefined metrics.
"""

import os
import math
import numpy as np
import pandas as pd
from typing import Dict, List, Tuple, Any

# Critical t-value for df = 4 (N = 5 runs) at 95% confidence level
T_CRIT_95_DF4 = 2.776


def compute_confidence_interval(series: pd.Series) -> Tuple[float, float, float, float, float]:
    """
    Computes (mean, std, min, max, ci95_margin) for a numeric series.
    Ignores NaN values.
    """
    clean = series.dropna()
    clean = clean[clean >= 0]  # Exclude sentinel -1.0 values if present
    n = len(clean)
    if n == 0:
        return float("nan"), float("nan"), float("nan"), float("nan"), float("nan")
    if n == 1:
        val = float(clean.iloc[0])
        return val, 0.0, val, val, 0.0

    mean_val = float(clean.mean())
    std_val = float(clean.std(ddof=1))
    min_val = float(clean.min())
    max_val = float(clean.max())

    # t-distribution critical value
    t_val = T_CRIT_95_DF4 if n == 5 else 1.96
    ci_margin = t_val * (std_val / math.sqrt(n)) if n > 1 else 0.0

    return mean_val, std_val, min_val, max_val, ci_margin


def generate_statistical_summaries(
    df_runs: pd.DataFrame,
    output_dir: str
) -> Tuple[pd.DataFrame, pd.DataFrame]:
    """
    Aggregates the multi-run dataset into:
    1. Comprehensive scenario summary table (Mean ± 95% CI).
    2. Response Enabled vs. Disabled comparison matrix.
    """
    os.makedirs(output_dir, exist_ok=True)

    metrics_to_aggregate = [
        "Precision", "Recall", "F1", "FalseAlarmRate", "DetectionLatency_s",
        "MinTTC_s", "MeanRiskTTC_s", "MinGap_m", "MaxClosingSpeed_mps",
        "RiskEventsCount", "BrakingActionsCount", "RespondingVehiclesCount",
        "MaxSpeedReduction_kmh", "BrakingResponseTime_ms",
        "WarningsGenerated", "WarningsDelivered", "PDR_Percent",
        "AvgE2EDelay_ms", "AvgHopCount", "RoutingOverhead"
    ]

    summary_records = []

    # Group by Scenario and ResponseEnabled
    grouped = df_runs.groupby(["Scenario", "ResponseEnabled"])

    for (sc, resp_en), group in grouped:
        rec = {
            "Scenario": sc,
            "ResponseEnabled": resp_en,
            "Mode": "Closed-Loop (Response ON)" if resp_en else "Open-Loop (Response OFF)",
            "RunsCount": len(group)
        }

        for m in metrics_to_aggregate:
            if m in group.columns:
                mean_v, std_v, min_v, max_v, ci_m = compute_confidence_interval(group[m])
                rec[f"{m}_Mean"] = round(mean_v, 4) if not math.isnan(mean_v) else np.nan
                rec[f"{m}_Std"] = round(std_v, 4) if not math.isnan(std_v) else np.nan
                rec[f"{m}_Min"] = round(min_v, 2) if not math.isnan(min_v) else np.nan
                rec[f"{m}_Max"] = round(max_v, 2) if not math.isnan(max_v) else np.nan
                rec[f"{m}_CI95_Margin"] = round(ci_m, 4) if not math.isnan(ci_m) else np.nan

        summary_records.append(rec)

    df_summary = pd.DataFrame(summary_records)
    summary_path = os.path.join(output_dir, "phase7_statistical_summary.csv")
    df_summary.to_csv(summary_path, index=False)

    # 2. Build Response Enabled vs. Disabled Contrast Matrix
    comparison_records = []
    scenarios = df_runs["Scenario"].unique()

    for sc in scenarios:
        sub_en = df_runs[(df_runs["Scenario"] == sc) & (df_runs["ResponseEnabled"] == True)]
        sub_dis = df_runs[(df_runs["Scenario"] == sc) & (df_runs["ResponseEnabled"] == False)]

        comp = {
            "Scenario": sc,
            "Baseline_MinGap_m": round(sub_dis["MinGap_m"].replace(-1.0, np.nan).mean(), 2) if not sub_dis.empty else np.nan,
            "ClosedLoop_MinGap_m": round(sub_en["MinGap_m"].replace(-1.0, np.nan).mean(), 2) if not sub_en.empty else np.nan,
            "Gap_Preserved_m": round(
                (sub_en["MinGap_m"].replace(-1.0, np.nan).mean() - sub_dis["MinGap_m"].replace(-1.0, np.nan).mean()), 2
            ) if (not sub_en.empty and not sub_dis.empty) else np.nan,
            "Baseline_BrakingActions": int(sub_dis["BrakingActionsCount"].mean()) if not sub_dis.empty else 0,
            "ClosedLoop_BrakingActions": int(sub_en["BrakingActionsCount"].mean()) if not sub_en.empty else 0,
            "Follower_Speed_Reduction_kmh": round(sub_en["MaxSpeedReduction_kmh"].mean(), 2) if not sub_en.empty else 0.0,
            "Avg_Braking_Response_ms": round(sub_en["BrakingResponseTime_ms"].replace(-1.0, np.nan).mean(), 2) if not sub_en.empty else np.nan,
            "PDR_Percent": round(sub_en["PDR_Percent"].mean(), 1) if not sub_en.empty else np.nan,
            "Avg_Latency_ms": round(sub_en["AvgE2EDelay_ms"].replace(-1.0, np.nan).mean(), 3) if not sub_en.empty else np.nan
        }
        comparison_records.append(comp)

    df_comp = pd.DataFrame(comparison_records)
    comp_path = os.path.join(output_dir, "phase7_response_comparison_matrix.csv")
    df_comp.to_csv(comp_path, index=False)

    print(f"[STATISTICAL SUMMARY] Saved to:\n  - {summary_path}\n  - {comp_path}")
    return df_summary, df_comp
