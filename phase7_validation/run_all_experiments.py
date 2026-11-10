"""
VANET Collision Warning & Traffic Management - Phase 7
Master Experiment Execution & Analytical Report Pipeline

Coordinates:
1. Execution of full experiment matrix (6 scenarios x 2 modes x 5 runs = 60 runs)
2. Threshold sensitivity evaluation (TTC = 3.0s, 4.0s, 5.0s)
3. Statistical analysis with 95% Confidence Intervals
4. Generation of publication-quality plots
"""

import os
import sys
import argparse
import pandas as pd

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from phase7_validation.experiment_engine import execute_full_phase7_experiments
from phase7_validation.statistical_analyzer import generate_statistical_summaries
from phase7_validation.plot_generator import generate_all_phase7_plots


def main():
    parser = argparse.ArgumentParser(description="Phase 7 Advanced Experiments Runner")
    parser.add_argument("--runs", type=int, default=5, help="Number of runs per scenario condition (default: 5)")
    args = parser.parse_args()

    output_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "results"))
    os.makedirs(output_dir, exist_ok=True)

    # 1. Run Experiments
    df_runs, df_sens, _ = execute_full_phase7_experiments(
        runs_per_scenario=args.runs,
        output_dir=output_dir
    )

    # 2. Statistical Analysis
    df_summary, df_comp = generate_statistical_summaries(
        df_runs=df_runs,
        output_dir=output_dir
    )

    # 3. Generate Visualizations
    generate_all_phase7_plots(
        df_runs=df_runs,
        df_summary=df_summary,
        df_comp=df_comp,
        df_sens=df_sens,
        output_dir=output_dir
    )

    print("\n" + "=" * 80)
    print(" PHASE 7 EXPERIMENT SUITE COMPLETE")
    print("=" * 80)
    print("\n--- Summary Comparison Matrix (Response ON vs OFF) ---")
    print(df_comp[["Scenario", "Baseline_MinGap_m", "ClosedLoop_MinGap_m", "Gap_Preserved_m", "Follower_Speed_Reduction_kmh", "PDR_Percent", "Avg_Latency_ms"]].to_string(index=False))
    print("=" * 80)


if __name__ == "__main__":
    main()
