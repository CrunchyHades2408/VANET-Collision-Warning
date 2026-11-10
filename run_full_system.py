"""
VANET Collision Warning & Traffic Management Using AODV Routing
Master Integration & Unified System Pipeline (Phase 8)

Executes the complete end-to-end 10-step vehicular safety workflow:
1. System & Environment Validation (SUMO, TraCI, Python Stack)
2. Road Traffic Simulation Initialization (Eclipse SUMO 1.28.0)
3. Real-Time Microscopic Telemetry Extraction (TraCI)
4. Same-Lane Rear-End Collision Hazard Detection (TTC, Gap, Closing Speed)
5. Structured Warning Message Generation (ETSI DENM Standard)
6. Ad-Hoc Wireless Dissemination (RFC 3561 AODV Multi-Hop Routing)
7. On-Board Unit Warning Delivery Confirmation & Validation
8. Autonomous Braking Actuation & Deceleration Monitoring (SUMO slowDown)
9. Normal Car-Following Speed Control Recovery (SUMO setSpeed -1)
10. Telemetry Logging, Statistical Aggregation & Dashboard Dataset Preparation
"""

import os
import sys
import time
import argparse
import pandas as pd
from datetime import datetime

# Setup paths
PROJECT_ROOT = os.path.abspath(os.path.dirname(__file__))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

if "SUMO_HOME" not in os.environ:
    os.environ["SUMO_HOME"] = r"C:\Program Files (x86)\SUMO"
tools_dir = os.path.join(os.environ["SUMO_HOME"], "tools")
if tools_dir not in sys.path:
    sys.path.insert(0, tools_dir)

import sumolib
import traci

# Phase modules
from phase6_autonomous_response.phase6_cosimulation import (
    run_phase6_simulation, export_phase6_results, generate_phase6_comparison_plots
)
from phase7_validation.experiment_engine import execute_full_phase7_experiments
from phase7_validation.statistical_analyzer import generate_statistical_summaries
from phase7_validation.plot_generator import generate_all_phase7_plots


def verify_environment() -> bool:
    """Step 1: Check environment dependencies and tool versions."""
    print("=" * 80)
    print(" STEP 1: VERIFYING SIMULATION ENVIRONMENT & DEPENDENCIES")
    print("=" * 80)
    print(f" Python Version      : {sys.version.split()[0]}")
    print(f" SUMO_HOME           : {os.environ.get('SUMO_HOME', 'NOT SET')}")

    try:
        sumo_bin = sumolib.checkBinary("sumo")
        print(f" SUMO Binary Found   : {sumo_bin}")
    except Exception as e:
        print(f" ERROR: SUMO binary not found: {e}")
        return False

    required_pkgs = ["traci", "pandas", "matplotlib", "plotly", "streamlit"]
    for pkg in required_pkgs:
        try:
            __import__(pkg)
            print(f" Python Package OK   : {pkg}")
        except ImportError:
            print(f" ERROR: Missing required Python package: {pkg}")
            return False

    print("\n [OK] All environment prerequisites verified successfully.\n")
    return True


def execute_master_pipeline(
    scenario: str = "sudden_braking",
    run_experiments: bool = True,
    runs_count: int = 5,
    gui: bool = False
):
    """
    Executes the integrated closed-loop pipeline and populates final deliverables.
    """
    start_time = time.time()
    final_results_dir = os.path.join(PROJECT_ROOT, "final_results")
    logs_dir = os.path.join(final_results_dir, "logs")
    figures_dir = os.path.join(final_results_dir, "figures")
    summaries_dir = os.path.join(final_results_dir, "summaries")

    for d in (logs_dir, figures_dir, summaries_dir):
        os.makedirs(d, exist_ok=True)

    print("=" * 80)
    print(" VANET COLLISION WARNING & TRAFFIC MANAGEMENT USING AODV ROUTING")
    print(" MASTER SYSTEM EXECUTION PIPELINE (PHASES 1 - 8 INTEGRATED)")
    print(f" Execution Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 80)

    # 1. Environment Verification
    if not verify_environment():
        sys.exit(1)

    # 2. Closed-Loop Execution (Primary Benchmark)
    print("=" * 80)
    print(" STEPS 2-9: EXECUTING CLOSED-LOOP VANET SIMULATION (SCENARIO: sudden_braking)")
    print("=" * 80)

    print("\n>>> Running Closed-Loop System (Autonomous Response ENABLED) ...")
    cosim_enabled, metrics_enabled = run_phase6_simulation(
        scenario_name=scenario,
        autonomous_response_enabled=True,
        gui=gui,
        verbose=True
    )
    export_phase6_results(cosim_enabled, logs_dir)

    print("\n>>> Running Open-Loop Baseline (Autonomous Response DISABLED) ...")
    cosim_disabled, metrics_disabled = run_phase6_simulation(
        scenario_name=scenario,
        autonomous_response_enabled=False,
        gui=gui,
        verbose=False
    )

    # Generate comparative figures
    generate_phase6_comparison_plots(cosim_enabled, cosim_disabled, figures_dir)
    print(f"\n [OK] Closed-Loop & Baseline Simulations Completed.")

    # 3. Multi-Run Experiments & Statistical Analysis (Phase 7)
    if run_experiments:
        print("\n" + "=" * 80)
        print(f" STEP 10: RUNNING MULTI-SEED EXPERIMENTS & STATISTICAL AGGREGATION ({runs_count} RUNS)")
        print("=" * 80)

        df_runs, df_sens, _ = execute_full_phase7_experiments(
            runs_per_scenario=runs_count,
            output_dir=logs_dir
        )

        df_summary, df_comp = generate_statistical_summaries(
            df_runs=df_runs,
            output_dir=summaries_dir
        )

        generate_all_phase7_plots(
            df_runs=df_runs,
            df_summary=df_summary,
            df_comp=df_comp,
            df_sens=df_sens,
            output_dir=figures_dir
        )

        print("\n" + "=" * 80)
        print(" FINAL COMPARATIVE MATRIX (RESPONSE ENABLED VS. DISABLED)")
        print("=" * 80)
        print(df_comp[["Scenario", "Baseline_MinGap_m", "ClosedLoop_MinGap_m", "Gap_Preserved_m", "Follower_Speed_Reduction_kmh", "PDR_Percent", "Avg_Latency_ms"]].to_string(index=False))

    elapsed = time.time() - start_time
    print("\n" + "=" * 80)
    print(f" MASTER INTEGRATION PIPELINE COMPLETED SUCCESSFULLY IN {elapsed:.1f} SECONDS")
    print(f" All Deliverables Consolidated in: {final_results_dir}")
    print("=" * 80)
    print("\nTo launch the interactive dashboard, run:")
    print("   streamlit run dashboard/app.py\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Master VANET System Integration Runner")
    parser.add_argument("--scenario", type=str, default="sudden_braking")
    parser.add_argument("--skip-experiments", action="store_true", help="Skip 60-run matrix and run benchmark only")
    parser.add_argument("--runs", type=int, default=5, help="Number of runs per scenario condition (default: 5)")
    parser.add_argument("--gui", action="store_true", help="Launch SUMO with GUI")
    args = parser.parse_args()

    execute_master_pipeline(
        scenario=args.scenario,
        run_experiments=not args.skip_experiments,
        runs_count=args.runs,
        gui=args.gui
    )
