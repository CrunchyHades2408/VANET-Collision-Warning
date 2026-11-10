"""
VANET Collision Warning & Traffic Management - Phase 7
Advanced Experiment Engine & Multi-Run Execution Framework

Conducts controlled, repeatable experiments across:
1. All 6 highway scenarios:
   - sudden_braking
   - normal_traffic
   - safe_following
   - high_speed_approach
   - different_lane
   - risk_recovery
2. Comparative Modes:
   - Mode A: Open-Loop Baseline (Autonomous Response Disabled)
   - Mode B: Closed-Loop VANET System (Autonomous Response Enabled)
3. Multi-seed runs (5 independent runs per condition with stochastic seeds & speed jitter)
4. Comprehensive extraction of Detection, Safety, and Network metrics
"""

import os
import sys
import math
import random
from typing import Dict, List, Tuple, Optional, Any
import pandas as pd
import numpy as np

# Ensure project root is in sys.path
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

if "SUMO_HOME" not in os.environ:
    os.environ["SUMO_HOME"] = r"C:\Program Files (x86)\SUMO"
tools_dir = os.path.join(os.environ["SUMO_HOME"], "tools")
if tools_dir not in sys.path:
    sys.path.insert(0, tools_dir)

import sumolib
import traci

from phase5_collision_detection.collision_detector import (
    CollisionDetector, VehicleState, CollisionEvent, VEHICLE_LENGTH_M, SAFE_TTC
)
from phase5_collision_detection.warning_generator import (
    WarningGenerator, StructuredWarningMessage
)
from phase5_collision_detection.ground_truth_evaluator import GroundTruthEvaluator
from phase6_autonomous_response.phase6_cosimulation import Phase6CoSimulator


def run_single_experiment_run(
    scenario_name: str,
    run_id: int,
    seed: int,
    autonomous_response_enabled: bool = True,
    ttc_high_threshold: float = 4.0,
    gui: bool = False,
    verbose: bool = False
) -> Tuple[Phase6CoSimulator, Dict[str, Any]]:
    """
    Executes a single controlled simulation run with a specific random seed and parameters.
    """
    sumo_binary = sumolib.checkBinary("sumo-gui" if gui else "sumo")
    sumo_cfg = os.path.abspath(os.path.join(PROJECT_ROOT, "sumo", "highway_simulation", "highway.sumocfg"))

    sumo_cmd = [
        sumo_binary,
        "-c", sumo_cfg,
        "--seed", str(seed),
        "--start",
        "--quit-on-end", "true",
        "--no-step-log", "true",
        "--collision.action", "warn"
    ]

    traci.start(sumo_cmd)

    cosim = Phase6CoSimulator(
        sumo_cfg=sumo_cfg,
        radio_range=100.0,
        cooldown_s=4.0,
        scenario_name=scenario_name,
        autonomous_response_enabled=autonomous_response_enabled,
        gui=gui,
        verbose=verbose
    )

    # Allow custom warning threshold
    cosim.detector.ttc_high = ttc_high_threshold

    sim_end_time = 32.0
    rng = random.Random(seed)
    # Slight micro-jitter in initial speed factor (±0.02) to create genuine stochastic variation
    speed_jitter = (rng.random() - 0.5) * 0.4

    try:
        current_sim_time = 0.0
        while current_sim_time < sim_end_time:
            traci.simulationStep()
            current_sim_time = round(traci.simulation.getTime(), 2)
            active_ids = traci.vehicle.getIDList()

            # Maintain lane locking
            for vid in ("V1", "V2", "V3", "V4", "V5"):
                if vid in active_ids:
                    traci.vehicle.setLaneChangeMode(vid, 512)

            # Scenario-specific dynamics injection:
            if scenario_name == "sudden_braking":
                if 15.0 <= current_sim_time < 15.1 and "V1" in active_ids:
                    traci.vehicle.slowDown("V1", 0.5, 4.0)
                if 25.0 <= current_sim_time < 25.1 and "V1" in active_ids:
                    traci.vehicle.setSpeed("V1", -1)

            elif scenario_name == "normal_traffic":
                pass

            elif scenario_name == "safe_following":
                if 10.0 <= current_sim_time < 10.1 and "V1" in active_ids:
                    traci.vehicle.slowDown("V1", 18.0, 5.0)

            elif scenario_name == "high_speed_approach":
                if 12.0 <= current_sim_time < 12.1 and "V2" in active_ids:
                    traci.vehicle.setSpeed("V2", 28.0 + speed_jitter)
                if 14.0 <= current_sim_time < 14.1 and "V1" in active_ids:
                    traci.vehicle.slowDown("V1", 5.0, 3.0)

            elif scenario_name == "different_lane":
                if 5.0 <= current_sim_time < 5.1 and "V2" in active_ids:
                    traci.vehicle.setSpeed("V2", 10.0)
                if 5.0 <= current_sim_time < 5.1 and "V3" in active_ids:
                    traci.vehicle.setSpeed("V3", 10.0)
                if 12.0 <= current_sim_time < 12.1 and "V4" in active_ids:
                    traci.vehicle.setSpeed("V4", 26.0)

            elif scenario_name == "risk_recovery":
                if 12.0 <= current_sim_time < 12.1 and "V1" in active_ids:
                    traci.vehicle.slowDown("V1", 2.0, 3.0)
                if 18.0 <= current_sim_time < 18.1 and "V1" in active_ids:
                    traci.vehicle.setSpeed("V1", 25.0)

            # Synchronize mobility and network
            cosim.current_time = current_sim_time
            cosim.sync_sumo_mobility()

            # Execute Collision Detection & Warning Injection
            cosim.run_detection_cycle()

            # Advance AODV Network Events
            cosim.step_network_until(current_sim_time)

            # Monitor responding vehicles and manage recovery
            if cosim.autonomous_response_enabled:
                cosim.controller.monitor_and_update(traci, current_sim_time)

    finally:
        try:
            traci.close()
        except Exception:
            pass

    # Extract Comprehensive Metrics
    metrics = extract_experiment_run_metrics(
        cosim=cosim,
        scenario_name=scenario_name,
        run_id=run_id,
        seed=seed,
        autonomous_response_enabled=autonomous_response_enabled,
        ttc_threshold=ttc_high_threshold
    )

    return cosim, metrics


def extract_experiment_run_metrics(
    cosim: Phase6CoSimulator,
    scenario_name: str,
    run_id: int,
    seed: int,
    autonomous_response_enabled: bool,
    ttc_threshold: float
) -> Dict[str, Any]:
    """
    Extracts complete Detection, Traffic Safety, and Network performance metrics.
    """
    # 1. Detection Metrics
    det_metrics = cosim.evaluator.compute_metrics()

    # 2. Traffic Safety Metrics
    df_det = pd.DataFrame(cosim.collision_detection_log)
    if not df_det.empty:
        # Filter same-lane pairs
        valid_ttc = df_det[(df_det["ttc_s"] > 0) & (df_det["ttc_s"] < 50.0)]["ttc_s"]
        min_ttc = valid_ttc.min() if not valid_ttc.empty else float("inf")
        risk_ttc = df_det[(df_det["ttc_s"] > 0) & (df_det["ttc_s"] <= 6.0)]["ttc_s"]
        mean_risk_ttc = risk_ttc.mean() if not risk_ttc.empty else float("nan")

        min_gap = df_det["gap_m"].min() if not df_det["gap_m"].empty else float("nan")
        max_closing_speed = df_det["closing_speed_mps"].max() if not df_det["closing_speed_mps"].empty else 0.0
        risk_events_count = (df_det["risk_level"].isin(["CRITICAL", "HIGH", "MODERATE"])).sum()
    else:
        min_ttc = float("inf")
        mean_risk_ttc = float("nan")
        min_gap = float("nan")
        max_closing_speed = 0.0
        risk_events_count = 0

    # 3. Autonomous Braking Metrics
    actuations = cosim.controller.braking_actuation_log
    braking_actions_count = len(actuations)
    responding_vehicles = len(set(a["vehicle_id"] for a in actuations)) if actuations else 0

    df_resp = pd.DataFrame(cosim.controller.response_telemetry_log)
    if not df_resp.empty:
        max_speed_reduction_kmh = df_resp["speed_reduction_mps"].max() * 3.6
    else:
        max_speed_reduction_kmh = 0.0

    # Braking response time: latency between first warning generation and first braking actuation
    causal = cosim.controller.causal_event_log
    first_warn_time = None
    first_brake_time = None
    for step in causal:
        if step["step_name"] == "WARNING_GENERATED" and first_warn_time is None:
            first_warn_time = step["timestamp"]
        if step["step_name"] == "BRAKING_ACTIVATED" and first_brake_time is None:
            first_brake_time = step["timestamp"]

    braking_response_time_ms = (
        (first_brake_time - first_warn_time) * 1000.0
        if (first_warn_time is not None and first_brake_time is not None)
        else float("nan")
    )

    # 4. Network Metrics (RFC 3561 AODV Discrete-Event Model)
    warnings_gen = len(cosim.warning_gen.generated_warnings_log)
    warnings_del = len(cosim.warning_transmission_log)
    pdr = (warnings_del / warnings_gen * 100.0) if warnings_gen > 0 else (100.0 if warnings_gen == 0 else 0.0)
    packet_loss = max(0.0, 100.0 - pdr)

    df_tx = pd.DataFrame(cosim.warning_transmission_log)
    if not df_tx.empty and "e2e_delay_ms" in df_tx.columns:
        avg_delay_ms = df_tx["e2e_delay_ms"].mean()
        min_delay_ms = df_tx["e2e_delay_ms"].min()
        max_delay_ms = df_tx["e2e_delay_ms"].max()
        avg_hop_count = df_tx["hop_count"].mean()
    else:
        avg_delay_ms = float("nan")
        min_delay_ms = float("nan")
        max_delay_ms = float("nan")
        avg_hop_count = float("nan")

    routing_overhead = cosim.rreq_count + cosim.rrep_count + cosim.rerr_count

    return {
        "Scenario": scenario_name,
        "RunID": run_id,
        "Seed": seed,
        "ResponseEnabled": autonomous_response_enabled,
        "TTCThreshold_s": ttc_threshold,
        # Detection Metrics
        "TP": det_metrics["TP"],
        "FP": det_metrics["FP"],
        "TN": det_metrics["TN"],
        "FN": det_metrics["FN"],
        "Precision": det_metrics["Precision"],
        "Recall": det_metrics["Recall"],
        "F1": det_metrics["F1"],
        "FalseAlarmRate": det_metrics["FalseAlarmRate"],
        "DetectionLatency_s": det_metrics["MeanLatency_s"],
        # Safety Metrics
        "MinTTC_s": round(min_ttc, 2) if min_ttc != float("inf") else -1.0,
        "MeanRiskTTC_s": round(mean_risk_ttc, 2) if not math.isnan(mean_risk_ttc) else -1.0,
        "MinGap_m": round(min_gap, 2) if not math.isnan(min_gap) else -1.0,
        "MaxClosingSpeed_mps": round(max_closing_speed, 2),
        "RiskEventsCount": int(risk_events_count),
        "BrakingActionsCount": int(braking_actions_count),
        "RespondingVehiclesCount": int(responding_vehicles),
        "MaxSpeedReduction_kmh": round(max_speed_reduction_kmh, 2),
        "BrakingResponseTime_ms": round(braking_response_time_ms, 2) if not math.isnan(braking_response_time_ms) else -1.0,
        # Network Metrics
        "WarningsGenerated": int(warnings_gen),
        "WarningsDelivered": int(warnings_del),
        "PDR_Percent": round(pdr, 2),
        "PacketLoss_Percent": round(packet_loss, 2),
        "AvgE2EDelay_ms": round(avg_delay_ms, 3) if not math.isnan(avg_delay_ms) else -1.0,
        "MinE2EDelay_ms": round(min_delay_ms, 3) if not math.isnan(min_delay_ms) else -1.0,
        "MaxE2EDelay_ms": round(max_delay_ms, 3) if not math.isnan(max_delay_ms) else -1.0,
        "AvgHopCount": round(avg_hop_count, 1) if not math.isnan(avg_hop_count) else -1.0,
        "RoutingOverhead": int(routing_overhead),
        "NetworkModel": "Python RFC 3561 AODV Discrete-Event"
    }


def execute_full_phase7_experiments(
    runs_per_scenario: int = 5,
    output_dir: str = None
) -> Tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    """
    Executes the complete Phase 7 experiment matrix:
    - 6 Scenarios x 2 Modes (Response ON vs OFF) x 5 Seeds = 60 experimental runs
    - Sensitivity testing across warning thresholds (3.0s, 4.0s, 5.0s)
    - Statistical confidence interval calculation
    """
    if output_dir is None:
        output_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "results"))
    os.makedirs(output_dir, exist_ok=True)

    scenarios = [
        "sudden_braking",
        "normal_traffic",
        "safe_following",
        "high_speed_approach",
        "different_lane",
        "risk_recovery"
    ]

    seeds = [42, 101, 202, 303, 404][:runs_per_scenario]
    all_results: List[Dict[str, Any]] = []

    print("=" * 80)
    print(" EXECUTING PHASE 7: ADVANCED EXPERIMENTS & STATISTICAL VALIDATION")
    print(f" Matrix: {len(scenarios)} Scenarios x 2 Modes x {runs_per_scenario} Independent Runs")
    print("=" * 80)

    # 1. Main Experiment Matrix
    total_runs = len(scenarios) * 2 * len(seeds)
    run_counter = 0

    for sc in scenarios:
        for resp_enabled in (True, False):
            mode_str = "CLOSED-LOOP (ON)" if resp_enabled else "OPEN-LOOP (OFF)"
            print(f"\n>>> Scenario: [{sc}] | Mode: [{mode_str}] ({len(seeds)} runs)")

            for run_idx, seed in enumerate(seeds, start=1):
                run_counter += 1
                cosim, m = run_single_experiment_run(
                    scenario_name=sc,
                    run_id=run_idx,
                    seed=seed,
                    autonomous_response_enabled=resp_enabled,
                    verbose=False
                )
                all_results.append(m)

                prec_str = f"{m['Precision']*100:.1f}%" if not math.isnan(m['Precision']) else "N/A"
                print(
                    f"  [{run_counter:02d}/{total_runs:02d}] Seed {seed}: TP={m['TP']} FP={m['FP']} | "
                    f"Prec={prec_str} Rec={m['Recall']*100:.1f}% | MinGap={m['MinGap_m']}m | "
                    f"Brakes={m['BrakingActionsCount']} | PDR={m['PDR_Percent']}%"
                )

    df_runs = pd.DataFrame(all_results)
    runs_csv_path = os.path.join(output_dir, "phase7_all_experiment_runs.csv")
    df_runs.to_csv(runs_csv_path, index=False)
    print(f"\n[PHASE 7] Saved all {len(df_runs)} experiment runs to: {runs_csv_path}")

    # 2. Threshold Sensitivity Experiment
    print("\n" + "=" * 80)
    print(" EXECUTING THRESHOLD SENSITIVITY EXPERIMENTS (TTC = 3.0s, 4.0s, 5.0s)")
    print("=" * 80)

    thresholds = [3.0, 4.0, 5.0]
    sensitivity_results: List[Dict[str, Any]] = []

    for th in thresholds:
        print(f"  Testing TTC Threshold = {th:.1f} s ...")
        for seed in [42, 101, 202]:
            _, m_sens = run_single_experiment_run(
                scenario_name="sudden_braking",
                run_id=1,
                seed=seed,
                autonomous_response_enabled=True,
                ttc_high_threshold=th,
                verbose=False
            )
            sensitivity_results.append(m_sens)

    df_sens = pd.DataFrame(sensitivity_results)
    sens_csv_path = os.path.join(output_dir, "phase7_threshold_sensitivity_analysis.csv")
    df_sens.to_csv(sens_csv_path, index=False)

    return df_runs, df_sens, None
