"""
VANET Collision Warning & Traffic Management - Phase 5
Integrated Co-Simulation Runner & Verification Pipeline

Couples:
1. SUMO TraCI Microscopic Traffic Simulation
2. Real-time Vehicle State Extraction
3. Same-Lane Rear-End Collision Detection (TTC, Gap, Closing Speed)
4. Structured Warning Message Generation (ETSI DENM format)
5. RFC 3561 AODV Multi-Hop Wireless Network Protocol Engine
6. Ground-Truth Metrics Evaluation (Precision, Recall, F1)
7. Automated Verification Across 6 Named Test Scenarios
"""

import os
import sys
import math
import heapq
import argparse
from typing import Dict, List, Tuple, Optional
import pandas as pd

# Set Matplotlib cache directory
os.environ["MPLCONFIGDIR"] = os.path.join(os.path.dirname(__file__), "results", ".matplotlib_cache")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# TraCI & SUMO Setup
if "SUMO_HOME" not in os.environ:
    os.environ["SUMO_HOME"] = r"C:\Program Files (x86)\SUMO"

tools_dir = os.path.join(os.environ["SUMO_HOME"], "tools")
if tools_dir not in sys.path:
    sys.path.insert(0, tools_dir)

import sumolib
import traci

# Ensure project root is in sys.path
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

# Phase 5 Components
from phase5_collision_detection.collision_detector import (
    CollisionDetector, VehicleState, CollisionEvent, VEHICLE_LENGTH_M, SAFE_TTC
)
from phase5_collision_detection.warning_generator import (
    WarningGenerator, StructuredWarningMessage
)
from phase5_collision_detection.ground_truth_evaluator import GroundTruthEvaluator

# Re-use Phase 4 AODV Data Structures & Logic
from omnetpp.veins_integration.run_veins_cosimulation import (
    MobileNodeState, RoutingTableEntry, AodvRreq, AodvRrep, AodvRerr,
    UdpWarningPacket, SimEvent, VeinsCoSimulator
)


class Phase5CoSimulator(VeinsCoSimulator):
    """
    Extends Phase 4 VeinsCoSimulator with:
    - Collision Detection per simulation step
    - Warning Generation and direct injection into AODV
    - Comprehensive CSV Telemetry logging
    - Ground-truth evaluation
    """
    def __init__(
        self,
        sumo_cfg: str,
        radio_range: float = 100.0,
        cooldown_s: float = 4.0,
        scenario_name: str = "sudden_braking",
        gui: bool = False
    ):
        super().__init__(sumo_cfg=sumo_cfg, radio_range=radio_range, gui=gui)
        self.scenario_name = scenario_name

        # Mapping for IP lookup
        self.vehicle_ip_map = {vid: ip for vid, (omnet_id, ip) in self.vehicle_mapping.items()}

        # Phase 5 Modules
        self.detector = CollisionDetector(
            vehicle_ip_map=self.vehicle_ip_map,
            cooldown_s=cooldown_s,
            verbose=False
        )
        self.warning_gen = WarningGenerator(verbose=False)
        self.evaluator = GroundTruthEvaluator()

        # Telemetry Logs
        self.vehicle_state_log: List[dict] = []
        self.collision_detection_log: List[dict] = []
        self.warning_transmission_log: List[dict] = []

    def log_vehicle_states(self, vehicle_states: Dict[str, VehicleState]):
        for vs in vehicle_states.values():
            self.vehicle_state_log.append({
                "timestamp": round(vs.timestamp, 2),
                "vehicle_id": vs.vehicle_id,
                "pos_x": round(vs.pos_x, 2),
                "pos_y": round(vs.pos_y, 2),
                "speed_mps": round(vs.speed_mps, 2),
                "speed_kmh": round(vs.speed_kmh, 2),
                "lane_id": vs.lane_id,
                "edge_id": vs.edge_id
            })

    def run_detection_cycle(self):
        # 1. Build VehicleState snapshots from active SUMO vehicles
        active_ids = traci.vehicle.getIDList()
        vehicle_states: Dict[str, VehicleState] = {}

        for vid in active_ids:
            if vid in self.vehicle_mapping:
                pos = traci.vehicle.getPosition(vid)
                speed = traci.vehicle.getSpeed(vid)
                lane = traci.vehicle.getLaneID(vid)
                edge = traci.vehicle.getRoadID(vid)

                vs = VehicleState(
                    vehicle_id=vid,
                    pos_x=pos[0],
                    pos_y=pos[1],
                    speed_mps=speed,
                    lane_id=lane,
                    edge_id=edge,
                    timestamp=self.current_time
                )
                vehicle_states[vid] = vs

        self.log_vehicle_states(vehicle_states)

        # 2. Run Collision Detector across active vehicle states
        events = self.detector.detect(vehicle_states, self.current_time)
        events_by_pair = {(ev.follower_id, ev.leader_id): ev for ev in events}

        for ev in events:
            self.collision_detection_log.append({
                "timestamp": round(ev.timestamp, 2),
                "follower_id": ev.follower_id,
                "leader_id": ev.leader_id,
                "lane_id": ev.lane_id,
                "gap_m": ev.gap_m,
                "closing_speed_mps": ev.closing_speed_mps,
                "ttc_s": ev.ttc_s if ev.ttc_s != SAFE_TTC else -1.0,
                "risk_level": ev.risk_level,
                "warning_generated": int(ev.warning_generated),
                "event_id": ev.event_id
            })

            # 3. Generate Structured Warning Message & Inject into AODV Network
            if ev.warning_generated:
                msg = self.warning_gen.generate(ev)
                if msg:
                    # In rear-end hazard, sender is Leader or sensing vehicle
                    # Route to Follower via AODV
                    self.send_warning_packet(
                        src_ip=msg.sender_ip,
                        dest_ip=msg.target_ip,
                        warning_type=msg.warning_type,
                        msg_id=msg.event_id
                    )

        # 4. Run Ground Truth evaluation for all same-lane pairs
        lane_groups: Dict[str, List[VehicleState]] = {}
        for vs in vehicle_states.values():
            lane_groups.setdefault(vs.lane_id, []).append(vs)

        for lane_id, v_list in lane_groups.items():
            if len(v_list) >= 2:
                v_sorted = sorted(v_list, key=lambda v: v.pos_x, reverse=True)
                for i in range(len(v_sorted) - 1):
                    leader = v_sorted[i]
                    follower = v_sorted[i + 1]

                    raw_sep = leader.pos_x - follower.pos_x
                    gap_m = max(0.0, raw_sep - VEHICLE_LENGTH_M)
                    closing_speed = follower.speed_mps - leader.speed_mps
                    ttc_s = (gap_m / closing_speed) if (closing_speed > 0.01 and gap_m > 0) else SAFE_TTC

                    pair_key = (follower.vehicle_id, leader.vehicle_id)
                    ev = events_by_pair.get(pair_key)
                    detected_risk = ev.risk_level if ev else "SAFE"
                    warn_gen = ev.warning_generated if ev else False

                    self.evaluator.evaluate_step(
                        current_time=self.current_time,
                        pair=pair_key,
                        lane_id=lane_id,
                        gap_m=gap_m,
                        closing_speed_mps=closing_speed,
                        ttc_s=ttc_s,
                        detected_risk=detected_risk,
                        warning_generated=warn_gen
                    )

    def handle_data_packet(self, receiver: MobileNodeState, pkt: UdpWarningPacket):
        # Override to record warning delivery telemetry
        super().handle_data_packet(receiver, pkt)

        if receiver.ip == pkt.dest_ip:
            e2e_delay_ms = (self.current_time - pkt.creation_time) * 1000.0
            self.warning_transmission_log.append({
                "event_id": pkt.msg_id,
                "sender_ip": pkt.source_ip,
                "receiver_ip": pkt.dest_ip,
                "sender_id": [k for k, v in self.vehicle_ip_map.items() if v == pkt.source_ip][0],
                "receiver_id": [k for k, v in self.vehicle_ip_map.items() if v == pkt.dest_ip][0],
                "warning_type": pkt.warning_type,
                "tx_timestamp": round(pkt.creation_time, 4),
                "rx_timestamp": round(self.current_time, 4),
                "delivery_status": "DELIVERED",
                "e2e_delay_ms": round(e2e_delay_ms, 3),
                "hop_count": pkt.hop_count,
                "routing_path": "->".join(pkt.traversed_path)
            })


def run_scenario(scenario_name: str, gui: bool = False) -> Tuple[Phase5CoSimulator, Dict]:
    sumo_binary = sumolib.checkBinary("sumo-gui" if gui else "sumo")
    sumo_cfg = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "sumo", "highway_simulation", "highway.sumocfg"))

    sumo_cmd = [sumo_binary, "-c", sumo_cfg, "--start", "--quit-on-end", "true", "--no-step-log", "true"]
    traci.start(sumo_cmd)

    cosim = Phase5CoSimulator(
        sumo_cfg=sumo_cfg,
        radio_range=100.0,
        cooldown_s=4.0,
        scenario_name=scenario_name,
        gui=gui
    )

    sim_end_time = 30.0
    try:
        current_sim_time = 0.0
        while current_sim_time < sim_end_time:
            traci.simulationStep()
            current_sim_time = round(traci.simulation.getTime(), 2)

            active_ids = traci.vehicle.getIDList()

            # Ensure lane stability for deterministic test scenarios
            for vid in ("V1", "V2", "V3", "V4", "V5"):
                if vid in active_ids:
                    traci.vehicle.setLaneChangeMode(vid, 512)

            # Scenario-specific dynamics injection:
            if scenario_name == "sudden_braking":
                # Standard benchmark: V1 brakes abruptly at t=15s
                if 15.0 <= current_sim_time < 15.1 and "V1" in active_ids:
                    traci.vehicle.slowDown("V1", 0.5, 4.0)
                if 25.0 <= current_sim_time < 25.1 and "V1" in active_ids:
                    traci.vehicle.setSpeed("V1", -1)

            elif scenario_name == "normal_traffic":
                # Free-flow cruise: no sudden braking
                pass

            elif scenario_name == "safe_following":
                # Maintain safe speed, V1 cruises gently
                if 10.0 <= current_sim_time < 10.1 and "V1" in active_ids:
                    traci.vehicle.slowDown("V1", 18.0, 5.0)

            elif scenario_name == "high_speed_approach":
                # V2 accelerated, V1 slows down slightly
                if 12.0 <= current_sim_time < 12.1 and "V2" in active_ids:
                    traci.vehicle.setSpeed("V2", 28.0)
                if 14.0 <= current_sim_time < 14.1 and "V1" in active_ids:
                    traci.vehicle.slowDown("V1", 5.0, 3.0)

            elif scenario_name == "different_lane":
                # Vehicles in different lanes:
                # V4 in Lane 1 catches up and overtakes V1 in Lane 0.
                # Even when they are laterally adjacent (pos_x separation < 5m),
                # no rear-end collision hazard should be detected because lanes differ!
                # V2 and V3 are removed or held at large distance so no Lane 0 hazard occurs.
                if 5.0 <= current_sim_time < 5.1 and "V2" in active_ids:
                    traci.vehicle.setSpeed("V2", 10.0)
                if 5.0 <= current_sim_time < 5.1 and "V3" in active_ids:
                    traci.vehicle.setSpeed("V3", 10.0)
                if 12.0 <= current_sim_time < 12.1 and "V4" in active_ids:
                    traci.vehicle.setSpeed("V4", 26.0)  # V4 flies past V1 in Lane 1

            elif scenario_name == "risk_recovery":
                # V1 slows down sharply at t=12s, triggering collision warning to V2.
                # At t=18s, V1 accelerates aggressively back to 25 m/s, rapidly widening gap
                # and restoring safe following conditions (risk level returns to SAFE).
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

    finally:
        try:
            traci.close()
        except Exception:
            pass

    metrics = cosim.evaluator.compute_metrics()
    return cosim, metrics


def generate_scenario_plots(cosim: Phase5CoSimulator, output_dir: str):
    os.makedirs(output_dir, exist_ok=True)
    df_det = pd.DataFrame(cosim.collision_detection_log)
    df_tx = pd.DataFrame(cosim.warning_transmission_log)

    if not df_det.empty:
        # Plot 1: TTC & Gap Dynamics over time
        plt.figure(figsize=(9, 4.5), dpi=300)
        v2_v1 = df_det[(df_det["follower_id"] == "V2") & (df_det["leader_id"] == "V1")]
        if not v2_v1.empty:
            ax1 = plt.gca()
            ax2 = ax1.twinx()

            ax1.plot(v2_v1["timestamp"], v2_v1["gap_m"], 'b-', label="Headway Gap (m)", linewidth=2)
            ttc_vals = [t if (t > 0 and t < 20) else None for t in v2_v1["ttc_s"]]
            ax2.plot(v2_v1["timestamp"], ttc_vals, 'r--', label="TTC (s)", linewidth=2)
            ax2.axhline(1.5, color='darkred', linestyle=':', label="Critical Threshold (1.5s)")
            ax2.axhline(3.0, color='orange', linestyle=':', label="High Threshold (3.0s)")

            ax1.set_xlabel("Simulation Time (s)", fontweight="bold")
            ax1.set_ylabel("Bumper-to-Bumper Gap (m)", color='b', fontweight="bold")
            ax2.set_ylabel("Time-to-Collision (s)", color='r', fontweight="bold")
            plt.title(f"Phase 5 Collision Detection Dynamics ({cosim.scenario_name})", fontsize=12, fontweight="bold")
            plt.grid(True, linestyle="--", alpha=0.5)

            plot_path = os.path.join(output_dir, f"phase5_{cosim.scenario_name}_ttc_dynamics.png")
            plt.savefig(plot_path, bbox_inches="tight")
            plt.close()


def run_all_phase5_experiments():
    results_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "results"))
    os.makedirs(results_dir, exist_ok=True)

    scenarios = [
        "sudden_braking",
        "normal_traffic",
        "safe_following",
        "high_speed_approach",
        "different_lane",
        "risk_recovery"
    ]

    all_metrics = []

    print("=" * 80)
    print(" EXECUTING PHASE 5: COLLISION DETECTION & AODV WARNING DISSEMINATION")
    print("=" * 80)

    primary_cosim = None

    for sc in scenarios:
        print(f"\n[SCENARIO EXECUTION] Running: {sc} ...")
        cosim, metrics = run_scenario(sc, gui=False)
        metrics["Scenario"] = sc
        metrics["WarningsGenerated"] = len(cosim.warning_gen.generated_warnings_log)
        metrics["WarningsDelivered"] = len(cosim.warning_transmission_log)
        all_metrics.append(metrics)

        print(
            f"  -> Results: TP={metrics['TP']}, FP={metrics['FP']}, TN={metrics['TN']}, FN={metrics['FN']} | "
            f"Precision={metrics['Precision']*100:.1f}%, Recall={metrics['Recall']*100:.1f}%, F1={metrics['F1']*100:.1f}% | "
            f"Warnings: {metrics['WarningsGenerated']} Gen / {metrics['WarningsDelivered']} Delivered"
        )

        generate_scenario_plots(cosim, results_dir)

        if sc == "sudden_braking":
            primary_cosim = cosim
            # Export Detailed CSVs for primary benchmark scenario
            pd.DataFrame(cosim.vehicle_state_log).to_csv(
                os.path.join(results_dir, "vehicle_state_log.csv"), index=False
            )
            pd.DataFrame(cosim.collision_detection_log).to_csv(
                os.path.join(results_dir, "collision_detection_log.csv"), index=False
            )
            pd.DataFrame(cosim.warning_transmission_log).to_csv(
                os.path.join(results_dir, "warning_transmission_log.csv"), index=False
            )
            cosim.evaluator.export_csv(
                os.path.join(results_dir, "ground_truth_evaluations.csv")
            )

    # Save summary metrics table
    df_summary = pd.DataFrame(all_metrics)
    summary_path = os.path.join(results_dir, "phase5_scenario_evaluation_metrics.csv")
    df_summary.to_csv(summary_path, index=False)

    print("\n" + "=" * 80)
    print(" PHASE 5 ALL 6 SCENARIOS COMPLETE - SUMMARY MATRIX")
    print("=" * 80)
    print(df_summary[["Scenario", "Precision", "Recall", "F1", "WarningsGenerated", "WarningsDelivered", "MeanLatency_s"]].to_string(index=False))
    print("=" * 80)
    print(f"Results and CSV telemetry saved to: {results_dir}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Phase 5 Collision Detection & Warning Verification")
    parser.add_argument("--scenario", type=str, default="all", help="Scenario name or 'all'")
    parser.add_argument("--gui", action="store_true", help="Launch with SUMO-GUI")
    args = parser.parse_args()

    if args.scenario == "all":
        run_all_phase5_experiments()
    else:
        cosim, metrics = run_scenario(args.scenario, gui=args.gui)
        results_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "results"))
        generate_scenario_plots(cosim, results_dir)
        print("\nScenario Results:", metrics)
