"""
VANET Collision Warning & Traffic Management - Phase 6
Integrated Closed-Loop Co-Simulation Runner

Couples:
1. SUMO TraCI Microscopic Traffic Simulation (0.1s step)
2. Real-time Vehicle State Extraction
3. Rear-End Collision Detection (TTC, Gap, Closing Speed)
4. Structured Warning Message Generation (ETSI DENM format)
5. RFC 3561 AODV Multi-Hop Wireless Network Protocol Engine
6. Recipient On-Board Unit Warning Validation & Rejection Protection
7. TraCI Autonomous Braking Actuation on Recipient Vehicle
8. Continuous Speed Monitoring & Safe Recovery Control
9. End-to-End 8-Step Causal Sequence Logging & Telemetry
"""

import os
import sys
import math
import heapq
import argparse
from typing import Dict, List, Tuple, Optional, Any
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

# Network Simulation Core
from omnetpp.veins_integration.run_veins_cosimulation import (
    MobileNodeState, RoutingTableEntry, AodvRreq, AodvRrep, AodvRerr,
    UdpWarningPacket, SimEvent, VeinsCoSimulator
)

# Phase 6 Autonomous Controller
from phase6_autonomous_response.autonomous_controller import (
    AutonomousVehicleController, WarningReceiptValidator
)


class Phase6CoSimulator(VeinsCoSimulator):
    """
    Closed-Loop Co-Simulator coupling SUMO vehicle mobility,
    AODV wireless networking, collision detection, and autonomous actuation.
    """

    def __init__(
        self,
        sumo_cfg: str,
        radio_range: float = 100.0,
        cooldown_s: float = 4.0,
        scenario_name: str = "sudden_braking",
        autonomous_response_enabled: bool = True,
        gui: bool = False,
        verbose: bool = True
    ):
        super().__init__(sumo_cfg=sumo_cfg, radio_range=radio_range, gui=gui)
        self.scenario_name = scenario_name
        self.autonomous_response_enabled = autonomous_response_enabled
        self.verbose = verbose

        # Mapping for IP lookup
        self.vehicle_ip_map = {vid: ip for vid, (omnet_id, ip) in self.vehicle_mapping.items()}
        self.ip_to_vehicle = {ip: vid for vid, ip in self.vehicle_ip_map.items()}

        # Phase 5 Modules
        self.detector = CollisionDetector(
            vehicle_ip_map=self.vehicle_ip_map,
            cooldown_s=cooldown_s,
            verbose=False
        )
        self.warning_gen = WarningGenerator(verbose=False)
        self.evaluator = GroundTruthEvaluator()

        # Phase 6 Autonomous Controller
        self.controller = AutonomousVehicleController(verbose=verbose)

        # Message storage for in-flight packets
        self.active_warning_messages: Dict[str, StructuredWarningMessage] = {}

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
        # 1. Snapshot active vehicle states from SUMO TraCI
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

        # 2. Run Collision Detector
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

            # Step 1: Hazard Detected
            if ev.warning_generated:
                self.controller.log_causal_step(
                    current_time=self.current_time,
                    step_number=1,
                    step_name="HAZARD_DETECTED",
                    event_id=ev.event_id,
                    vehicle_id=ev.follower_id,
                    details=f"Collision hazard detected: leader={ev.leader_id}, gap={ev.gap_m:.2f}m, closing={ev.closing_speed_mps:.2f}m/s, TTC={ev.ttc_s:.2f}s, risk={ev.risk_level}",
                    speed_mps=vehicle_states[ev.follower_id].speed_mps if ev.follower_id in vehicle_states else 0.0,
                    gap_m=ev.gap_m,
                    ttc_s=ev.ttc_s
                )

                # Step 2: Warning Generated
                msg = self.warning_gen.generate(ev)
                if msg:
                    msg.delivery_status = "TRANSMITTED"
                    next_pkt_id = f"{msg.event_id}_{self.packets_generated + 1}"
                    self.active_warning_messages[msg.event_id] = msg
                    self.active_warning_messages[next_pkt_id] = msg

                    self.controller.log_causal_step(
                        current_time=self.current_time,
                        step_number=2,
                        step_name="WARNING_GENERATED",
                        event_id=msg.event_id,
                        vehicle_id=msg.sender_id,
                        details=f"Generated ETSI DENM message from {msg.sender_id} for target {msg.target_id}. Type={msg.warning_type}, Risk={msg.risk_level}",
                        speed_mps=vehicle_states[ev.follower_id].speed_mps if ev.follower_id in vehicle_states else 0.0,
                        gap_m=msg.gap_m,
                        ttc_s=msg.ttc_s
                    )

                    # Step 3: Warning Transmitted into AODV Network
                    self.controller.log_causal_step(
                        current_time=self.current_time,
                        step_number=3,
                        step_name="WARNING_TRANSMITTED",
                        event_id=msg.event_id,
                        vehicle_id=msg.sender_id,
                        details=f"Packet injected into AODV layer ({msg.sender_ip} -> {msg.target_ip})"
                    )

                    self.send_warning_packet(
                        src_ip=msg.sender_ip,
                        dest_ip=msg.target_ip,
                        warning_type=msg.warning_type,
                        msg_id=msg.event_id
                    )

        # 3. Ground Truth Evaluation
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
        # Let base AODV layer process packet delivery
        super().handle_data_packet(receiver, pkt)

        # On delivery at intended destination
        if receiver.ip == pkt.dest_ip:
            e2e_delay_ms = (self.current_time - pkt.creation_time) * 1000.0
            sender_id = self.ip_to_vehicle.get(pkt.source_ip, "UNKNOWN")
            receiver_id = self.ip_to_vehicle.get(pkt.dest_ip, "UNKNOWN")

            self.warning_transmission_log.append({
                "event_id": pkt.msg_id,
                "sender_ip": pkt.source_ip,
                "receiver_ip": pkt.dest_ip,
                "sender_id": sender_id,
                "receiver_id": receiver_id,
                "warning_type": pkt.warning_type,
                "tx_timestamp": round(pkt.creation_time, 4),
                "rx_timestamp": round(self.current_time, 4),
                "delivery_status": "DELIVERED",
                "e2e_delay_ms": round(e2e_delay_ms, 3),
                "hop_count": pkt.hop_count,
                "routing_path": "->".join(pkt.traversed_path)
            })

            # Retrieve full structured message
            msg = self.active_warning_messages.get(pkt.msg_id)
            if not msg and "_" in pkt.msg_id:
                base_id = pkt.msg_id.rsplit("_", 1)[0]
                msg = self.active_warning_messages.get(base_id)
            if msg:
                msg.delivery_status = "DELIVERED"
                if self.autonomous_response_enabled:
                    # CLOSED LOOP: Trigger validated autonomous braking
                    self.controller.handle_delivered_warning(
                        traci_module=traci,
                        warning=msg,
                        current_time=self.current_time
                    )
                else:
                    if self.verbose:
                        print(f"[{self.current_time:6.2f}s] [OPEN-LOOP BASELINE] Warning delivered to {receiver_id}, but autonomous response is disabled.")


def run_phase6_simulation(
    scenario_name: str = "sudden_braking",
    autonomous_response_enabled: bool = True,
    gui: bool = False,
    verbose: bool = True
) -> Tuple[Phase6CoSimulator, Dict[str, Any]]:
    """
    Executes a complete closed-loop Phase 6 simulation with SUMO TraCI.
    """
    sumo_binary = sumolib.checkBinary("sumo-gui" if gui else "sumo")
    sumo_cfg = os.path.abspath(os.path.join(PROJECT_ROOT, "sumo", "highway_simulation", "highway.sumocfg"))

    sumo_cmd = [
        sumo_binary,
        "-c", sumo_cfg,
        "--start",
        "--quit-on-end", "true",
        "--no-step-log", "true",
        "--collision.action", "warn"
    ]
    if gui:
        sumo_cmd.extend(["--delay", "100"])

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

    sim_end_time = 35.0
    try:
        current_sim_time = 0.0
        while current_sim_time < sim_end_time:
            traci.simulationStep()
            current_sim_time = round(traci.simulation.getTime(), 2)

            active_ids = traci.vehicle.getIDList()

            # Ensure lane stability
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
                pass

            elif scenario_name == "safe_following":
                if 10.0 <= current_sim_time < 10.1 and "V1" in active_ids:
                    traci.vehicle.slowDown("V1", 18.0, 5.0)

            elif scenario_name == "high_speed_approach":
                if 12.0 <= current_sim_time < 12.1 and "V2" in active_ids:
                    traci.vehicle.setSpeed("V2", 28.0)
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

    metrics = cosim.evaluator.compute_metrics()
    metrics["Scenario"] = scenario_name
    metrics["ResponseEnabled"] = autonomous_response_enabled
    metrics["WarningsGenerated"] = len(cosim.warning_gen.generated_warnings_log)
    metrics["WarningsDelivered"] = len(cosim.warning_transmission_log)
    metrics["BrakingActions"] = len(cosim.controller.braking_actuation_log)
    metrics["CausalStepsRecorded"] = len(cosim.controller.causal_event_log)

    return cosim, metrics


def export_phase6_results(cosim: Phase6CoSimulator, output_dir: str):
    os.makedirs(output_dir, exist_ok=True)

    # 1. Causal Sequence Log
    pd.DataFrame(cosim.controller.causal_event_log).to_csv(
        os.path.join(output_dir, "causal_event_sequence.csv"), index=False
    )
    # 2. Braking Actuations
    pd.DataFrame(cosim.controller.braking_actuation_log).to_csv(
        os.path.join(output_dir, "autonomous_braking_actuations.csv"), index=False
    )
    # 3. Response Telemetry
    pd.DataFrame(cosim.controller.response_telemetry_log).to_csv(
        os.path.join(output_dir, "response_telemetry_log.csv"), index=False
    )
    # 4. Validation Log
    pd.DataFrame(cosim.controller.validation_log).to_csv(
        os.path.join(output_dir, "warning_validation_log.csv"), index=False
    )
    # 5. Vehicle State Log
    pd.DataFrame(cosim.vehicle_state_log).to_csv(
        os.path.join(output_dir, "vehicle_state_log.csv"), index=False
    )
    # 6. Warning Transmission Log
    pd.DataFrame(cosim.warning_transmission_log).to_csv(
        os.path.join(output_dir, "warning_transmission_log.csv"), index=False
    )
    # 7. Collision Detection Log
    pd.DataFrame(cosim.collision_detection_log).to_csv(
        os.path.join(output_dir, "collision_detection_log.csv"), index=False
    )
    # 8. Ground truth
    cosim.evaluator.export_csv(
        os.path.join(output_dir, "ground_truth_evaluations.csv")
    )


def generate_phase6_comparison_plots(
    cosim_enabled: Phase6CoSimulator,
    cosim_disabled: Phase6CoSimulator,
    output_dir: str
):
    os.makedirs(output_dir, exist_ok=True)

    df_en_veh = pd.DataFrame(cosim_enabled.vehicle_state_log)
    df_dis_veh = pd.DataFrame(cosim_disabled.vehicle_state_log)

    df_en_det = pd.DataFrame(cosim_enabled.collision_detection_log)
    df_dis_det = pd.DataFrame(cosim_disabled.collision_detection_log)

    # Plot 1: Follower Speed Comparison (Response ON vs OFF)
    plt.figure(figsize=(10, 5), dpi=300)
    en_v2 = df_en_veh[df_en_veh["vehicle_id"] == "V2"]
    dis_v2 = df_dis_veh[df_dis_veh["vehicle_id"] == "V2"]
    en_v1 = df_en_veh[df_en_veh["vehicle_id"] == "V1"]

    if not en_v1.empty:
        plt.plot(en_v1["timestamp"], en_v1["speed_kmh"], 'r--', label="V1 (Leading Vehicle - Sudden Braking)", linewidth=2.0)
    if not dis_v2.empty:
        plt.plot(dis_v2["timestamp"], dis_v2["speed_kmh"], 'k:', label="V2 Speed (Response Disabled / Open-Loop)", linewidth=2.2)
    if not en_v2.empty:
        plt.plot(en_v2["timestamp"], en_v2["speed_kmh"], 'g-', label="V2 Speed (Autonomous Response Enabled)", linewidth=2.5)

    # Add braking trigger annotation
    actuations = cosim_enabled.controller.braking_actuation_log
    for act in actuations:
        if act["vehicle_id"] == "V2":
            plt.axvline(act["timestamp"], color='orange', linestyle='--', alpha=0.8)
            plt.annotate(
                f"Warning Received\nBraking Activated\n(t={act['timestamp']}s)",
                xy=(act["timestamp"], act["initial_speed_kmh"]),
                xytext=(act["timestamp"] + 1.2, act["initial_speed_kmh"] + 8),
                arrowprops=dict(facecolor='black', shrink=0.05, width=1.5, headwidth=6),
                fontsize=9, fontweight="bold",
                bbox=dict(boxstyle="round,pad=0.3", fc="yellow", alpha=0.5)
            )

    plt.xlabel("Simulation Time (s)", fontweight="bold", fontsize=11)
    plt.ylabel("Vehicle Speed (km/h)", fontweight="bold", fontsize=11)
    plt.title("Phase 6: Autonomous Vehicle Response - Follower Speed Dynamics", fontweight="bold", fontsize=12)
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend(loc="upper right", framealpha=0.9)
    plt.savefig(os.path.join(output_dir, "phase6_speed_comparison_response_on_vs_off.png"), bbox_inches="tight")
    plt.close()

    # Plot 2: Headway Gap Preservation (Response ON vs OFF)
    plt.figure(figsize=(10, 5), dpi=300)
    en_gap = df_en_det[(df_en_det["follower_id"] == "V2") & (df_en_det["leader_id"] == "V1")]
    dis_gap = df_dis_det[(df_dis_det["follower_id"] == "V2") & (df_dis_det["leader_id"] == "V1")]

    if not dis_gap.empty:
        plt.plot(dis_gap["timestamp"], dis_gap["gap_m"], 'r--', label="Bumper-to-Bumper Gap (Response Disabled)", linewidth=2.2)
    if not en_gap.empty:
        plt.plot(en_gap["timestamp"], en_gap["gap_m"], 'b-', label="Bumper-to-Bumper Gap (Autonomous Response Enabled)", linewidth=2.5)

    plt.axhline(2.5, color='darkred', linestyle=':', label="Physical Collision Boundary (2.5m)", linewidth=1.5)
    plt.xlabel("Simulation Time (s)", fontweight="bold", fontsize=11)
    plt.ylabel("Bumper-to-Bumper Gap (m)", fontweight="bold", fontsize=11)
    plt.title("Phase 6: Collision Margin Preservation Through Autonomous Braking", fontweight="bold", fontsize=12)
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend(loc="upper right", framealpha=0.9)
    plt.savefig(os.path.join(output_dir, "phase6_headway_gap_preservation.png"), bbox_inches="tight")
    plt.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Phase 6 Closed-Loop Autonomous Response Simulation")
    parser.add_argument("--scenario", type=str, default="sudden_braking")
    parser.add_argument("--disable-response", action="store_true", help="Run in open-loop mode (response disabled)")
    parser.add_argument("--gui", action="store_true", help="Launch SUMO with GUI")
    args = parser.parse_args()

    results_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "results"))
    cosim, metrics = run_phase6_simulation(
        scenario_name=args.scenario,
        autonomous_response_enabled=not args.disable_response,
        gui=args.gui
    )
    export_phase6_results(cosim, results_dir)
    print("\nSimulation Metrics:", metrics)
