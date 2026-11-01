"""
VANET Collision Warning & Traffic Management Simulation
Phase 1 - Vehicle Mobility and Sudden Braking Scenario with TraCI

This script manages vehicle mobility in SUMO via the TraCI API.
Scenario:
- Two vehicles (V1 leading, V2 following) travel along a 3 km highway in Lane 0.
- At simulation time t = 15.0 s, leading vehicle V1 performs sudden braking,
  decelerating from cruise speed (~22 m/s) to 0.5 m/s over 4.0 seconds.
- Following vehicle V2 closes in, causing a sudden drop in headway distance
  and Time-to-Collision (TTC), triggering a Forward Collision Warning (FCW).
- At t = 25.0 s, normal speed control is restored to V1.
- Detailed vehicle telemetry (positions, speeds, accelerations, gaps, TTC)
  is logged to console and written to CSV in the experiment_logs directory.
"""

import os
import sys
import time
import argparse
from datetime import datetime

# ---------------------------------------------------------------------------
# 1. Environment & SUMO Tools Configuration
# ---------------------------------------------------------------------------

if "SUMO_HOME" not in os.environ:
    default_sumo = r"C:\Program Files (x86)\SUMO"
    if os.path.exists(default_sumo):
        os.environ["SUMO_HOME"] = default_sumo
    else:
        raise EnvironmentError(
            "SUMO_HOME is not set in the environment and default directory "
            f"({default_sumo}) was not found. Please set SUMO_HOME."
        )

tools_dir = os.path.join(os.environ["SUMO_HOME"], "tools")
if tools_dir not in sys.path:
    sys.path.insert(0, tools_dir)

try:
    import sumolib
    import traci
except ImportError as e:
    raise ImportError(
        f"Failed to import SUMO libraries (traci/sumolib) from {tools_dir}: {e}"
    )


# ---------------------------------------------------------------------------
# 2. Argument Parsing & Experiment Parameters
# ---------------------------------------------------------------------------

def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Run SUMO Sudden-Braking Collision Warning Scenario"
    )
    parser.add_argument(
        "--gui",
        action="store_true",
        default=False,
        help="Launch SUMO with GUI (default: False for automated runs, True if specified)"
    )
    parser.add_argument(
        "--end-time",
        type=float,
        default=40.0,
        help="Simulation end time in seconds (default: 40.0)"
    )
    parser.add_argument(
        "--brake-time",
        type=float,
        default=15.0,
        help="Timestamp when V1 sudden braking starts (default: 15.0)"
    )
    parser.add_argument(
        "--restore-time",
        type=float,
        default=25.0,
        help="Timestamp when V1 normal speed control resumes (default: 25.0)"
    )
    parser.add_argument(
        "--target-brake-speed",
        type=float,
        default=0.5,
        help="Target speed for V1 during braking in m/s (default: 0.5)"
    )
    parser.add_argument(
        "--brake-duration",
        type=float,
        default=4.0,
        help="Duration of V1 braking maneuver in seconds (default: 4.0)"
    )
    parser.add_argument(
        "--ttc-threshold",
        type=float,
        default=4.0,
        help="TTC warning threshold in seconds (default: 4.0)"
    )
    return parser.parse_args()


# ---------------------------------------------------------------------------
# 3. Main Simulation Execution
# ---------------------------------------------------------------------------

def run_simulation():
    args = parse_arguments()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_file = os.path.join(script_dir, "highway.sumocfg")
    logs_dir = os.path.join(script_dir, "experiment_logs")
    os.makedirs(logs_dir, exist_ok=True)

    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_filename = os.path.join(logs_dir, f"braking_experiment_{timestamp_str}.csv")
    latest_log_filename = os.path.join(logs_dir, "braking_experiment_latest.csv")

    binary_name = "sumo-gui" if args.gui else "sumo"
    try:
        sumo_binary = sumolib.checkBinary(binary_name)
    except Exception:
        print(f"Warning: '{binary_name}' binary check failed, falling back to 'sumo'")
        sumo_binary = sumolib.checkBinary("sumo")

    sumo_cmd = [
        sumo_binary,
        "-c", config_file,
        "--start",
        "--quit-on-end", "true",
        "--no-step-log", "true",
        "--collision.action", "warn"
    ]
    if args.gui:
        sumo_cmd.extend(["--delay", "100"])

    print("=" * 70)
    print(" VANET COLLISION WARNING - SUDDEN BRAKING EXPERIMENT")
    print("=" * 70)
    print(f"SUMO Binary       : {sumo_binary}")
    print(f"Configuration     : {config_file}")
    print(f"Experiment End    : {args.end_time} s")
    print(f"Braking Event     : t = {args.brake_time} s (target {args.target_brake_speed} m/s over {args.brake_duration} s)")
    print(f"Speed Restored    : t = {args.restore_time} s")
    print(f"TTC FCW Threshold : {args.ttc_threshold} s")
    print(f"Log File          : {log_filename}")
    print("=" * 70)

    traci.start(sumo_cmd)

    # State variables
    braking_initiated = False
    speed_restored = False
    event_status = "NORMAL"

    csv_records = []
    csv_header = [
        "timestamp_sec",
        "vehicle_id",
        "pos_x_m",
        "pos_y_m",
        "edge_id",
        "lane_id",
        "lane_index",
        "speed_mps",
        "speed_kmh",
        "acceleration_mps2",
        "event_status",
        "lead_vehicle_id",
        "gap_to_lead_m",
        "closing_speed_mps",
        "ttc_sec",
        "collision_warning"
    ]

    previous_speeds = {}
    last_console_print = -1.0

    try:
        while True:
            traci.simulationStep()
            sim_time = round(traci.simulation.getTime(), 2)

            if sim_time > args.end_time:
                break

            active_vehicles = traci.vehicle.getIDList()

            # Prevent lane-changing for V1 and V2 to ensure controlled in-lane car-following
            for vid in ("V1", "V2"):
                if vid in active_vehicles:
                    # 512 disables strategic/speed-gain lane changes while keeping safety
                    traci.vehicle.setLaneChangeMode(vid, 512)

            # ---------------------------------------------------------------
            # Handle Braking Event Triggering
            # ---------------------------------------------------------------
            if sim_time >= args.brake_time and not braking_initiated:
                if "V1" in active_vehicles:
                    v1_speed_before = traci.vehicle.getSpeed("V1")
                    traci.vehicle.slowDown("V1", args.target_brake_speed, args.brake_duration)
                    braking_initiated = True
                    event_status = "BRAKING_TRIGGERED"
                    print(f"\n[t={sim_time:.1f}s] >>> V1 SUDDEN BRAKING TRIGGERED! <<<")
                    print(f"          Initial Speed: {v1_speed_before * 3.6:.2f} km/h -> Target: {args.target_brake_speed * 3.6:.2f} km/h in {args.brake_duration}s\n")
                else:
                    print(f"WARNING: V1 was not active at scheduled braking time t={sim_time:.1f}s!")
                    braking_initiated = True

            elif braking_initiated and not speed_restored:
                if sim_time < args.brake_time + args.brake_duration:
                    event_status = "DECELERATING"
                elif sim_time < args.restore_time:
                    event_status = "BRAKING_HOLD"
                else:
                    if "V1" in active_vehicles:
                        traci.vehicle.setSpeed("V1", -1)
                        print(f"\n[t={sim_time:.1f}s] >>> V1 NORMAL SPEED CONTROL RESTORED <<<\n")
                    speed_restored = True
                    event_status = "SPEED_RESTORED"

            # ---------------------------------------------------------------
            # Compute Headway, Closing Speed, and TTC between V1 and V2
            # ---------------------------------------------------------------
            gap_v1_v2 = None
            closing_speed_v1_v2 = None
            ttc_v1_v2 = None
            fcw_warning = "CLEAR"

            if "V1" in active_vehicles and "V2" in active_vehicles:
                p1 = traci.vehicle.getPosition("V1")
                p2 = traci.vehicle.getPosition("V2")
                s1 = traci.vehicle.getSpeed("V1")
                s2 = traci.vehicle.getSpeed("V2")
                l1_len = traci.vehicle.getLength("V1")

                # Longitudinal headway along straight X-axis (bumper-to-bumper)
                # Headway = (X_front_lead - X_front_follow) - length_lead
                raw_gap = (p1[0] - p2[0]) - l1_len
                gap_v1_v2 = max(0.0, round(raw_gap, 2))

                # Closing speed: positive if V2 is faster than V1 (closing in)
                closing_speed = s2 - s1
                closing_speed_v1_v2 = round(closing_speed, 2)

                # Time-to-Collision (TTC)
                if closing_speed > 0.05 and gap_v1_v2 > 0:
                    ttc = gap_v1_v2 / closing_speed
                    ttc_v1_v2 = round(ttc, 2)
                    if ttc <= args.ttc_threshold:
                        fcw_warning = "WARNING"
                else:
                    ttc_v1_v2 = float("inf")

            # ---------------------------------------------------------------
            # Telemetry Collection for Each Active Vehicle
            # ---------------------------------------------------------------
            for vid in sorted(active_vehicles):
                pos = traci.vehicle.getPosition(vid)
                speed_mps = traci.vehicle.getSpeed(vid)
                speed_kmh = speed_mps * 3.6
                edge_id = traci.vehicle.getRoadID(vid)
                lane_id = traci.vehicle.getLaneID(vid)
                lane_idx = traci.vehicle.getLaneIndex(vid)

                # Compute numerical acceleration (dv / dt)
                prev_speed = previous_speeds.get(vid, speed_mps)
                accel = (speed_mps - prev_speed) / 0.1
                previous_speeds[vid] = speed_mps

                if vid == "V2" and gap_v1_v2 is not None:
                    veh_lead = "V1"
                    veh_gap = gap_v1_v2
                    veh_closing = closing_speed_v1_v2
                    veh_ttc = "inf" if ttc_v1_v2 == float("inf") else ttc_v1_v2
                    veh_warning = fcw_warning
                else:
                    veh_lead = "None"
                    veh_gap = ""
                    veh_closing = ""
                    veh_ttc = ""
                    veh_warning = ""

                csv_records.append([
                    f"{sim_time:.2f}",
                    vid,
                    f"{pos[0]:.2f}",
                    f"{pos[1]:.2f}",
                    edge_id,
                    lane_id,
                    lane_idx,
                    f"{speed_mps:.2f}",
                    f"{speed_kmh:.2f}",
                    f"{accel:.2f}",
                    event_status if vid == "V1" else "NORMAL",
                    veh_lead,
                    veh_gap,
                    veh_closing,
                    veh_ttc,
                    veh_warning
                ])

            # ---------------------------------------------------------------
            # Periodic Console Reporting (Every 1.0 second)
            # ---------------------------------------------------------------
            if sim_time - last_console_print >= 0.99:
                last_console_print = sim_time
                status_line = f"t={sim_time:4.1f}s | Active: {len(active_vehicles)} veh | Status: {event_status:<16}"
                print(status_line)

                if "V1" in active_vehicles and "V2" in active_vehicles:
                    v1_kmh = traci.vehicle.getSpeed("V1") * 3.6
                    v2_kmh = traci.vehicle.getSpeed("V2") * 3.6
                    ttc_str = f"{ttc_v1_v2:.2f}s" if (ttc_v1_v2 is not None and ttc_v1_v2 != float("inf")) else "inf"
                    print(
                        f"   V1 Speed: {v1_kmh:5.1f} km/h | V2 Speed: {v2_kmh:5.1f} km/h | "
                        f"Gap: {gap_v1_v2:5.1f} m | Closing: {closing_speed_v1_v2:+5.2f} m/s | "
                        f"TTC: {ttc_str:>6} | Alert: [{fcw_warning}]"
                    )

    finally:
        try:
            traci.close()
        except Exception:
            pass

    # -----------------------------------------------------------------------
    # Write CSV Log Files
    # -----------------------------------------------------------------------
    with open(log_filename, "w", encoding="utf-8") as f:
        f.write(",".join(csv_header) + "\n")
        for rec in csv_records:
            f.write(",".join(str(c) for c in rec) + "\n")

    # Also save as latest
    with open(latest_log_filename, "w", encoding="utf-8") as f:
        f.write(",".join(csv_header) + "\n")
        for rec in csv_records:
            f.write(",".join(str(c) for c in rec) + "\n")

    print("\n" + "=" * 70)
    print(" EXPERIMENT COMPLETE")
    print("=" * 70)
    print(f"Total telemetry data points logged : {len(csv_records)}")
    print(f"Timestamped log file saved to      : {log_filename}")
    print(f"Latest log pointer saved to        : {latest_log_filename}")
    print("=" * 70)


if __name__ == "__main__":
    run_simulation()
