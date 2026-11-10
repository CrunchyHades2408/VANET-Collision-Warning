# Final Results & Evidence Repository

**Project Title:** VANET-Based Collision Warning and Traffic Management Using AODV Routing  
**Deliverable Milestone:** Phase 8 Final System Integration & Academic Submission Deliverables  
**Simulation Toolchain:** Eclipse SUMO 1.28.0 • Python TraCI • RFC 3561 AODV Discrete-Event Wireless Model  

---

## 1. Directory Structure Overview

```
final_results/
├── README.md                      # This comprehensive data dictionary and evidence guide
├── configs/                       # SUMO simulation definitions and network geometry
│   ├── highway.net.xml            # 3.0 km, 2-lane multi-edge highway road network
│   ├── highway.rou.xml            # Microscopic vehicle types and route schedules (V1-V5)
│   └── highway.sumocfg            # Master SUMO configuration file
├── figures/                       # Presentation-ready analytical plots (300 DPI)
│   ├── phase6_speed_comparison_response_on_vs_off.png
│   ├── phase6_headway_gap_preservation.png
│   ├── phase7_scenario_detection_performance.png
│   ├── phase7_minimum_gap_preservation.png
│   ├── phase7_network_pdr_and_latency.png
│   ├── phase7_braking_response_and_speed_reduction.png
│   ├── phase7_threshold_sensitivity_curves.png
│   └── phase5_*_ttc_dynamics.png  # Historical Phase 5 dynamics figures
├── logs/                          # Raw simulation and experimental traces
│   ├── causal_event_sequence.csv  # Chronological 8-step causal trace (Hazard -> Recovery)
│   ├── autonomous_braking_actuations.csv # TraCI slowDown() execution records
│   ├── response_telemetry_log.csv # Continuous deceleration & speed drop telemetry
│   ├── warning_validation_log.csv # Audit of accepted and rejected warnings
│   ├── phase7_all_experiment_runs.csv # All 60 controlled experimental runs
│   ├── phase7_threshold_sensitivity_analysis.csv # Sensitivity data across TTC thresholds
│   ├── vehicle_state_log.csv      # Microscopic coordinates and speeds per vehicle
│   ├── warning_transmission_log.csv # AODV wireless packet delivery records
│   ├── collision_detection_log.csv  # TTC, gap, closing speed, risk tiers
│   └── ground_truth_evaluations.csv # Timestep-by-timestep TP/FP/TN/FN metrics
├── summaries/                     # Aggregated benchmarks and statistical matrices
│   ├── phase7_response_comparison_matrix.csv # Response Enabled vs. Disabled contrast
│   ├── phase7_statistical_summary.csv # Mean, Std, Min, Max, 95% Confidence Intervals
│   └── phase5_scenario_evaluation_metrics.csv # Historical Phase 5 metrics
└── test_reports/                  # Automated verification and unit test records
    └── phase6_unit_test_results.csv # 6/6 Unit & Edge-Case Tests (100% Passed)
```

---

## 2. File & Column Data Dictionary

### 2.1 `causal_event_sequence.csv` (Phase 6 Evidence)
Captures the complete 8-step closed-loop sequence from initial hazard detection to safe recovery:
- `timestamp`: Simulation timestamp (seconds, 0.1s resolution).
- `step_number`: Causal index (1 to 8):
  1. `HAZARD_DETECTED` (Same-lane closing motion & critical TTC).
  2. `WARNING_GENERATED` (ETSI DENM format constructed).
  3. `WARNING_TRANSMITTED` (Injected into AODV network layer).
  4. `WARNING_DELIVERED` (Confirmed arrival at target vehicle IP).
  5. `WARNING_RECEIVED_VALID` (OBU validated freshness, recipient match, deduplication).
  6. `BRAKING_ACTIVATED` (TraCI `slowDown()` command dispatched).
  7. `VEHICLE_SPEED_CHANGED` (Measurable speed drop confirmed).
  8. `RESPONSE_COMPLETED` (Normal car-following restored via `setSpeed(-1)`).
- `step_name`: Formal causal transition state identifier.
- `event_id`: Unique hazard incident key.
- `vehicle_id`: SUMO vehicle identifier (`V1`, `V2`, `V3`).
- `details`: Detailed telemetry string including speeds, deceleration, and reasons.
- `speed_mps`: Instantaneous vehicle speed in metres per second.
- `gap_m`: Instantaneous bumper-to-bumper distance in metres.
- `ttc_s`: Instantaneous Time-to-Collision in seconds.

### 2.2 `autonomous_braking_actuations.csv`
Logs every autonomous deceleration maneuver executed by the `AutonomousVehicleController`:
- `timestamp`: Time of braking activation (s).
- `event_id`: Warning ID that triggered the response.
- `vehicle_id`: Recipient vehicle executing the braking maneuver.
- `sender_id`: Source vehicle initiating the alert.
- `risk_level`: Severity tier (`CRITICAL`, `HIGH`, `MODERATE`).
- `initial_speed_mps` / `initial_speed_kmh`: Speed at brake onset.
- `target_speed_mps` / `target_speed_kmh`: Target speed commanded to TraCI `slowDown()`.
- `duration_s`: Deceleration duration window (s).
- `expected_decel_mps2`: Deceleration rate $\frac{v_{\text{initial}} - v_{\text{target}}}{t_{\text{duration}}}$ ($\text{m/s}^2$).
- `ttc_s`: Triggering Time-to-Collision (s).
- `gap_m`: Triggering bumper-to-bumper gap (m).

### 2.3 `phase7_all_experiment_runs.csv`
Contains the complete matrix of 60 controlled experimental runs across all 6 scenarios:
- `Scenario`: `sudden_braking`, `normal_traffic`, `safe_following`, `high_speed_approach`, `different_lane`, `risk_recovery`.
- `RunID`: Index of run ($1$ to $5$).
- `Seed`: Pseudo-random seed passed to SUMO ($42, 101, 202, 303, 404$).
- `ResponseEnabled`: `True` (Closed-Loop) vs `False` (Open-Loop Baseline).
- `TP`, `FP`, `TN`, `FN`: Ground-truth confusion matrix counts.
- `Precision`, `Recall`, `F1`, `FalseAlarmRate`: Standard performance scores (Precision handles zero denominators as `NaN` / `N/A`).
- `MinTTC_s`: Minimum Time-to-Collision observed during run.
- `MinGap_m`: Closest bumper-to-bumper distance observed (m).
- `BrakingActionsCount`: Number of TraCI `slowDown()` actuations.
- `MaxSpeedReduction_kmh`: Peak speed drop ($\Delta v$) in km/h.
- `BrakingResponseTime_ms`: Latency from warning emission to braking activation.
- `PDR_Percent`: Packet delivery ratio (%).
- `AvgE2EDelay_ms`: Mean end-to-end multi-hop latency (ms).

### 2.4 `phase7_statistical_summary.csv`
Aggregates the 60 runs with Student's t-distribution 95% Confidence Intervals:
- `*_Mean`: Sample average ($\bar{x}$).
- `*_Std`: Sample standard deviation ($s$).
- `*_Min` / `*_Max`: Extremes observed across the 5 independent seeds.
- `*_CI95_Margin`: Margin of error ($2.776 \cdot \frac{s}{\sqrt{5}}$).

---

## 3. Reproduction Instructions

To reproduce all results from scratch in a clean environment:
```powershell
# 1. Execute the master integrated workflow (runs simulation, experiments, and exports deliverables)
python run_full_system.py --runs 5

# 2. Run the automated Phase 6 unit test suite
python phase6_autonomous_response\test_phase6_response.py

# 3. Launch the interactive results dashboard
streamlit run dashboard\app.py
```
