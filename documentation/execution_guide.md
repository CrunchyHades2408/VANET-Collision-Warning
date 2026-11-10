# Simulation Execution & Replication Guide

This guide provides step-by-step instructions to run, replicate, and evaluate every phase of the project from a clean environment.

---

## 1. System Requirements & Setup

### 1.1 Prerequisites
- **Operating System**: Windows 10 / 11 x64
- **Python**: 3.12+ (Installed and on system `PATH`)
- **Eclipse SUMO**: Version 1.28.0 (Installed at `C:\Program Files (x86)\SUMO\`)
- **SUMO_HOME Environment Variable**: Set to `C:\Program Files (x86)\SUMO`

### 1.2 Python Dependencies
Install required packages via PowerShell:
```powershell
pip install traci sumolib pandas matplotlib plotly streamlit numpy Pillow
```

---

## 2. Quickstart: Full System Execution (Recommended)

To run the complete integrated pipeline (SUMO traffic $\to$ collision detection $\to$ AODV warning dissemination $\to$ TraCI autonomous response $\to$ statistical evaluation $\to$ deliverables export):

```powershell
# Open terminal in the project root folder
cd "C:\VANET Collision Warning"

# Execute the integrated 10-step master workflow
python run_full_system.py --runs 5
```

### Expected Output:
- Executes the primary benchmark scenario (`sudden_braking`) under Closed-Loop (Response ON) and Open-Loop Baseline (Response OFF).
- Executes 60 multi-seed experimental runs across all 6 highway scenarios.
- Computes Student's t-distribution 95% Confidence Intervals.
- Generates 300 DPI analytical charts and exports consolidated CSVs to `final_results/`.

---

## 3. Interactive Results Dashboard

To launch the web-based interactive analytics dashboard:
```powershell
streamlit run dashboard/app.py
```
- **URL**: Automatically opens `http://localhost:8501` in your default browser.
- **Features**:
  - Page 1: High-level KPIs, project overview, active dataset registry.
  - Page 2: Microscopic vehicle speed and position trajectories.
  - Page 3: Real-time TTC dynamics and ground-truth metrics (with N/A handling).
  - Page 4: Phase 6 Closed-Loop Autonomous Response timeline & braking telemetry.
  - Page 5: Phase 7 Advanced Experiments, 95% Confidence Intervals, and sensitivity curves.
  - Page 6: Cross-scenario comparative benchmarks.
  - Page 7: AODV network delivery ratio and latency distributions.
  - Page 8: Results gallery and direct one-click CSV dataset downloads.

---

## 4. Phase-by-Phase Execution Instructions

### Phase 1: Microscopic Traffic & Sudden-Braking
```powershell
cd "C:\VANET Collision Warning\sumo\highway_simulation"
python braking_scenario.py --end-time 40.0
```
- Output: `sumo/highway_simulation/experiment_logs/braking_experiment_latest.csv`.

### Phase 2: OMNeT++ 5-Node Discrete-Event Emulation
```powershell
cd "C:\VANET Collision Warning\omnetpp\five_vehicle_network"
python simulate_five_nodes.py
```
- Output: Console trace showing 4-hop forwarding from V1 to V5 with $3.365\text{ ms}$ latency.

### Phase 3: RFC 3561 AODV Ad-Hoc Routing Suite
```powershell
cd "C:\VANET Collision Warning\omnetpp\aodv_simulation"
python run_aodv_experiments.py
```
- Output: 5 analytical figures and `omnetpp/aodv_simulation/results/aodv_experiment_metrics.csv`.

### Phase 4: SUMO–AODV Mobility Co-Simulation
```powershell
cd "C:\VANET Collision Warning\omnetpp\veins_integration"
python run_veins_cosimulation.py
```
- Output: Dynamic topology plots and `omnetpp/veins_integration/results/veins_mobility_sync.csv`.

### Phase 5: Collision Detection Across 6 Scenarios
```powershell
cd "C:\VANET Collision Warning\phase5_collision_detection"
python phase5_cosimulation.py --scenario all
```
- Output: Dynamic TTC plots and `phase5_collision_detection/results/phase5_scenario_evaluation_metrics.csv`.

### Phase 6: Closed-Loop Autonomous Vehicle Response
```powershell
# 1. Run the automated unit test suite (6/6 tests)
python "C:\VANET Collision Warning\phase6_autonomous_response\test_phase6_response.py"

# 2. Run live SUMO closed-loop co-simulation
python "C:\VANET Collision Warning\phase6_autonomous_response\phase6_cosimulation.py" --scenario sudden_braking
```
- Output: `phase6_autonomous_response/results/causal_event_sequence.csv` and `autonomous_braking_actuations.csv`.

### Phase 7: Advanced Multi-Run Experiments & Statistical Analysis
```powershell
python "C:\VANET Collision Warning\phase7_validation\run_all_experiments.py" --runs 5
```
- Output: 60-run matrix, 95% Confidence Interval summary, and 5 presentation figures in `phase7_validation/results/`.
