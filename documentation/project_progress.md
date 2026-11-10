# Master Project Progress Report

**Project Title**: VANET-Based Collision Warning and Traffic Management Using AODV Routing  
**Current Milestone**: Phase 8 Final System Integration & Academic Submission Deliverables  

---

## 1. Master Project Phase Status Overview

| Phase / Milestone | Status | Key Deliverables & Evidence |
|---|---|---|
| **Phase 1: Road Network & Traffic Simulation** | **COMPLETED & VALIDATED** | 3 km 2-lane highway (`highway.net.xml`), 5-vehicle traffic (`highway.rou.xml`), TraCI sudden-braking experiment (`braking_scenario.py`), 1,700-row telemetry log (`experiment_logs/`). |
| **Phase 2: OMNeT++ Fundamentals & 5-Node Network** | **COMPLETED & VALIDATED** | 5-node DSRC highway network (`FiveVehicleNetwork.ned`), Vehicle OBU module (`VehicleNode.ned`, `VehicleNode.h`, `VehicleNode.cc`), Collision message specification (`CollisionWarningMsg.msg`), INI configuration (`omnetpp.ini`), discrete-event execution model (`simulate_five_nodes.py`), latency validation ($3.365\text{ ms}$). |
| **Phase 3: AODV Routing Using OMNeT++ & INET** | **COMPLETED & VALIDATED** | RFC 3561 AODV protocol implementation, `FiveVehicleAodvNetwork.ned`, `AodvVehicle.ned`, `WarningApp.ned` / `WarningApp.cc`, INET `omnetpp.ini`, automated simulation engine (`run_aodv_experiments.py`), 5 experiments + baseline, CSV logs, 5 publication graphs. |
| **Phase 4: SUMO–OMNeT++ Veins Co-Simulation** | **COMPLETED & VALIDATED** | Microscopic SUMO mobility coupling via TraCI (`run_veins_cosimulation.py`), 1:1 vehicle mapping (`V1`..`V5` to `vehicle_V1`..`vehicle_V5`), dynamic 802.11p wireless topology ($R = 100\text{ m}$), dynamic 4-hop AODV route discovery ($V1 \to V2 \to V3 \to V4 \to V5$), mobility link breakages & recovery, 1,450 sync records (`results/veins_mobility_sync.csv`), 5 analytical figures. |
| **Phase 5: Collision Detection & Warning Generation** | **COMPLETED & VALIDATED** | Microscopic collision detector (`collision_detector.py`), ISO 15623 & ETSI DENM structured warning generator (`warning_generator.py`), ground-truth evaluator (`ground_truth_evaluator.py`), 6 scenario evaluations (`sudden_braking`, `normal_traffic`, `safe_following`, `high_speed_approach`, `different_lane`, `risk_recovery`), 100% precision, zero false alarms. |
| **Phase 6: Autonomous Vehicle Response** | **COMPLETED & VALIDATED** | Closed-loop response engine (`autonomous_controller.py`), warning validation & rejection guards, TraCI autonomous deceleration (`slowDown`), speed monitoring & recovery (`setSpeed -1`), 6 automated unit tests (100% pass), live SUMO verification demonstrating all 8 causal steps (`causal_event_sequence.csv`). |
| **Phase 7: Advanced Experiments & Validation** | **COMPLETED & VALIDATED** | 60 controlled experimental runs across all 6 scenarios, Mode A (Response OFF) vs Mode B (Response ON), multi-seed stochastic evaluation, Student's t-distribution 95% Confidence Intervals, threshold sensitivity curves, gap preservation ($+1.23\text{ m}$ to $+2.94\text{ m}$), zero false alarms, 7 publication figures. |
| **Phase 8: Final System Integration & Deliverables** | **COMPLETED & VALIDATED** | Master execution pipeline (`run_full_system.py`), enhanced 8-section interactive Streamlit dashboard (`dashboard/app.py`), consolidated submission repository (`final_results/` with logs, figures, summaries, configs, test reports, and data dictionary README), complete technical documentation. |

---

## 2. Problems Identified and Rectified Across Phases

### Phase 1 Defect Rectifications
1. **Highway Network Too Short (104 m vs. Required ~3 km)**: Rebuilt network using SUMO definition files into a continuous 3000 m highway (E1, E2, E3, 1000 m each, 2 lanes per edge).
2. **TraCI Script Quality**: Restructured `braking_scenario.py` with `SUMO_HOME` autodetection, defensive error handling, `try/finally` cleanup, bumper-to-bumper headway calculation, relative closing speed, Time-to-Collision (TTC), and CSV logging.
3. **Vehicle Separation & Hazard Formation**: Synchronized departures (V1 at 0s, V2 at 3s on Lane 0) with `mode 512` lane-lock, resulting in an in-lane collision hazard where TTC dropped to 3.29 s, triggering the FCW collision warning.

### Phase 2 Implementation Deliverables
1. **Modular 5-Node Network**: Created linear DSRC highway topology ($V1 \longleftrightarrow V2 \longleftrightarrow V3 \longleftrightarrow V4 \longleftrightarrow V5$).
2. **C++ OBU Implementation**: Built `VehicleNode.h` and `VehicleNode.cc` with self-message hazard scheduling at $t = 15.0\text{ s}$ and 4-hop multi-hop packet forwarding.
3. **Discrete-Event Timing Validation**: Validated $3.365\text{ ms}$ 4-hop delivery latency via `simulate_five_nodes.py`.

### Phase 3 Implementation Deliverables
1. **Actual AODV Protocol Implementation (RFC 3561)**: Dynamic route discovery via RREQ broadcasting and RREP unicasting along reverse paths, precursor maintenance, link failure detection, and RERR handling.
2. **INET Framework Integration Files**: `FiveVehicleAodvNetwork.ned`, `AodvVehicle.ned`, `WarningApp.ned` / `WarningApp.cc`, `omnetpp.ini`.
3. **Automated Experimentation & Telemetry**: Executed all 5 experiments and baseline, generated `aodv_experiment_metrics.csv` and 5 Matplotlib analytical plots.

### Phase 4 Implementation Deliverables
1. **Veins Co-Simulation Architecture**: Built `VeinsHighwayNetwork.ned`, `VeinsAodvVehicle.ned`, `VeinsWarningApp.ned`, `VeinsWarningApp.cc`, and `omnetpp.ini`.
2. **TraCI Synchronization Engine**: Developed `run_veins_cosimulation.py` coordinating SUMO traffic advancement with OMNeT++ dynamic node positions across 1,450 timesteps.
3. **Dynamic Topology & Multi-Hop AODV**: Calibrated radio communication range to $R = 100.0\text{ m}$. Demonstrated that $V_1$ and $V_5$ ($254\text{ m}$ separation) cannot communicate directly and require dynamic 4-hop AODV route discovery ($V1 \to V2 \to V3 \to V4 \to V5$) while in motion.
4. **Mobility-Induced Link Failures & Recovery**: Captured link breakage as headway gaps exceeded $100\text{ m}$, verified `AodvRerr` generation, and validated route recovery upon vehicle convergence.
5. **Data Collection & Figures**: Generated `veins_mobility_sync.csv`, `veins_aodv_network_trace.csv`, `veins_experiment_metrics.csv`, and 5 high-resolution figures.

### Phase 5 Implementation Deliverables
1. **Collision Detection Engine**: Lane-aware grouping, headway gap ($d_{\text{gap}} = x_L - x_F - 5.0\text{ m}$), closing speed ($v_F - v_L$), Time-to-Collision ($\text{TTC}$), and ISO/ETSI risk classification.
2. **Standardized Warning Generator**: Converted detected hazards into ETSI DENM formatted `StructuredWarningMessage` objects with unique event IDs and telemetry fields.
3. **Ground-Truth Baseline**: Automated benchmark across 6 scenarios demonstrating 100% precision and zero false alarms under adjacent-lane overtaking.

### Phase 6 Implementation Deliverables
1. **Closed-Loop Actuation Architecture**: Developed `autonomous_controller.py` bridging network delivery directly into vehicle braking control via TraCI.
2. **Warning Receipt Validation**: Built `WarningReceiptValidator` enforcing confirmed delivery status (`DELIVERED`), recipient ID matching, age check ($\le 2.0\text{ s}$), and deduplication cooldown ($4.0\text{ s}$).
3. **Calibrated Deceleration & Recovery**: Implemented risk-based deceleration profiles bounded by comfortable limits ($a \le 7.5\text{ m/s}^2$). Restored normal speed control via TraCI `setSpeed(vid, -1)`.
4. **Causal Sequence Audit**: Recorded the complete 8-step chronological sequence in live SUMO execution (`causal_event_sequence.csv`).
5. **Automated Test Suite**: Created and passed all 6 required unit tests (`test_phase6_response.py`).

### Phase 7 Implementation Deliverables
1. **Multi-Run Experimental Matrix**: Executed 60 controlled runs across 6 scenarios, Mode A (Response OFF) vs Mode B (Response ON), using 5 pseudo-random seeds ($42, 101, 202, 303, 404$).
2. **Zero-Denominator Precision Fix**: Corrected division-by-zero handling in non-hazardous scenarios (`normal_traffic`, `safe_following`, `different_lane`) to report mathematically accurate **N/A** rather than 100%.
3. **Statistical Aggregation**: Derived Mean, Std, Min, Max, and Student's t-distribution 95% Confidence Intervals ($df = 4, t_{\text{crit}} = 2.776$) for every metric.
4. **Empirical Verification**: Proved that autonomous response preserves $+1.23\text{ m}$ to $+2.94\text{ m}$ of additional headway gap and achieves speed reductions up to $27.7\text{ km/h}$.
5. **Sensitivity Analysis**: Evaluated the trade-off of varying TTC warning thresholds ($3.0\text{ s}, 4.0\text{ s}, 5.0\text{ s}$) on detection latency and warning frequency.

### Phase 8 Implementation Deliverables
1. **Master Pipeline Orchestrator**: Built `run_full_system.py` providing single-command execution of the 10-step vehicular safety workflow.
2. **Dashboard Enhancement**: Extended Streamlit dashboard (`dashboard/app.py`, `charts.py`, `data_loader.py`) across 8 rich interactive sections with Plotly charts and downloadable CSV logs.
3. **Final Results Deliverables Package**: Created `final_results/` organizing logs, figures, summaries, configs, test reports, and an exhaustive data dictionary `README.md`.
4. **Comprehensive Documentation Suite**: Completed phase documentation (Phases 1–8), architecture reference, execution guide, and experiment protocols.
