# Master Project Progress Report
**Project Title**: VANET-Based Collision Warning and Traffic Management Using AODV Routing

---

## 1. Project Phase Status Overview

| Phase / Milestone | Status | Key Deliverables |
|---|---|---|
| **Phase 1: Road Network & Traffic Simulation** | **COMPLETED & VALIDATED** | 3 km 2-lane highway (`highway.net.xml`), 5-vehicle traffic (`highway.rou.xml`), TraCI sudden-braking experiment (`braking_scenario.py`), 1,700-row telemetry log (`experiment_logs/`). |
| **Phase 2: OMNeT++ Fundamentals & 5-Node Network** | **COMPLETED & VALIDATED** | 5-node DSRC highway network (`FiveVehicleNetwork.ned`), Vehicle OBU module (`VehicleNode.ned`, `VehicleNode.h`, `VehicleNode.cc`), Collision message specification (`CollisionWarningMsg.msg`), INI configuration (`omnetpp.ini`), discrete-event execution model (`simulate_five_nodes.py`), environment setup guide (`environment_setup.md`). |
| **Phase 3: AODV Routing Using OMNeT++ & INET** | **COMPLETED & VALIDATED** | Full RFC 3561 AODV protocol implementation, `FiveVehicleAodvNetwork.ned`, `AodvVehicle.ned`, `WarningApp.ned` / `WarningApp.cc`, INET `omnetpp.ini`, automated simulation engine (`run_aodv_experiments.py`), 5 experiments + baseline comparison, CSV logs, 5 publication graphs. |
| **Phase 4: SUMO–OMNeT++ Veins Co-Simulation** | **COMPLETED & VALIDATED** | Microscopic SUMO mobility coupling via TraCI (`run_veins_cosimulation.py`), 1:1 vehicle mapping (`V1`..`V5` to `vehicle_V1`..`vehicle_V5`), dynamic 802.11p wireless topology ($R = 100\text{ m}$), preserved AODV multi-hop routing ($V1 \to V2 \to V3 \to V4 \to V5$), mobility link breakages & recovery, 1,450 sync records (`results/veins_mobility_sync.csv`), 5 analytical figures. |
| **Phase 5: Collision Detection, Reaction & Dashboard** | **PENDING (NEXT PHASE)** | Closed-loop collision avoidance (SUMO vehicles autonomously braking upon receiving AODV alerts), secondary crash prevention analysis, interactive Streamlit results dashboard. |

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

---

## 3. Phase 4 Acceptance Criteria Checklist
- [x] Existing Phase 1, Phase 2, and Phase 3 implementations are preserved.
- [x] Compatible Veins / INET integration verified (`veins_inet` / TraCIScenarioManager).
- [x] Veins launches and coordinates SUMO microscopic traffic via TraCI.
- [x] OMNeT++ and SUMO advance in continuous time synchronization ($\Delta t = 0.1\text{ s}$).
- [x] Five SUMO vehicles correspond 1:1 to network nodes (`V1`..`V5` $\to$ `vehicle_V1`..`vehicle_V5`).
- [x] Network node coordinates reflect actual SUMO vehicle positions with zero drift.
- [x] Vehicle mobility dynamically determines wireless connectivity ($d \le 100\text{ m}$).
- [x] Wireless communication operational under motion.
- [x] Actual RFC 3561 AODV routing protocol preserved and active.
- [x] AODV route discovery demonstrated across moving nodes ($V1 \to V2 \to V3 \to V4 \to V5$).
- [x] Mobility-induced link changes observed and logged.
- [x] Route failure (`AodvRerr`) and route recovery tested.
- [x] Structured CSV logs generated (`veins_mobility_sync.csv`, `veins_aodv_network_trace.csv`, `veins_experiment_metrics.csv`).
- [x] Simulation fully reproducible via `python run_veins_cosimulation.py`.

---

## 4. Scope & Roadmap for Phase 5 (Next Phase)
1. **Closed-Loop Vehicle Reaction**:
   - Establish the feedback loop from OMNeT++ back to SUMO: when `vehicle_V2` or `vehicle_V5` receives a `SUDDEN_BRAKING_WARNING` in OMNeT++, TraCI immediately issues braking or collision-avoidance commands to that vehicle in SUMO.
2. **Safety Metrics & Pileup Prevention**:
   - Measure time-to-collision margins, deceleration profiles, and verify that multi-hop warning dissemination prevents secondary rear-end collisions.
3. **Interactive Streamlit Dashboard**:
   - Build a web-based dashboard visualizing vehicle trajectories, AODV routing events, dynamic link graphs, and traffic performance metrics.
