# Phase 4 Documentation: SUMO–OMNeT++ Co-Simulation Using Veins & INET AODV

## 1. Executive Summary & Integration Architecture

Phase 4 establishes real-time bi-directional co-simulation coupling microscopic vehicular mobility in Eclipse SUMO with dynamic wireless communication and Ad hoc On-Demand Distance Vector (AODV, RFC 3561) routing in OMNeT++ using the Veins vehicular simulation framework.

```
+----------------------------------------------------------------------------------------------------+
|                                    VEINS CO-SIMULATION ARCHITECTURE                                |
|                                                                                                    |
|    +-----------------------------+                           +--------------------------------+    |
|    |        ECLIPSE SUMO         |                           |            OMNeT++             |    |
|    | (Microscopic Traffic Model) |                           |  (Network Simulation Engine)   |    |
|    |                             |                           |                                |    |
|    |  * highway.net.xml (3 km)   |   TraCI TCP Protocol      |  * VeinsHighwayNetwork.ned     |    |
|    |  * highway.rou.xml (5 veh)  | <=======================> |  * TraCIScenarioManager        |    |
|    |  * Car-Following Dynamics   |   (Port 9999 / Loopback)  |  * UnitDiskRadioMedium (100m)  |    |
|    |  * Sudden-Braking Protocol  |                           |  * Ipv4NetworkConfigurator     |    |
|    +-----------------------------+                           +--------------------------------+    |
|                   |                                                          |                     |
|                   v                                                          v                     |
|       [ SUMO Active Vehicles ]                                    [ Dynamic OMNeT++ Nodes ]        |
|          V1, V2, V3, V4, V5       ====== (1:1 Synchronized) =====>   vehicle_V1 .. vehicle_V5      |
|                                                                              |                     |
|                                                                              v                     |
|                                                                   [ INET AODV Routing Layer ]      |
|                                                                   * Dynamic Link Graph G(t)        |
|                                                                   * On-Demand RREQ / RREP          |
|                                                                   * Mobility-Induced RERR          |
+----------------------------------------------------------------------------------------------------+
```

---

## 2. Environment & Version Compatibility Report

- **Host Operating System**: Windows 11 / Windows 10 x64 (AMD64)
- **Microscopic Traffic Simulator**: Eclipse SUMO 1.28.0 (Installed at `C:\Program Files (x86)\SUMO\`)
- **Discrete-Event Network Simulator**: OMNeT++ 6.0.3 (or 5.7.1)
- **Network Protocol Library**: INET Framework 4.5.2 (or 4.2.8)
- **Vehicular Simulation Bridge**: Veins 5.2 / `veins_inet` integration layer
- **Communication Interface**: SUMO Traffic Control Interface (TraCI) over TCP socket loopback (`127.0.0.1:9999`)
- **Python Automation & Data Science Stack**: Python 3.12.10, `pandas`, `matplotlib`, `numpy`

### Integration Compatibility Notes
Direct coupling between OMNeT++ and SUMO requires `TraCIScenarioManager` to bridge the disparate time domains:
- SUMO advances micro-timesteps ($\Delta t = 0.1\text{ s}$) calculating vehicle acceleration, lane-following, and braking.
- OMNeT++ advances event-by-event with sub-microsecond precision, calculating radio wave propagation, packet serialization, and MAC collisions.
- The `TraCIScenarioManager` coordinates both engines at each timestep $\Delta t$, querying SUMO vehicle coordinates and updating the OMNeT++ node mobility submodules.

---

## 3. Vehicle Mapping & Mobility Synchronization

### 3.1 Entity Mapping Table
| SUMO Vehicle ID | OMNeT++ Node Identity | Assigned IP | Route / Lane | Dynamic Role in Co-Simulation |
|---|---|---|---|---|
| **V1** | `vehicle_V1` | `10.0.0.1` | `R1` / Lane 0 | Leading vehicle; triggers sudden braking at $t = 15\text{ s}$; Alert Initiator |
| **V2** | `vehicle_V2` | `10.0.0.2` | `R1` / Lane 0 | Following vehicle ($3\text{ s}$ delay); detects hazard, slows down; Forwarder |
| **V3** | `vehicle_V3` | `10.0.0.3` | `R1` / Lane 0 | Second follower ($6\text{ s}$ delay); intermediate ad-hoc relay node |
| **V4** | `vehicle_V4` | `10.0.0.4` | `R1` / Lane 1 | Passing lane vehicle ($9\text{ s}$ delay); intermediate ad-hoc relay node |
| **V5** | `vehicle_V5` | `10.0.0.5` | `R1` / Lane 1 | Passing lane vehicle ($12\text{ s}$ delay); target destination for collision alerts |

### 3.2 Synchronized Trajectory Telemetry
The microscopic trajectory synchronization was executed across 1,450 timesteps and logged to [`results/veins_mobility_sync.csv`](file:///C:/VANET%20Collision%20Warning/omnetpp/veins_integration/results/veins_mobility_sync.csv).
- At each timestep, SUMO exports: `pos_x`, `pos_y`, `speed_kmh`, `heading_deg`, `lane_id`, and `edge_id`.
- The corresponding OMNeT++ node reflects the exact spatial coordinates with zero spatial drift.
- Vehicle entry ($V1$ at $0\text{ s}$, $V2$ at $3\text{ s}$, $V3$ at $6\text{ s}$, $V4$ at $9\text{ s}$, $V5$ at $12\text{ s}$) dynamically instantiates network nodes; vehicle exit terminates the corresponding node.

---

## 4. Dynamic Wireless Connectivity & Radio Range Calibration

### 4.1 Radio Model & Physical Calibration
- **Radio Standard**: Dedicated Short-Range Communications (DSRC / IEEE 802.11p)
- **Data Rate**: $6.0\text{ Mbps}$
- **Propagation Delay**: $0.5\text{ ms}$
- **Calibrated Radio Communication Range**: $R = 100.0\text{ m}$ (UnitDiskRadio model)

### 4.2 Spatial Analysis of Moving Vehicles on the Highway
At $t = 15.0\text{ s}$ (the instant V1 initiates sudden braking), the microscopic positions on the highway are:
- $V_1$: $x = 296.84\text{ m}$
- $V_2$: $x = 247.49\text{ m}$ $\implies d(V_1, V_2) = 49.35\text{ m} \le 100\text{ m}$ (**Direct Link Active**)
- $V_3$: $x = 173.61\text{ m}$ $\implies d(V_2, V_3) = 73.88\text{ m} \le 100\text{ m}$ (**Direct Link Active**)
- $V_4$: $x = 99.81\text{ m}$ $\implies d(V_3, V_4) = 73.80\text{ m} \le 100\text{ m}$ (**Direct Link Active**)
- $V_5$: $x = 42.56\text{ m}$ $\implies d(V_4, V_5) = 57.25\text{ m} \le 100\text{ m}$ (**Direct Link Active**)

**Critical Network Consequence**:
- The total distance between $V_1$ ($296.84\text{ m}$) and $V_5$ ($42.56\text{ m}$) is **$254.28\text{ m}$**, which **greatly exceeds the $100\text{ m}$ radio range**!
- Therefore, direct single-hop communication between $V_1$ and $V_5$ is **physically impossible**.
- The **only** way a warning message from $V_1$ can reach $V_5$ is via **multi-hop AODV relaying** across intermediate moving vehicles:
  $$V_1 \longrightarrow V_2 \longrightarrow V_3 \longrightarrow V_4 \longrightarrow V_5 \quad (4\text{ Wireless Hops})$$

---

## 5. Preservation & Verification of AODV Routing under Mobility

The actual RFC 3561 AODV protocol implemented in Phase 3 was integrated directly into the co-simulation. The packet trace from [`results/veins_aodv_network_trace.csv`](file:///C:/VANET%20Collision%20Warning/omnetpp/veins_integration/results/veins_aodv_network_trace.csv) proves the protocol's operation under dynamic vehicular mobility:

### 5.1 Dynamic Route Discovery Trace ($t = 15.0\text{ s}$)
1. **$t = 15.000000\text{ s}$**: $V_1$ ($x = 296.8\text{ m}$) detects sudden braking, buffers warning packet `EXP3_MULTIHOP_1`, and broadcasts `RREQ_3` for destination `10.0.0.5`.
2. **$t = 15.000564\text{ s}$**: $V_2$ ($x = 247.5\text{ m}$) receives RREQ ($d = 49.4\text{ m} \le 100\text{ m}$), installs reverse route to $V_1$ (Hop 1), and rebroadcasts RREQ.
3. **$t = 15.001128\text{ s}$**: $V_3$ ($x = 173.6\text{ m}$) receives RREQ ($d = 73.9\text{ m} \le 100\text{ m}$), installs reverse route to $V_1$ via $V_2$ (Hop 2), and rebroadcasts RREQ.
4. **$t = 15.001692\text{ s}$**: $V_4$ ($x = 99.8\text{ m}$) receives RREQ ($d = 73.8\text{ m} \le 100\text{ m}$), installs reverse route to $V_1$ via $V_3$ (Hop 3), and rebroadcasts RREQ.
5. **$t = 15.002256\text{ s}$**: $V_5$ ($x = 42.6\text{ m}$) receives RREQ ($d = 57.3\text{ m} \le 100\text{ m}$), matches destination IP, and generates unicast `RREP`.
6. **$t = 15.002815\text{ s} \to 15.004491\text{ s}$**: RREP travels back along the reverse path $V_5 \to V_4 \to V_3 \to V_2 \to V_1$, installing forward routes at each intermediate moving vehicle.
7. **$t = 15.004491\text{ s}$**: Route established at $V_1$ in **$4.491\text{ ms}$**. Buffered packet is transmitted.
8. **$t = 15.007856\text{ s}$**: Warning packet is successfully delivered to $V_5$ via path:
   $$\text{vehicle\_V1} \longrightarrow \text{vehicle\_V2} \longrightarrow \text{vehicle\_V3} \longrightarrow \text{vehicle\_V4} \longrightarrow \text{vehicle\_V5}$$
   **Total End-to-End Latency**: **$7.856\text{ ms}$** ($4.491\text{ ms}$ discovery + $3.365\text{ ms}$ 4-hop data propagation).

### 5.2 Mobility-Induced Link Failure & RERR Notification ($t = 22.0\text{ s}$)
- Between $t = 15\text{ s}$ and $t = 22\text{ s}$, $V_1$ and $V_2$ decelerate to $0.5\text{ m/s}$ while $V_4$ and $V_5$ cruise at $100\text{ km/h}$.
- The headway gap between vehicles stretches beyond the $100\text{ m}$ radio range boundary.
- When $V_1$ transmits packet `EXP4_MOBILITY_FAIL` at $t = 22.0\text{ s}$, intermediate connectivity is broken.
- AODV detects link failure, marks routes as `INVALID`, generates `AodvRerr`, and notifies precursors.

### 5.3 Route Recovery Upon Vehicle Convergence ($t = 32.0\text{ s}$)
- At $t = 25.0\text{ s}$, normal speed control is restored in SUMO. $V_1$ accelerates back to cruise speed ($80\text{ km/h}$).
- The inter-vehicle distance closes back within the $100\text{ m}$ radio range.
- At $t = 32.0\text{ s}$, packet `EXP5_RECOVERY` triggers an on-demand rediscovery (`RREQ_5`), successfully establishing a new route and restoring packet delivery ($100\%\text{ PDR}$).

---

## 6. Experimental Results & Performance Evaluation

All metrics were captured during active co-simulation and saved to [`results/veins_experiment_metrics.csv`](file:///C:/VANET%20Collision%20Warning/omnetpp/veins_integration/results/veins_experiment_metrics.csv):

| Experiment Configuration | Packets Sent | Packets Delivered | PDR (%) | Average E2E Delay | Route Discovery Time | Routing Overhead | Observations & Validation |
|---|---|---|---|---|---|---|---|
| **Exp 1: Normal Mobility Sync** | 0 | 0 | **100.0%** | $0.000\text{ ms}$ | $0.000\text{ ms}$ | 0 pkts | 1,450 TraCI timesteps; 5 vehicles synchronized 1:1 with zero drift. |
| **Exp 2: Wireless P2P Moving** | 4 | 4 | **100.0%** | **$0.841\text{ ms}$** (Cached) | $1.123\text{ ms}$ | 2 pkts | 1-hop point-to-point delivery between moving vehicles $V_1 \to V_2$. |
| **Exp 3: Dynamic Multi-Hop AODV** | 2 | 2 | **100.0%** | **$5.611\text{ ms}$** (Cached: $3.365\text{ ms}$) | **$4.491\text{ ms}$** | 8 pkts | **4-hop on-demand discovery** across moving highway nodes $V_1 \to V_2 \to V_3 \to V_4 \to V_5$. |
| **Exp 4: Mobility Link Failure** | 1 | 0 | **0.0%** | N/A | N/A | 1 pkt | Braking caused headway $> 100\text{ m}$; link broke; AODV issued `AodvRerr`. |
| **Exp 5: Route Recovery** | 1 | 1 | **100.0%** | **$1.964\text{ ms}$** | $1.123\text{ ms}$ | 2 pkts | Speeds restored; vehicles converged $< 100\text{ m}$; fresh route established. |

---

## 7. Analytical Figures Generated

Five high-resolution analytical figures have been generated and saved to the results directory:
1. [`results/veins_vehicle_trajectories.png`](file:///C:/VANET%20Collision%20Warning/omnetpp/veins_integration/results/veins_vehicle_trajectories.png): Time-space diagram displaying continuous trajectories of all 5 vehicles along the 3 km highway.
2. [`results/veins_dynamic_distances.png`](file:///C:/VANET%20Collision%20Warning/omnetpp/veins_integration/results/veins_dynamic_distances.png): Pairwise headway distances plotted against the $100\text{ m}$ radio range boundary over time.
3. [`results/veins_mobility_aodv_pdr.png`](file:///C:/VANET%20Collision%20Warning/omnetpp/veins_integration/results/veins_mobility_aodv_pdr.png): Bar chart of Packet Delivery Ratio across all 5 mobility experiments.
4. [`results/veins_mobility_e2e_delay.png`](file:///C:/VANET%20Collision%20Warning/omnetpp/veins_integration/results/veins_mobility_e2e_delay.png): Latency distribution (1-hop vs. 4-hop multi-hop, initial discovery vs. route caching).
5. [`results/veins_topology_snapshot.png`](file:///C:/VANET%20Collision%20Warning/omnetpp/veins_integration/results/veins_topology_snapshot.png): Visual spatial snapshot of vehicular nodes and wireless link connectivity lines at $t = 15.0\text{ s}$.

---

## 8. Execution Commands

### 8.1 Execute Automated Co-Simulation
```powershell
cd "C:\VANET Collision Warning\omnetpp\veins_integration"
python run_veins_cosimulation.py
```

### 8.2 Run with SUMO Graphical Visualization
```powershell
cd "C:\VANET Collision Warning\sumo\highway_simulation"
python braking_scenario.py --gui --end-time 40.0
```

---

## 9. Scope & Transition to Phase 5

### What was completed in Phase 4:
- Real-time TraCI synchronization between SUMO vehicle mobility and OMNeT++ dynamic nodes.
- Preserved actual RFC 3561 AODV routing protocol across moving vehicular topologies.
- Validated dynamic 4-hop multi-hop communication, route caching, mobility-induced link breakage, and route recovery.
- Generated comprehensive CSV telemetry logs and 5 analytical plots.

### What is reserved for Phase 5 (Collision Warning & Traffic Management):
- **Closed-Loop Vehicle Reaction**: Connecting the received OMNeT++ warning messages back to TraCI so following vehicles in SUMO react autonomously (e.g. initiating braking or changing lanes).
- **Secondary Crash Prevention**: Measuring safety metrics (reduction in collision risk, time-to-collision margins, secondary pileup avoidance).
- **Streamlit Results Dashboard**: Developing an interactive web dashboard for real-time visualization of vehicle trajectories, AODV routing events, and traffic management performance metrics.
