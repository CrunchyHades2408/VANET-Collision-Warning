# Complete System Architecture & Technical Specifications

**Project Title:** VANET-Based Collision Warning and Traffic Management Using AODV Routing  
**Document Type:** Final Architecture & Technical Reference  

---

## 1. High-Level Modular Decomposition

```
+----------------------------------------------------------------------------------------------------+
|                                    VANET COLLISION WARNING SYSTEM                                  |
|                                                                                                    |
|    +-----------------------------+                           +--------------------------------+    |
|    |      SUMO TRAFFIC CORE      |                           |     DISCRETE-EVENT NETWORK     |    |
|    |                             |                           |                                |    |
|    |  * highway.net.xml (3 km)   |   TraCI TCP Protocol      |  * UnitDiskRadio (R = 100m)    |    |
|    |  * highway.rou.xml (5 veh)  | <=======================> |  * IEEE 802.11p DSRC (6 Mbps)  |    |
|    |  * Microscopic Kinematics   |                           |  * RFC 3561 AODV Routing Engine|    |
|    |  * In-Lane Sudden Braking   |                           |  * Dynamic Route Discovery     |    |
|    +-----------------------------+                           +--------------------------------+    |
|                   |                                                          |                     |
|                   v                                                          v                     |
|       [ Telemetry Extraction ]                                    [ Multi-Hop Warning Relay ]      |
|       pos_x, speed, lane_id                                       RREQ / RREP / RERR / UDP         |
|                   |                                                          |                     |
|                   v                                                          v                     |
|    +-----------------------------+                           +--------------------------------+    |
|    |  COLLISION DETECTION ENGINE |                           |      OBU RECEIPT VALIDATOR     |    |
|    |                             |                           |                                |    |
|    |  * Same-Lane Pair Grouping  |                           |  * Delivery Confirmation Check |    |
|    |  * Headway Gap & Approach   |                           |  * Intended Recipient Match    |    |
|    |  * TTC & ISO Risk Tiers     |                           |  * Freshness & Max Age Filter  |    |
|    |  * Hysteresis & Cooldown    |                           |  * Deduplication Guard         |    |
|    +-----------------------------+                           +--------------------------------+    |
|                   |                                                          |                     |
|                   +================= ETSI DENM Message =====================>+                     |
|                                                                              |                     |
|                                                                              v                     |
|                                                              +--------------------------------+    |
|                                                              |  AUTONOMOUS VEHICLE CONTROLLER |    |
|                                                              |                                |    |
|                                                              |  * Risk Deceleration Profiles  |    |
|                                                              |  * TraCI slowDown() Actuation  |    |
|                                                              |  * Speed Reduction Monitoring  |    |
|                                                              |  * Normal Driving Recovery     |    |
|                                                              +--------------------------------+    |
|                                                                              |                     |
|                                                                              v                     |
|                                                                 [ Actuation Command Feedback ]     |
|                                                                 traci.vehicle.slowDown(V2, ...)    |
|                                                                 traci.vehicle.setSpeed(V2, -1)     |
+----------------------------------------------------------------------------------------------------+
```

---

## 2. Component Specifications

### 2.1 Road Traffic Simulator (Eclipse SUMO 1.28.0)
- **Road Network**: Continuous 3.0 km multi-lane highway (`highway.net.xml`), 2 lanes per direction (Lane 0: Right / cruising lane, Lane 1: Left / passing lane). Maximum speed limit: $27.78\text{ m/s}$ ($100\text{ km/h}$).
- **Vehicular Model**: Standard passenger cars (`vType id="car"`), length $l = 5.0\text{ m}$, minimum gap $2.5\text{ m}$, acceleration $2.6\text{ m/s}^2$, deceleration $4.5\text{ m/s}^2$, emergency deceleration $9.0\text{ m/s}^2$.
- **Car-Following Model**: Krauss car-following model with TraCI override (`setLaneChangeMode(512)`) to ensure controlled in-lane following evaluation.

### 2.2 Collision Risk Detector (`CollisionDetector`)
- **Spatial Isolation**: Groups vehicles strictly by `lane_id` to prevent false cross-lane alarms.
- **Headway Gap**: $d_{\text{gap}} = (x_{\text{leader}} - x_{\text{follower}}) - l_{\text{veh}}$.
- **Closing Velocity**: $v_{\text{closing}} = v_{\text{follower}} - v_{\text{leader}}$.
- **Time-to-Collision (TTC)**:
  $$\text{TTC} = \begin{cases} \dfrac{d_{\text{gap}}}{v_{\text{closing}}}, & \text{if } v_{\text{closing}} > 0.01\text{ m/s and } d_{\text{gap}} > 0 \\ \infty, & \text{otherwise} \end{cases}$$
- **Risk Tiers (ISO 15623 / ETSI ITS)**:
  - `CRITICAL`: $\text{TTC} < 2.0\text{ s}$ (Emergency braking required).
  - `HIGH`: $2.0\text{ s} \le \text{TTC} < 4.0\text{ s}$ (Urgent forward collision warning).
  - `MODERATE`: $4.0\text{ s} \le \text{TTC} < 6.0\text{ s}$ (Caution advisory).
  - `SAFE`: $\text{TTC} \ge 6.0\text{ s}$ or $v_{\text{closing}} \le 0$.

### 2.3 Network Routing Layer (RFC 3561 AODV)
- **PHY/MAC Layer**: IEEE 802.11p Dedicated Short-Range Communications (DSRC).
- **Communication Range**: $R = 100.0\text{ m}$ (UnitDisk model).
- **Bitrate**: $6.0\text{ Mbps}$.
- **Propagation Delay**: $0.5\text{ ms}$ per hop.
- **Routing Engine**: Reactive on-demand route discovery:
  - Broadcast `AodvRreq` with hop count and sequence numbers.
  - Intermediate nodes install reverse routes and forward RREQ.
  - Destination unicasts `AodvRrep` back along reverse path.
  - Forward routes installed; buffered UDP warning datagrams released.
  - Mobility link breakage triggers `AodvRerr` to precursors.

### 2.4 Autonomous Vehicle Controller (`AutonomousVehicleController`)
- **Validation**: Enforces delivery confirmation, recipient IP matching, age threshold ($\le 2.0\text{ s}$), and deduplication cooldown ($4.0\text{ s}$).
- **Actuation Profiles**:
  - `CRITICAL`: Target speed $2.0\text{ m/s}$ ($7.2\text{ km/h}$) over $3.0\text{ s}$ ($a \approx 5.5\text{ m/s}^2$).
  - `HIGH`: Target speed $8.0\text{ m/s}$ ($28.8\text{ km/h}$) over $3.5\text{ s}$ ($a \approx 3.5\text{ m/s}^2$).
  - `MODERATE`: Target speed $14.0\text{ m/s}$ ($50.4\text{ km/h}$) over $4.0\text{ s}$ ($a \approx 1.5\text{ m/s}^2$).
  - Physical deceleration limit: $7.5\text{ m/s}^2$.
- **Recovery**: `traci.vehicle.setSpeed(vid, -1)` restores default driver car-following control once threat duration elapses.

---

## 3. Dual Network Implementation Architecture

To ensure strict academic transparency, the repository maintains two parallel network layers:

1. **Native OMNeT++ / INET / Veins Architecture**:
   - Resides in `omnetpp/` (`FiveVehicleNetwork.ned`, `AodvVehicle.ned`, `WarningApp.cc`, `omnetpp.ini`).
   - Prepared for direct compilation in the OMNeT++ 6.0.3 / INET 4.5.2 IDE.
2. **Validated Python RFC 3561 Discrete-Event Simulator**:
   - Resides in `phase6_autonomous_response/` and `phase7_validation/`.
   - Executable natively in standard Python environments without requiring external C++ MSYS2 toolchains.
   - Rigorously models the exact RFC 3561 state machine, packet headers, serialization delays, and distance-based topology dynamics.
