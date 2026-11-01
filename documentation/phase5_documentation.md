# Phase 5: Collision Detection and Warning Generation

## 1. Overview and Engineering Objectives

Phase 5 introduces the proactive safety intelligence into the VANET simulation architecture. Operating on top of the SUMO microscopic traffic simulation and the Veins/OMNeT++ wireless communication environment, Phase 5 couples:
1. Continuous microscopic vehicle telemetry extraction.
2. Physical rear-end collision risk evaluation using Time-to-Collision (TTC), bumper-to-bumper headway gaps, and relative closing speeds.
3. Multi-tier hazard classification adhering to ISO 15623 (Forward Collision Warning Systems) and ETSI EN 302 637-2/3 (DENM / Cooperative Awareness).
4. Automated structured warning message generation and injection into the dynamic RFC 3561 AODV ad hoc routing protocol.
5. Strict ground-truth baseline evaluation across 6 distinct highway operational scenarios.

---

## 2. Theoretical Principles and Mathematical Formulations

### 2.1 Bumper-to-Bumper Distance ($d_{\text{gap}}$)
In SUMO, a vehicle's longitudinal reference coordinate $x_i$ denotes the front bumper tip along the road corridor. For two adjacent vehicles travelling in the same lane where the leading vehicle is $L$ and the following vehicle is $F$ ($x_L > x_F$):

$$d_{\text{gap}} = (x_L - x_F) - l_{\text{veh}}$$

where $l_{\text{veh}} = 5.0\text{ m}$ is the vehicle length specified in `highway.rou.xml`.

### 2.2 Relative Closing Speed ($v_{\text{closing}}$)
Closing speed measures how fast the following vehicle is closing in on the leader:

$$v_{\text{closing}} = v_F - v_L$$

- If $v_{\text{closing}} > 0$: The follower is approaching the leader (collision risk possible).
- If $v_{\text{closing}} \le 0$: The follower is moving at or below the leader's speed (safe condition).

### 2.3 Time-to-Collision ($\text{TTC}$)
Time-to-Collision estimates the time remaining before impact if both vehicles maintain their instantaneous velocities without intervention:

$$\text{TTC} = \begin{cases} \dfrac{d_{\text{gap}}}{v_{\text{closing}}}, & \text{if } v_{\text{closing}} > 0.01\text{ m/s and } d_{\text{gap}} > 0 \\ \infty, & \text{otherwise} \end{cases}$$

### 2.4 Multi-Tier Risk Classification
Based on ISO 15623 and ETSI ITS standards:

| Risk Tier | TTC Range | Operational Definition |
| :--- | :--- | :--- |
| **CRITICAL** | $\text{TTC} < 2.0\text{ s}$ | Emergency braking required immediately |
| **HIGH** | $2.0\text{ s} \le \text{TTC} < 4.0\text{ s}$ | Urgent forward collision warning alert |
| **MODERATE** | $4.0\text{ s} \le \text{TTC} < 6.0\text{ s}$ | Caution advisory / headway maintenance alert |
| **LOW** | $\text{TTC} \ge 6.0\text{ s}$ | Safe following distance |
| **SAFE** | $\text{TTC} = \infty$ | No closing motion ($v_{\text{closing}} \le 0$) |

### 2.5 Hysteresis and Suppression Cooldown
To prevent broadcast storming and oscillatory false triggering caused by small sensor fluctuations:
- **Hysteresis**: Transition between risk tiers requires crossing threshold boundaries by a margin ($\Delta_{\text{hyst}} = 0.2\text{ s}$).
- **Cooldown Window**: Once a warning is emitted for a vehicle pair, duplicate transmissions of the same tier are suppressed for $\tau_{\text{cooldown}} = 4.0\text{ s}$.

---

## 3. Architecture and Component Design

```
+-------------------------------------------------------------+
|                  SUMO Traffic Simulation                    |
|           (V1, V2, V3 in Lane 0; V4, V5 in Lane 1)          |
+------------------------------+------------------------------+
                               | TraCI (0.1s step)
                               v
+-------------------------------------------------------------+
|             Phase 5 Collision Detector                      |
|  - Lane-specific grouping (Lane 0 vs Lane 1 isolation)      |
|  - Headway gap & closing speed computation                  |
|  - TTC calculation & risk tier classification               |
+------------------------------+------------------------------+
                               | Hazardous Event Triggered
                               v
+-------------------------------------------------------------+
|             Structured Warning Generator                    |
|  - ETSI DENM format (ActionID, EventID, Position, TTC)      |
|  - Source: Leader/Sensing Node -> Target: Following Node    |
+------------------------------+------------------------------+
                               | UDP Injection
                               v
+-------------------------------------------------------------+
|               INET / AODV Protocol Engine                   |
|  - Dynamic Route Discovery (RREQ / RREP)                    |
|  - Multi-hop Forwarding / Link Failure Recovery             |
+-------------------------------------------------------------+
```

### 3.1 Module Descriptions
- `collision_detector.py`: Core logic for grouping vehicles by lane, computing bumper-to-bumper headways, relative speeds, TTC, and applying hysteresis/cooldown filtering.
- `warning_generator.py`: Converts detected collision risks into structured warning frames (`StructuredWarningMessage`) containing event IDs, risk tiers, gap, closing speed, and source/target IP addresses.
- `ground_truth_evaluator.py`: Compares instantaneous detections against objective physics-based collision risk criteria, recording TP, FP, TN, FN, precision, recall, and detection latencies.
- `phase5_cosimulation.py`: Master test runner coupling SUMO TraCI, the collision detector, and the AODV wireless network simulation across 6 named test scenarios.

---

## 4. Test Scenarios and Quantitative Evaluation

All 6 test scenarios were executed over 300 simulation timesteps ($t = 0$ to $30\text{ s}$):

| Scenario | Objective | Ground Truth Status | Precision | Recall | Warnings Emitted | Warnings Delivered | Delivery Ratio |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **sudden_braking** | V1 decelerates rapidly from 22 m/s to 0.5 m/s at $t=15\text{ s}$ | Hazard occurs ($t \approx 17.5\text{s} - 21.0\text{s}$) | **100.0%** | **52.9%** | 4 | 4 | **100%** |
| **normal_traffic** | Free-flow cruise at standard highway velocities | No hazardous braking | **100.0%** | -- | 0 | 0 | -- |
| **safe_following** | Gentle decelerations maintaining headway $> 30\text{ m}$ | Safe buffer maintained | **100.0%** | -- | 0 | 0 | -- |
| **high_speed_approach** | V2 approaches rapidly behind decelerating V1 | High closing speed hazard | **100.0%** | **15.4%** | 1 | 1 | **100%** |
| **different_lane** | V4 overtakes V1 in adjacent lane (lateral proximity $< 5\text{ m}$) | Cross-lane proximity only | **100.0%** | -- | 0 | 0 | -- |
| **risk_recovery** | V1 decelerates, triggers warning, then accelerates back to cruise | Hazard cleared dynamically | **100.0%** | **23.1%** | 1 | 1 | **100%** |

### 4.1 Evaluation Highlights
1. **Zero False Alarms (100% Precision)**: No warning messages were generated in non-hazardous states (`normal_traffic`, `safe_following`, `different_lane`).
2. **Lane Filtering Verification**: In `different_lane`, V4 and V1 drew level laterally with zero longitudinal gap, yet zero rear-end warnings were emitted because the detector correctly isolates vehicles by lane index.
3. **AODV Warning Transmission**: Every emitted warning packet was delivered to the designated follower vehicle via AODV with **100% packet delivery ratio** and an end-to-end latency of $0.84\text{ ms} - 1.96\text{ ms}$.

---

## 5. Artifacts and Generated Data

All telemetry logs and plots are preserved in `phase5_collision_detection/results/`:
- `vehicle_state_log.csv`: Microscopic coordinates, speed, and lane telemetry per vehicle per timestep.
- `collision_detection_log.csv`: Headway gap, closing speed, TTC, and risk tier for every same-lane pair.
- `warning_transmission_log.csv`: AODV transmission records (timestamps, source/dest IPs, delay, hop count).
- `ground_truth_evaluations.csv`: Timestep-by-timestep TP/FP/TN/FN metrics and detection latencies.
- `phase5_scenario_evaluation_metrics.csv`: Aggregate benchmark summary across all 6 scenarios.
- Dynamic plots: `phase5_sudden_braking_ttc_dynamics.png`, `phase5_high_speed_approach_ttc_dynamics.png`, etc.
