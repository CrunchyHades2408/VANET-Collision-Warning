# Phase 6: Autonomous Vehicle Response Documentation

## 1. Executive Summary & Engineering Objectives

Phase 6 closes the vehicular safety loop. Prior to Phase 6, Phase 5 detected collision hazards and transmitted warnings across the AODV ad-hoc network, but recipient vehicles took no evasive or defensive action.

Phase 6 implements a complete, closed-loop response architecture in which recipient vehicles dynamically react to received warnings in Eclipse SUMO via TraCI. The validated causal workflow is:

$$\text{SUMO Microscopic Telemetry} \longrightarrow \text{Collision Hazard Detection} \longrightarrow \text{Structured Warning Generation} \longrightarrow \text{AODV Wireless Transmission} \longrightarrow \text{Network Delivery Confirmation} \longrightarrow \text{Recipient OBU Validation} \longrightarrow \text{Autonomous Braking Actuation} \longrightarrow \text{Speed Monitoring \& Safe Recovery}$$

---

## 2. Warning Message Structure & Receipt Validation

### 2.1 Standardized Message Fields
The warning message retains and extends the ETSI DENM format established in Phase 5 (`StructuredWarningMessage`):
- `event_id`: Unique identifier of the detected incident (e.g., `EVENT_17.7_V2_V1`).
- `timestamp`: Simulation time of detection.
- `sender_id`: Sensing/lead vehicle ID (e.g., `V1`).
- `sender_ip`: Network IP of sender (e.g., `10.0.0.1`).
- `target_id`: Intended recipient vehicle ID (e.g., `V2`).
- `target_ip`: Network IP of target (e.g., `10.0.0.2`).
- `warning_type`: `EMERGENCY_BRAKE_WARNING`, `FORWARD_COLLISION_WARNING`, or `HEADWAY_ALERT`.
- `risk_level`: `CRITICAL`, `HIGH`, or `MODERATE`.
- `gap_m`: Instantaneous bumper-to-bumper gap (metres).
- `closing_speed_mps`: Relative approach speed (m/s).
- `ttc_s`: Time-to-Collision (seconds).
- `lane_id`: Highway lane identifier.
- `delivery_status`: State transition flag (`GENERATED` $\to$ `TRANSMITTED` $\to$ `DELIVERED`).

### 2.2 Strict Receipt Validation Logic (`WarningReceiptValidator`)
Braking is strictly decoupled from warning generation. A generated warning **never** directly triggers braking before delivery is confirmed. Actuation occurs only after the recipient's On-Board Unit (OBU) validates:
1. **Delivery Confirmation**: `delivery_status == "DELIVERED"`. Unsent or in-flight packets are rejected.
2. **Recipient Matching**: `warning.target_id == receiving_vehicle_id`. Messages delivered to non-target nodes are dropped.
3. **Vehicle Active Check**: The recipient vehicle must exist in the road network (`vehicle_id in traci.vehicle.getIDList()`).
4. **Freshness & Expiration**: Warning age ($\Delta t = t_{\text{current}} - t_{\text{origin}}$) must not exceed $\tau_{\text{max}} = 2.0\text{ s}$. Older packets are rejected as `EXPIRED`.
5. **Deduplication & Cooldown**: Repeated packets with the same `event_id` or arriving while the vehicle is actively responding are rejected as `DUPLICATE` or `ALREADY_BRAKING` to prevent erratic vehicle jerking.

---

## 3. Autonomous Braking via TraCI & Recovery Architecture

### 3.1 Actuation Profiles (`AutonomousVehicleController`)
When a warning passes validation, TraCI issues a controlled deceleration profile via `traci.vehicle.slowDown(veh_id, target_speed, duration)`:
- **CRITICAL Risk** ($\text{TTC} < 2.0\text{ s}$): Target speed $\approx 2.0\text{ m/s}$ ($7.2\text{ km/h}$) over $3.0\text{ s}$, deceleration $\approx 5.5\text{ m/s}^2$.
- **HIGH Risk** ($2.0\text{ s} \le \text{TTC} < 4.0\text{ s}$): Target speed $\approx 8.0\text{ m/s}$ ($28.8\text{ km/h}$) over $3.5\text{ s}$, deceleration $\approx 3.5\text{ m/s}^2$.
- **MODERATE Risk** ($4.0\text{ s} \le \text{TTC} < 6.0\text{ s}$): Target speed $\approx 14.0\text{ m/s}$ ($50.4\text{ km/h}$) over $4.0\text{ s}$, deceleration $\approx 1.5\text{ m/s}^2$.
- **Comfort & Safety Bound**: Deceleration is bounded by $a_{\text{max}} = 7.5\text{ m/s}^2$ to prevent unrealistic instantaneous stops.

### 3.2 Speed Monitoring & Normal Driving Recovery
During each simulation timestep:
1. Vehicle speed and acceleration are sampled via TraCI.
2. Speed reduction ($\Delta v = v_{\text{initial}} - v_{\text{current}}$) is verified and logged once $\Delta v \ge 1.0\text{ m/s}$.
3. When the braking duration has elapsed or safe headway is restored, `traci.vehicle.setSpeed(veh_id, -1)` is invoked, restoring default car-following speed control to SUMO's microscopic driver model.

---

## 4. Empirical Validation & The 8-Step Causal Sequence

The closed-loop simulation was executed under the primary benchmark scenario (`sudden_braking`) and compared directly against an open-loop baseline (response disabled).

### 4.1 Chronological Causal Trace (from `causal_event_sequence.csv`)
| Timestamp (s) | Step | Event Name | Vehicle | Telemetry / Action Details |
|---|---|---|---|---|
| **17.700** | **Step 1** | `HAZARD_DETECTED` | `V2` | Gap = $32.22\text{ m}$, Closing = $5.49\text{ m/s}$, $\text{TTC} = 5.87\text{ s}$, Tier = `MODERATE` |
| **17.700** | **Step 2** | `WARNING_GENERATED` | `V1` | ETSI DENM message generated: `EVENT_17.7_V2_V1` |
| **17.700** | **Step 3** | `WARNING_TRANSMITTED` | `V1` | Injected into AODV layer (`10.0.0.1` $\to$ `10.0.0.2`) |
| **17.702** | **Step 4** | `WARNING_DELIVERED` | `V2` | AODV packet delivered (Latency = $1.96\text{ ms}$) |
| **17.702** | **Step 5** | `WARNING_RECEIVED_VALID` | `V2` | OBU verified message integrity and freshness |
| **17.702** | **Step 6** | `BRAKING_ACTIVATED` | `V2` | TraCI `slowDown()` invoked: $54.5 \to 38.1\text{ km/h}$ over $4.0\text{ s}$ ($a = 1.14\text{ m/s}^2$) |
| **18.200** | **Step 7** | `VEHICLE_SPEED_CHANGED` | `V2` | Deceleration verified: speed dropped to $49.8\text{ km/h}$ |
| **18.700** | **Step 1–6** | Escalation to `HIGH` | `V2` | Threat escalated ($\text{TTC} = 3.93\text{ s}$); braking adjusted to $17.0\text{ km/h}$ |
| **18.902** | **Step 1–6** | Downstream Warning | `V3` | Multi-hop alert received by `V3`; TraCI `slowDown()` invoked: $68.7 \to 48.1\text{ km/h}$ |
| **23.900** | **Step 8** | `RESPONSE_COMPLETED` | `V2` | `setSpeed(-1)` restored default speed control; $\Delta v = 10.3\text{ km/h}$ |
| **27.000** | **Step 8** | `RESPONSE_COMPLETED` | `V3` | `setSpeed(-1)` restored default speed control; $\Delta v = 12.5\text{ km/h}$ |

---

## 5. Verification Test Suite Results

All six required test cases were executed via `test_phase6_response.py` and saved to `results/phase6_unit_test_results.csv`:

| Test Name | Test Description | Result |
|---|---|---|
| `test_successful_warning_receipt_triggers_braking` | Confirmed DELIVERED warning activates TraCI `slowDown()` | **PASSED** |
| `test_warning_generation_without_delivery_does_not_trigger_braking` | Unsent/in-flight warnings do NOT activate braking | **PASSED** |
| `test_duplicate_warning_does_not_trigger_repeated_braking` | Duplicate packets within cooldown are rejected | **PASSED** |
| `test_expired_warning_does_not_trigger_braking` | Packets older than $2.0\text{ s}$ are rejected as EXPIRED | **PASSED** |
| `test_missing_recipient_vehicle_is_handled_safely` | Non-existent/exited vehicles handled gracefully | **PASSED** |
| `test_normal_traffic_without_warning_does_not_trigger_braking` | Normal driving conditions initiate zero braking | **PASSED** |

---

## 6. Generated Visualizations & Telemetry Artifacts

All Phase 6 artifacts are preserved in `phase6_autonomous_response/results/`:
1. `causal_event_sequence.csv`: 37-row log detailing the exact 8-step causal sequence.
2. `autonomous_braking_actuations.csv`: Log of every TraCI `slowDown()` call with speed transitions and deceleration rates.
3. `response_telemetry_log.csv`: Continuous speed, acceleration, and headway telemetry during braking.
4. `warning_validation_log.csv`: Audit log of all accepted and rejected warning messages.
5. `phase6_unit_test_results.csv`: Execution report of the 6 unit tests.
6. `phase6_speed_comparison_response_on_vs_off.png`: Time-series graph demonstrating follower speed reduction when response is enabled versus uncontrolled closing when response is disabled.
7. `phase6_headway_gap_preservation.png`: Headway gap comparison showing that autonomous response preserves a safe buffer well above the physical collision boundary ($2.5\text{ m}$).
