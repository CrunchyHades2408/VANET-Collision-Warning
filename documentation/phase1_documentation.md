# Phase 1 Documentation: SUMO Road Network and Traffic Simulation

## 1. Executive Summary
Phase 1 establishes a validated microscopic traffic simulation using Eclipse SUMO and Python TraCI. It provides the vehicular mobility foundation required for subsequent VANET collision warning and multi-hop communication experiments.

---

## 2. Road Network Architecture (`highway.net.xml`)

### 2.1 Topology and Geometry
The road network represents a continuous 3.0 km multi-lane highway constructed along the longitudinal X-axis using three sequential edge segments:
- **E1**: From Junction `J1` (0.0 m) to Junction `J2` (1000.0 m) – Length: 1000.0 m.
- **E2**: From Junction `J2` (1000.0 m) to Junction `J3` (2000.0 m) – Length: 1000.0 m.
- **E3**: From Junction `J3` (2000.0 m) to Junction `J4` (3000.0 m) – Length: 1000.0 m.
- **Total Highway Length**: 3000.0 m (3.0 km).

### 2.2 Lane Configuration & Connections
- **Lanes per Edge**: 2 lanes across all edges (`Lane 0`: Right lane, `Lane 1`: Left passing lane).
- **Speed Limit**: 27.78 m/s (100.0 km/h).
- **Lane Width**: Standard 3.20 m per lane.
- **Connections**:
  - `E1` Lane 0 connects directly to `E2` Lane 0 via internal lane `:J2_0_0`.
  - `E1` Lane 1 connects directly to `E2` Lane 1 via internal lane `:J2_0_1`.
  - `E2` Lane 0 connects directly to `E3` Lane 0 via internal lane `:J3_0_0`.
  - `E2` Lane 1 connects directly to `E3` Lane 1 via internal lane `:J3_0_1`.

### 2.3 Network Definition Files
To ensure complete reproducibility, the network source definitions are maintained:
- `highway.nod.xml`: Node definitions (J1 dead-end, J2 priority, J3 priority, J4 dead-end).
- `highway.edg.xml`: Edge definitions with lane count and speed limit.
- `highway.con.xml`: Straight lane-to-lane junction connections.
- Compilation command:
  ```powershell
  netconvert --node-files highway.nod.xml --edge-files highway.edg.xml --connection-files highway.con.xml --output-file highway.net.xml
  ```

---

## 3. Traffic Configuration (`highway.rou.xml`)

### 3.1 Vehicle Type Parameters
A standard passenger vehicle type (`vType id="car"`) is configured with realistic vehicular dynamics:
- **Acceleration (`accel`)**: 2.6 m/s²
- **Deceleration (`decel`)**: 4.5 m/s²
- **Emergency Deceleration (`emergencyDecel`)**: 9.0 m/s²
- **Vehicle Length (`length`)**: 5.0 m
- **Minimum Gap (`minGap`)**: 2.5 m
- **Maximum Speed (`maxSpeed`)**: 27.78 m/s (100 km/h)

### 3.2 Vehicle Schedule & Distribution
Five vehicles (`V1` to `V5`) are defined across route `R1` (`E1 E2 E3`):
- **V1 (Leading Vehicle)**: Departs at $t = 0.0$ s on Lane 0, initial speed 10.0 m/s, speed factor 0.8 (desired speed ~22.2 m/s / 80 km/h).
- **V2 (Following Vehicle)**: Departs at $t = 3.0$ s on Lane 0, initial speed 10.0 m/s, speed factor 1.0 (desired speed ~27.8 m/s / 100 km/h).
- **V3 (Follower 2)**: Departs at $t = 6.0$ s on Lane 0, initial speed 10.0 m/s, speed factor 0.9.
- **V4 (Passing Lane)**: Departs at $t = 9.0$ s on Lane 1, initial speed 10.0 m/s, speed factor 1.0.
- **V5 (Passing Lane)**: Departs at $t = 12.0$ s on Lane 1, initial speed 10.0 m/s, speed factor 0.7.

---

## 4. TraCI Sudden-Braking & Collision Detection (`braking_scenario.py`)

### 4.1 Scenario Protocol
1. **Simulation Initialization**: Connects to TraCI, starts SUMO (`sumo` or `sumo-gui`), loads `highway.sumocfg` with time step $\Delta t = 0.1$ s.
2. **Lane-Keeping Mode**: `traci.vehicle.setLaneChangeMode(vid, 512)` locks `V1` and `V2` to Lane 0 to simulate a strict single-lane car-following hazard.
3. **Sudden-Braking Trigger ($t = 15.0$ s)**:
   - TraCI calls `traci.vehicle.slowDown("V1", 0.5, 4.0)` to decelerate V1 from ~80 km/h to 1.8 km/h over 4.0 seconds.
4. **Collision Hazard Calculation**:
   - **Headway Gap**: $d_{\text{gap}} = (x_{V1} - x_{V2}) - L_{V1}$
   - **Closing Speed**: $v_{\text{closing}} = v_{V2} - v_{V1}$
   - **Time-to-Collision (TTC)**:
     $$\text{TTC} = \frac{d_{\text{gap}}}{v_{\text{closing}}} \quad (\text{when } v_{\text{closing}} > 0)$$
   - **Forward Collision Warning (FCW)**: If $\text{TTC} \le 4.0$ s, a collision warning alert is triggered.
5. **Speed Restoration ($t = 25.0$ s)**:
   - Normal speed control is restored using `traci.vehicle.setSpeed("V1", -1)`.
6. **Telemetry Logging**:
   - Simulation state is captured every 0.1 s step and recorded to CSV in `sumo/highway_simulation/experiment_logs/`.

---

## 5. Experimental Results and Validation

### 5.1 Observed Numerical Trajectory
| Time (s) | Event State | V1 Speed (km/h) | V2 Speed (km/h) | Gap (m) | Closing Speed (m/s) | TTC (s) | FCW Alert |
|---|---|---|---|---|---|---|---|
| 14.1 | NORMAL | 79.7 | 92.9 | 47.4 | +3.66 | 12.93 | CLEAR |
| 15.0 | BRAKING_TRIGGERED | 79.7 | 90.7 | 44.0 | +3.52 | 12.52 | CLEAR |
| 17.1 | DECELERATING | 45.7 | 63.7 | 35.4 | +5.00 | 7.08 | CLEAR |
| 19.1 | BRAKING_HOLD | 13.3 | 38.8 | 23.4 | +7.11 | 3.29 | **WARNING** |
| 22.1 | BRAKING_HOLD | 35.5 | 37.4 | 14.8 | +0.52 | 28.25 | CLEAR |
| 25.0 | SPEED_RESTORED | 56.2 | 52.9 | 15.7 | -0.92 | $\infty$ | CLEAR |
| 35.1 | SPEED_RESTORED | 79.7 | 78.4 | 23.2 | -0.37 | $\infty$ | CLEAR |

### 5.2 Validation Checklist
- [x] Highway length extended from 104 m to 3000 m (3.0 km).
- [x] Continuous 3-edge topology (E1, E2, E3) with 2 lanes per edge.
- [x] All 5 vehicles successfully enter and traverse the highway.
- [x] V1 and V2 remain active throughout the braking window (15.0 s to 25.0 s).
- [x] V1 decelerates to 13.3 km/h; V2 detects the hazard and decelerates.
- [x] TTC drops to 3.29 s, triggering the FCW collision warning.
- [x] V1 speed restores at 25.0 s; traffic resumes normal steady state.
- [x] 1,700 telemetry records saved to `experiment_logs/braking_experiment_latest.csv`.
