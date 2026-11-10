# Master Experiment Protocol & Methodology

**Project Title**: VANET-Based Collision Warning and Traffic Management Using AODV Routing  
**Document Type**: Experimental Design, Standards Compliance & Validation Protocol  

---

## 1. Physical & Network Highway Parameters

- **Highway Geometry**: 3000.0 m (3.0 km) continuous multi-lane straight highway.
  - Edge `E1`: 0.0 m to 1000.0 m (Junction J1 to J2)
  - Edge `E2`: 1000.0 m to 2000.0 m (Junction J2 to J3)
  - Edge `E3`: 2000.0 m to 3000.0 m (Junction J3 to J4)
- **Lanes**: 2 lanes per edge (Width = 3.2 m each).
  - Lane 0: Right cruising lane.
  - Lane 1: Left passing lane.
- **Speed Limit**: $27.78\text{ m/s}$ ($100.0\text{ km/h}$).
- **Vehicles**: 5 passenger cars (`V1`, `V2`, `V3`, `V4`, `V5`), length $l = 5.0\text{ m}$, minimum buffer $2.5\text{ m}$, acceleration $2.6\text{ m/s}^2$, deceleration $4.5\text{ m/s}^2$, emergency deceleration $9.0\text{ m/s}^2$.
- **Wireless Radio Model**:
  - Standard: IEEE 802.11p / WAVE Dedicated Short-Range Communications (DSRC).
  - Frequency / Bitrate: 5.9 GHz / $6.0\text{ Mbps}$.
  - Propagation Delay: $0.5\text{ ms}$ per hop.
  - Calibrated Transmission Range: $R = 100.0\text{ m}$ (UnitDisk model).
  - Single-Hop Serialization Delay ($256\text{ bytes}$ payload): $t_{\text{tx}} = \frac{256 \times 8}{6 \times 10^6} \approx 0.3413\text{ ms}$.
  - Single-Hop Minimum Latency: $t_{\text{hop}} = 0.3413 + 0.5000 = 0.8413\text{ ms}$.

---

## 2. Experimental Protocols by Phase

### 2.1 Phase 1 & 2: Baseline Mobility and Linear Forwarding
- **Sudden Braking Trigger**: At $t = 15.0\text{ s}$, $V_1$ initiates sudden braking from $22.2\text{ m/s}$ ($80\text{ km/h}$) to $0.5\text{ m/s}$ over $4.0\text{ s}$ ($a = 5.4\text{ m/s}^2$).
- **Multi-Hop Alert Dissemination**: $V_1$ dispatches 256-byte alert to $V_5$. The message traverses $V_1 \to V_2 \to V_3 \to V_4 \to V_5$ (4 hops) in $3.365\text{ ms}$.

### 2.2 Phase 3: RFC 3561 AODV Protocol Suite
- **Experiment 1 (Initial Route Discovery)**: Empty routing table $\to$ RREQ broadcast across 4 hops $\to$ RREP unicast back along reverse path $\to$ data release ($T_{\text{discovery}} = 4.49\text{ ms}, T_{\text{e2e}} = 7.86\text{ ms}$).
- **Experiment 2 (Route Caching)**: 10 repeated transmissions reuse established forward route without RREQ overhead ($T_{\text{e2e}} = 3.37\text{ ms}$).
- **Experiment 3 & 4 (Link Breakage & Recovery)**: Intermediate vehicle $V_3$ fails or drops out $\to$ upstream node $V_2$ detects loss $\to$ generates `AodvRerr` $\to$ precursor $V_1$ invalidates route $\to$ rediscovery triggered once link is restored.
- **Baseline Comparison**: Direct single-hop transmission from $V_1$ to $V_5$ ($1000\text{ m} > 300\text{ m}$ range) fails with 0% PDR, demonstrating that multi-hop routing is mandatory.

### 2.3 Phase 5 & 6: Collision Detection and Autonomous Braking
- **Ground-Truth Collision Hazard Criterion**:
  $$\text{Hazard Physically Exists} \iff (0 < \text{TTC} \le 6.0\text{ s}) \lor (d_{\text{gap}} \le 35.0\text{ m} \land v_{\text{closing}} \ge 1.0\text{ m/s})$$
- **Detection Classification**:
  - `CRITICAL`: $\text{TTC} < 2.0\text{ s}$ $\to$ Target speed $2.0\text{ m/s}$ ($7.2\text{ km/h}$) over $3.0\text{ s}$.
  - `HIGH`: $2.0\text{ s} \le \text{TTC} < 4.0\text{ s}$ $\to$ Target speed $8.0\text{ m/s}$ ($28.8\text{ km/h}$) over $3.5\text{ s}$.
  - `MODERATE`: $4.0\text{ s} \le \text{TTC} < 6.0\text{ s}$ $\to$ Target speed $14.0\text{ m/s}$ ($50.4\text{ km/h}$) over $4.0\text{ s}$.
  - Deceleration cap: $a \le 7.5\text{ m/s}^2$.
- **Recovery Criterion**: Braking maintained until duration elapses; `traci.vehicle.setSpeed(vid, -1)` restores driver car-following control.

### 2.4 Phase 7: Multi-Seed Comparative Matrix
- **Matrix Dimension**: 6 Scenarios $\times$ 2 Operational Modes (Response ON vs OFF) $\times$ 5 Independent Runs = 60 experimental executions.
- **Stochastic Seeds**: 42, 101, 202, 303, 404 with microscopic initial speed jitter ($\pm 0.2\text{ m/s}$).
- **Evaluation Statistics**: Mean, Standard Deviation, Min, Max, and Student's t-distribution 95% Confidence Intervals:
  $$CI_{95} = \bar{x} \pm t_{0.025, df} \cdot \frac{s}{\sqrt{N}} \quad (t_{0.025, 4} = 2.776 \text{ for } N = 5)$$
- **Zero-Denominator Rule**: Undefined precision ($\text{TP} + \text{FP} = 0$) reported as **N/A**, preserving scientific integrity.
