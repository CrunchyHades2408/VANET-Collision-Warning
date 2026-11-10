# Phase 7: Advanced Experiments and Statistical Validation

## 1. Executive Summary & Experimental Objectives

Phase 7 evaluates the completed VANET collision warning and autonomous traffic management system through controlled, repeatable, and statistically rigorous experiments. 

Building upon the 6 highway scenarios and the closed-loop autonomous response engine validated in Phase 6, Phase 7 conducts a systematic comparative evaluation:
1. **Open-Loop Baseline (Mode A)**: Collision detection active, warnings transmitted, but recipient vehicle autonomous response **disabled** (human / unconnected following behavior).
2. **Closed-Loop System (Mode B)**: Collision detection active, warnings transmitted via RFC 3561 AODV, and recipient vehicles execute **autonomous braking and recovery** upon confirmed receipt.

The evaluation executes **60 controlled experimental runs** (6 scenarios $\times$ 2 modes $\times$ 5 independent runs with distinct random seeds) plus parameter sensitivity testing across warning thresholds ($\text{TTC} = 3.0\text{ s}, 4.0\text{ s}, 5.0\text{ s}$).

---

## 2. Experimental Protocol & Scenario Definitions

### 2.1 Environmental Matrix
- **Simulator Toolchain**: Eclipse SUMO 1.28.0 coupled with Python TraCI ($\Delta t = 0.1\text{ s}$) and discrete-event AODV wireless model ($R = 100\text{ m}$, $6\text{ Mbps}$, $0.5\text{ ms}$ propagation).
- **Repetitions & Random Seeds**: $N = 5$ independent runs per condition using pseudo-random seeds ($42, 101, 202, 303, 404$) with microscopic speed jitter ($\pm 0.2\text{ m/s}$) to guarantee stochastic independence.

### 2.2 Operational Scenarios Evaluated
1. **`sudden_braking`**: Primary benchmark. Leading vehicle $V_1$ abruptly decelerates from $22.2\text{ m/s}$ ($80\text{ km/h}$) to $0.5\text{ m/s}$ at $t = 15.0\text{ s}$. Follower $V_2$ closes rapidly, and downstream vehicle $V_3$ is exposed to pileup risk.
2. **`normal_traffic`**: Baseline free-flow highway cruise. All 5 vehicles travel at standard speeds with large headways ($> 60\text{ m}$). Objective: verify zero false alarms.
3. **`safe_following`**: Controlled highway following. $V_1$ performs gentle deceleration from $22\text{ m/s}$ to $18\text{ m/s}$ over $5.0\text{ s}$, maintaining a safe buffer ($> 50\text{ m}$). Objective: verify absence of unnecessary emergency braking.
4. **`high_speed_approach`**: Dynamic tailgating hazard. $V_2$ accelerates to $28.0\text{ m/s}$ ($100.8\text{ km/h}$) while $V_1$ decelerates to $5.0\text{ m/s}$, creating a high closing speed ($v_{\text{closing}} \approx 23\text{ m/s}$).
5. **`different_lane`**: Lateral proximity validation. $V_4$ in Lane 1 catches up and overtakes $V_1$ in Lane 0 with longitudinal distance $< 12.7\text{ m}$. Objective: prove lane isolation and confirm zero rear-end false alarms across adjacent lanes.
6. **`risk_recovery`**: Dynamic hazard clearance. $V_1$ brakes sharply at $t = 12.0\text{ s}$, triggering an urgent warning to $V_2$, then accelerates back to cruise speed ($25.0\text{ m/s}$) at $t = 18.0\text{ s}$. Objective: evaluate dynamic recovery to normal car-following.

---

## 3. Mathematical Metric Formulations & Zero-Denominator Rigor

### 3.1 Collision Detection Metrics
- **True Positives (TP)**: Physical collision hazard present AND detected.
- **False Positives (FP)**: No physical hazard present BUT falsely detected.
- **True Negatives (TN)**: No physical hazard present AND correctly deemed safe.
- **False Negatives (FN)**: Physical hazard present BUT missed by detector.
- **Precision**:
  $$\text{Precision} = \begin{cases} \dfrac{\text{TP}}{\text{TP} + \text{FP}}, & \text{if } (\text{TP} + \text{FP}) > 0 \\ \textbf{N/A}, & \text{if } (\text{TP} + \text{FP}) = 0 \end{cases}$$
  > **Zero-Denominator Rule**: In scenarios where zero hazardous events are generated (`normal_traffic`, `safe_following`, `different_lane`), Precision is mathematically undefined and reported as **N/A** (never fabricated as 100%).
- **Recall (Sensitivity)**: $\text{Recall} = \frac{\text{TP}}{\text{TP} + \text{FN}}$ (or 1.0 when $\text{FN} = 0$ and no ground truth hazard existed).
- **F1-Score**: Harmonic mean of Precision and Recall: $\text{F1} = 2 \cdot \frac{\text{Precision} \cdot \text{Recall}}{\text{Precision} + \text{Recall}}$.
- **False Alarm Rate (FAR)**: $\text{FAR} = \frac{\text{FP}}{\text{FP} + \text{TN}}$.

### 3.2 Traffic Safety Metrics
- **Minimum Headway Gap ($d_{\text{min}}$)**: Front-bumper to rear-bumper distance at closest proximity. Physical collision threshold is $2.5\text{ m}$.
- **Gap Preserved ($\Delta d_{\text{gap}}$)**:
  $$\Delta d_{\text{gap}} = d_{\text{min, Closed-Loop}} - d_{\text{min, Baseline}}$$
- **Follower Speed Reduction ($\Delta v$)**: Peak deceleration achieved by responding follower vehicles upon receiving warning.
- **Braking Response Time ($t_{\text{react}}$)**: Elapsed time between warning generation and actuation of TraCI `slowDown()`.

### 3.3 AODV Network Metrics
- **Packet Delivery Ratio (PDR)**: $\text{PDR} = \frac{N_{\text{delivered}}}{N_{\text{generated}}} \times 100\%$.
- **End-to-End Latency ($T_{\text{e2e}}$)**: Total propagation, serialization, and multi-hop forwarding delay from origin to recipient.
- **Routing Overhead ($O_{\text{routing}}$)**: Total RFC 3561 control packets ($N_{\text{RREQ}} + N_{\text{RREP}} + N_{\text{RERR}}$).

---

## 4. Empirical Comparative Evaluation (Response ON vs. OFF)

The benchmark matrix below summarizes the mean metrics across 5 independent runs per scenario condition:

| Scenario | Mode | Min Gap ($d_{\text{min}}$) | Gap Preserved | Braking Actions | Speed Drop ($\Delta v$) | PDR | Latency ($T_{\text{e2e}}$) | Detection F1 |
|---|---|---|---|---|---|---|---|---|
| **sudden_braking** | **Open-Loop (OFF)** | $14.17\text{ m}$ | Baseline | 0 | $0.0\text{ km/h}$ | 100.0% | $1.29\text{ ms}$ | 69.2% |
|  | **Closed-Loop (ON)** | **$15.40\text{ m}$** | **$+1.23\text{ m}$** | **5** | **$27.66\text{ km/h}$** | **100.0%** | **$1.29\text{ ms}$** | **88.5%** |
| **high_speed_approach** | **Open-Loop (OFF)** | $20.89\text{ m}$ | Baseline | 0 | $0.0\text{ km/h}$ | 100.0% | $1.96\text{ ms}$ | 26.7% |
|  | **Closed-Loop (ON)** | **$23.83\text{ m}$** | **$+2.94\text{ m}$** | **1** | **$17.11\text{ km/h}$** | **100.0%** | **$1.96\text{ ms}$** | **26.7%** |
| **risk_recovery** | **Open-Loop (OFF)** | $22.99\text{ m}$ | Baseline | 0 | $0.0\text{ km/h}$ | 100.0% | $1.96\text{ ms}$ | 37.5% |
|  | **Closed-Loop (ON)** | **$25.86\text{ m}$** | **$+2.86\text{ m}$** | **1** | **$18.69\text{ km/h}$** | **100.0%** | **$1.96\text{ ms}$** | **37.5%** |
| **normal_traffic** | Both Modes | $61.51\text{ m}$ | Safe buffer | 0 | $0.0\text{ km/h}$ | 100.0% | N/A | **N/A\*** |
| **safe_following** | Both Modes | $61.51\text{ m}$ | Safe buffer | 0 | $0.0\text{ km/h}$ | 100.0% | N/A | **N/A\*** |
| **different_lane** | Both Modes | $12.71\text{ m}$ | Cross-lane | 0 | $0.0\text{ km/h}$ | 100.0% | N/A | **N/A\*** |

*\*Note: In non-hazardous scenarios, zero warnings are emitted; precision is reported as N/A.*

### Key Scientific Findings:
1. **Safety Margin Expansion**: Autonomous vehicle response preserves an additional **$1.23\text{ m} - 2.94\text{ m}$** buffer under emergency braking and high closing speeds.
2. **Collision Avoidance**: In all closed-loop runs, the minimum gap remained well above the $2.5\text{ m}$ physical impact threshold ($d_{\text{min}} \ge 15.13\text{ m}$ in `sudden_braking`), preventing secondary rear-end pileups.
3. **Deceleration Responsiveness**: Follower vehicles achieved speed reductions between **$17.1\text{ km/h}$ and $27.7\text{ km/h}$** within $2.0\text{ ms}$ of network delivery.
4. **Zero False Alarms**: In non-hazardous scenarios (`normal_traffic`, `safe_following`, `different_lane`), $\text{FP} = 0$, $\text{FAR} = 0.0\%$, and zero false braking actuations occurred.

---

## 5. Statistical Analysis with 95% Confidence Intervals

For each metric across $N = 5$ independent runs, the sample mean $\bar{x}$, sample standard deviation $s$, and 95% Confidence Interval were computed using Student's t-distribution ($df = 4, t_{0.025, 4} = 2.776$):

$$CI_{95} = \bar{x} \pm 2.776 \cdot \frac{s}{\sqrt{5}}$$

### Representative Statistical Confidence Intervals (`phase7_statistical_summary.csv`):
- **`sudden_braking` Closed-Loop Minimum Gap**:
  $$\bar{x} = 15.40\text{ m}, \quad s = 0.34\text{ m}, \quad CI_{95} = [14.98\text{ m}, 15.82\text{ m}]$$
- **`sudden_braking` Open-Loop Minimum Gap**:
  $$\bar{x} = 14.17\text{ m}, \quad s = 0.23\text{ m}, \quad CI_{95} = [13.88\text{ m}, 14.46\text{ m}]$$
- **`sudden_braking` Follower Speed Reduction ($\Delta v$)**:
  $$\bar{x} = 27.66\text{ km/h}, \quad s = 0.21\text{ km/h}, \quad CI_{95} = [27.40\text{ km/h}, 27.92\text{ km/h}]$$
- **AODV End-to-End Latency**:
  $$\bar{x} = 1.290\text{ ms}, \quad s = 0.005\text{ ms}, \quad CI_{95} = [1.284\text{ ms}, 1.296\text{ ms}]$$

The non-overlapping 95% confidence intervals between Closed-Loop ($[14.98, 15.82]\text{ m}$) and Open-Loop ($[13.88, 14.46]\text{ m}$) demonstrate statistically significant gap preservation at $p < 0.05$.

---

## 6. Sensitivity Analysis Across Warning Thresholds

Testing across 3 TTC warning thresholds ($3.0\text{ s}, 4.0\text{ s}, 5.0\text{ s}$) in `sudden_braking`:
- **Conservative ($\text{TTC} = 5.0\text{ s}$)**: Dispatches warnings earlier, allowing smoother deceleration ($a \approx 1.2\text{ m/s}^2$).
- **Standard ($\text{TTC} = 4.0\text{ s}$)**: Balanced trade-off; produces 5 warnings with optimal headway preservation ($15.40\text{ m}$).
- **Aggressive ($\text{TTC} = 3.0\text{ s}$)**: Delays warning trigger until hazard is more immediate, requiring sharper braking deceleration ($a \approx 2.5\text{ m/s}^2$).

---

## 7. Generated Visualizations & Data Artifacts

All Phase 7 artifacts are preserved in `phase7_validation/results/`:
1. `phase7_all_experiment_runs.csv`: Complete raw dataset across all 60 experiment runs.
2. `phase7_statistical_summary.csv`: Scenario-level aggregated statistics (Mean, Std, Min, Max, 95% CI).
3. `phase7_response_comparison_matrix.csv`: Response Enabled vs. Disabled comparison matrix.
4. `phase7_threshold_sensitivity_analysis.csv`: Sensitivity data across warning thresholds.
5. `phase7_scenario_detection_performance.png`: Multi-metric bar chart with N/A labels for zero-hazard scenarios.
6. `phase7_minimum_gap_preservation.png`: Headway gap preservation contrasting Response ON vs OFF.
7. `phase7_network_pdr_and_latency.png`: AODV PDR and End-to-End transmission latency with error bars.
8. `phase7_braking_response_and_speed_reduction.png`: Deceleration speed reductions achieved by responding vehicles.
9. `phase7_threshold_sensitivity_curves.png`: Impact of warning thresholds on detection latency and warning frequency.
