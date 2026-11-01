# Simulation Execution Guide

This guide provides step-by-step instructions to run and test all simulation components developed in Phase 1 and Phase 2.

---

## 1. Phase 1: SUMO Traffic & Sudden-Braking Simulation

All Phase 1 files reside in:
`C:\VANET Collision Warning\sumo\highway_simulation\`

### 1.1 Run Automated Headless Simulation (Recommended for Data Collection)
Open PowerShell or Command Prompt in the project folder and run:
```powershell
cd "C:\VANET Collision Warning\sumo\highway_simulation"
python braking_scenario.py --end-time 40.0
```
- **What happens**: SUMO executes headlessly at high speed, simulates vehicles V1 through V5, initiates sudden braking on V1 at $t = 15.0$ s, calculates real-time TTC and collision warnings, and logs all telemetry to `experiment_logs/braking_experiment_latest.csv`.

### 1.2 Run with SUMO Graphical User Interface (GUI)
To visually observe the vehicles, lanes, and braking deceleration in real time:
```powershell
cd "C:\VANET Collision Warning\sumo\highway_simulation"
python braking_scenario.py --gui --end-time 40.0
```

### 1.3 Recompile the Road Network (If Geometry Changes are Needed)
The network can be regenerated from its source XML definitions at any time:
```powershell
cd "C:\VANET Collision Warning\sumo\highway_simulation"
netconvert --node-files highway.nod.xml --edge-files highway.edg.xml --connection-files highway.con.xml --output-file highway.net.xml
```

---

## 2. Phase 2: OMNeT++ Five-Vehicle Multi-Hop Simulation

All Phase 2 files reside in:
`C:\VANET Collision Warning\omnetpp\five_vehicle_network\`

### 2.1 Run the Standalone Discrete-Event Simulation
To immediately verify the multi-hop network logic, transmission delays, and packet delivery:
```powershell
cd "C:\VANET Collision Warning\omnetpp\five_vehicle_network"
python simulate_five_nodes.py
```
- **Output**: Detailed console trace showing packet initiation at V1 ($t = 15.000\text{ s}$), multi-hop forwarding across V2, V3, and V4, and successful delivery at V5 ($t = 15.00337\text{ s}$) with an end-to-end latency of $3.3653\text{ ms}$.

### 2.2 Run in the OMNeT++ IDE
Once OMNeT++ is installed:
1. Open OMNeT++ IDE.
2. Select `File -> Import... -> General -> Existing Projects into Workspace`.
3. Choose the root folder: `C:\VANET Collision Warning\omnetpp\five_vehicle_network`.
4. Right-click the imported project -> `Build Project`.
5. Navigate to `simulations/omnetpp.ini`, right-click -> `Run As -> OMNeT++ Simulation`.
6. Use the OMNeT++ Qtenv/Tkenv runtime GUI to visualize the message packets moving between vehicle nodes along the highway.
