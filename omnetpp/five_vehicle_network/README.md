# OMNeT++ Five-Vehicle Multi-Hop Network - Phase 2

This directory contains the introductory five-vehicle communication network project for Phase 2.

## Directory Structure
- `ned/`
  - `CollisionWarningMsg.msg`: Message definition for collision alerts (senderId, targetId, hopCount, speed, coords).
  - `VehicleNode.ned`: Simple module defining the vehicle communication node (OBU) and gates (`gLeft`, `gRight`).
  - `FiveVehicleNetwork.ned`: Network topology module interconnecting vehicles V1 through V5 via a 6 Mbps DSRC highway channel.
- `src/`
  - `VehicleNode.h`: C++ header declaring the VehicleNode simple module class and signal definitions.
  - `VehicleNode.cc`: C++ implementation handling self-messages, emergency message dispatch, and multi-hop forwarding.
- `simulations/`
  - `omnetpp.ini`: Simulation configuration setting parameters for nodes V1-V5, end time (25s), and log outputs.
- `results/`: Directory for recording vector (`.vec`) and scalar (`.sca`) simulation outputs.
- `simulate_five_nodes.py`: Standalone discrete-event execution script that models and validates the exact NED/C++ multi-hop timing.

## Multi-Hop Transmission Trace
1. **t = 15.00000 s**: Vehicle **V1** detects sudden braking and dispatches `CollisionWarningMsg` targeted to **V5**.
2. **t = 15.00084 s**: Vehicle **V2** receives packet (Hop 1, latency 0.84 ms) and forwards to **V3**.
3. **t = 15.00168 s**: Vehicle **V3** receives packet (Hop 2, latency 1.68 ms) and forwards to **V4**.
4. **t = 15.00252 s**: Vehicle **V4** receives packet (Hop 3, latency 2.52 ms) and forwards to **V5**.
5. **t = 15.00337 s**: Target Vehicle **V5** receives packet (Hop 4, total end-to-end latency: 3.37 ms). Driver Forward Collision Warning (FCW) is triggered.

## Running in OMNeT++ IDE
1. Open OMNeT++ IDE.
2. Select `File -> Import... -> General -> Existing Projects into Workspace`.
3. Choose `C:\VANET Collision Warning\omnetpp\five_vehicle_network`.
4. Right-click project -> `Build Project`.
5. Right-click `simulations/omnetpp.ini` -> `Run As -> OMNeT++ Simulation`.

## Running the Standalone Discrete-Event Model
```powershell
python simulate_five_nodes.py
```
