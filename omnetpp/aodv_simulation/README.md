# Phase 3: AODV Multi-Hop VANET Simulation

This folder contains the complete Phase 3 Ad hoc On-Demand Distance Vector (AODV, RFC 3561) routing simulation.

## Directory Structure
- `ned/`
  - `WarningApp.ned`: Application module definition implementing UDP-based collision warning dissemination.
  - `AodvVehicle.ned`: Vehicle on-board unit (OBU) host extending INET's `AodvRouter`.
  - `FiveVehicleAodvNetwork.ned`: Five-node highway network topology with UnitDiskRadio wireless channels.
- `src/`
  - `WarningApp.h`: C++ header for the collision warning application.
  - `WarningApp.cc`: C++ implementation integrating with INET's `UdpSocket`.
- `simulations/`
  - `omnetpp.ini`: Complete INET configuration file covering all 5 experiment configurations and baseline.
- `results/`
  - `aodv_experiment_metrics.csv`: Tabular performance metrics (PDR, E2E delay, overhead, discovery time, hops).
  - `aodv_packet_trace.csv`: Detailed chronological event trace of RREQ, RREP, RERR, and UDP data packets.
  - `aodv_pdr_comparison.png`: Bar chart comparing Packet Delivery Ratio across all test scenarios.
  - `aodv_e2e_delay.png`: End-to-end latency analysis across multi-hop scenarios.
  - `aodv_routing_overhead.png`: Breakdown of RREQ, RREP, and RERR control packets.
  - `aodv_route_discovery_time.png`: Initial route discovery latency.
  - `aodv_hop_count.png`: Multi-hop traversal count verification.
- `run_aodv_experiments.py`: Complete automated simulation engine and experiment runner.

## Quick Execution
Run the automated experiment suite:
```powershell
python run_aodv_experiments.py
```
This executes all 5 experiments plus baseline, exports CSV metrics, and regenerates analytical figures.

## Running inside OMNeT++ IDE
1. Open OMNeT++ IDE with INET Framework imported.
2. Import `C:\VANET Collision Warning\omnetpp\aodv_simulation`.
3. Right-click project -> `Build Project`.
4. Right-click `simulations/omnetpp.ini` -> `Run As -> OMNeT++ Simulation`.
5. Select any of the defined configurations:
   - `InitialRouteDiscovery`
   - `RepeatedTransmission`
   - `LinkFailure`
   - `MultiHopWarningDelivery`
   - `BaselineSingleHop`
