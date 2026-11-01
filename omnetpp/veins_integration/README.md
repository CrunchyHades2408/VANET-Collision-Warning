# Phase 4: SUMO–OMNeT++ Veins Co-Simulation with INET AODV

This directory contains the Phase 4 co-simulation implementation integrating Eclipse SUMO with OMNeT++ using Veins and preserving INET's RFC 3561 AODV routing protocol.

## Directory Structure
- `ned/`
  - `VeinsWarningApp.ned`: Application module definition for vehicular warning dissemination.
  - `VeinsAodvVehicle.ned`: Mobile vehicle host combining TraCIMobility with INET AODV routing and 802.11p NIC.
  - `VeinsHighwayNetwork.ned`: Co-simulation network integrating TraCIScenarioManager, UnitDiskRadioMedium ($R = 100\text{ m}$), and dynamic vehicle hosts.
- `src/`
  - `VeinsWarningApp.h`: C++ header for the warning dissemination application.
  - `VeinsWarningApp.cc`: C++ implementation integrating with INET's `UdpSocket`.
- `simulations/`
  - `omnetpp.ini`: Complete INET/Veins configuration file defining mobility parameters, TraCI port, AODV protocol timers, and experiment configurations.
- `results/`
  - `veins_mobility_sync.csv`: Microscopic vehicular trajectory log (1,450 records) mapping SUMO vehicles to OMNeT++ nodes.
  - `veins_aodv_network_trace.csv`: Chronological packet event trace under dynamic mobility.
  - `veins_experiment_metrics.csv`: Performance metrics summary table for all 5 Phase 4 experiments.
  - `veins_vehicle_trajectories.png`: Time-space trajectory diagram of all 5 vehicles.
  - `veins_dynamic_distances.png`: Dynamic inter-vehicle distances vs. DSRC radio range threshold ($100\text{ m}$).
  - `veins_mobility_aodv_pdr.png`: Packet Delivery Ratio across mobility experiments.
  - `veins_mobility_e2e_delay.png`: End-to-end latency distribution across dynamic hops.
  - `veins_topology_snapshot.png`: Co-simulation vehicular topology and wireless connectivity graph at $t = 15.0\text{ s}$.
- `run_veins_cosimulation.py`: Complete automated SUMO-OMNeT++ TraCI co-simulation runner.

## Vehicle Mapping
| SUMO Vehicle ID | OMNeT++ Node Name | Assigned IPv4 Address | Role in Co-Simulation |
|---|---|---|---|
| `V1` | `vehicle_V1` | `10.0.0.1` | Incident vehicle (brakes at $t = 15\text{ s}$), Alert Initiator |
| `V2` | `vehicle_V2` | `10.0.0.2` | Immediate following vehicle (Lane 0), Forwarder |
| `V3` | `vehicle_V3` | `10.0.0.3` | Second following vehicle (Lane 0), Forwarder |
| `V4` | `vehicle_V4` | `10.0.0.4` | Passing lane vehicle (Lane 1), Forwarder |
| `V5` | `vehicle_V5` | `10.0.0.5` | Passing lane vehicle (Lane 1), Target Destination |

## Quick Execution
To run the automated co-simulation and regenerate all CSV logs and analytical plots:
```powershell
python run_veins_cosimulation.py
```
