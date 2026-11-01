# SUMO Highway Simulation - Phase 1

This folder contains the complete SUMO highway road network, traffic demand, and TraCI sudden-braking simulation for Phase 1.

## Files
- `highway.nod.xml`: Node/junction coordinates ($x = 0, 1000, 2000, 3000$ m).
- `highway.edg.xml`: Edge definitions for E1, E2, E3 (2 lanes, 27.78 m/s speed limit).
- `highway.con.xml`: Straight lane-to-lane connections at junctions J2 and J3.
- `highway.net.xml`: Compiled 3.0 km SUMO network file.
- `highway.rou.xml`: Vehicle demand file with 5 vehicles (V1 leading, V2 following).
- `highway.sumocfg`: Simulation configuration file.
- `braking_scenario.py`: Python script controlling vehicle braking, calculating TTC, and logging telemetry.
- `experiment_logs/`: Directory storing timestamped and latest CSV telemetry data.
- `backup_original/`: Original backup of Phase 1 files before extending network length.

## Execution
Run headless:
```powershell
python braking_scenario.py --end-time 40.0
```

Run with GUI:
```powershell
python braking_scenario.py --gui --end-time 40.0
```
