
import os
import sys
import traci

# --------------------------------------------------
# 1. Locate SUMO's Python tools
# --------------------------------------------------

if "SUMO_HOME" not in os.environ:
    raise EnvironmentError(
        "SUMO_HOME is not configured. "
        "Please configure it before running this script."
    )

tools = os.path.join(os.environ["SUMO_HOME"], "tools")
sys.path.append(tools)

# --------------------------------------------------
# 2. Start SUMO-GUI
# --------------------------------------------------

sumo_binary = "sumo-gui"

traci.start([
    sumo_binary,
    "-c", "highway.sumocfg",
    "--start",
    "--delay", "200"
])

braking_started = False
braking_finished = False

print("\nSimulation started")
print("-" * 55)

# --------------------------------------------------
# 3. Run simulation until 40 seconds
# --------------------------------------------------

while traci.simulation.getTime() < 40:

    traci.simulationStep()

    current_time = traci.simulation.getTime()
    active_vehicles = traci.vehicle.getIDList()

    # --------------------------------------------------
    # 4. Trigger sudden braking at 15 seconds
    # --------------------------------------------------

    if current_time >= 15 and not braking_started:

        if "V1" in active_vehicles:

            traci.vehicle.slowDown("V1", 0.5, 4)

            braking_started = True

            print("\n*** V1 SUDDEN BRAKING INITIATED ***\n")

        else:
            print("WARNING: V1 is no longer on the road at 15 seconds.")
            braking_started = True

    # --------------------------------------------------
    # 5. Resume normal speed control at 25 seconds
    # --------------------------------------------------

    if current_time >= 25 and braking_started and not braking_finished:

        if "V1" in active_vehicles:

            traci.vehicle.setSpeed("V1", -1)

            print("\n*** V1 RESUMED NORMAL SPEED CONTROL ***\n")

        braking_finished = True

    # --------------------------------------------------
    # 6. Display vehicle speeds every 1 second
    # --------------------------------------------------

    if abs(current_time - round(current_time)) < 0.001:

        print(f"\nSimulation time: {current_time:.1f} seconds")

        for vehicle in active_vehicles:

            speed = traci.vehicle.getSpeed(vehicle)
            speed_kmh = speed * 3.6

            print(
                f"{vehicle} | "
                f"Speed: {speed_kmh:.2f} km/h"
            )

# --------------------------------------------------
# 7. Close simulation
# --------------------------------------------------

traci.close()

print("\n" + "-" * 55)
print("Simulation completed successfully.")
