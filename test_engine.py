import time
from app import get_state

for i in range(3):
    time.sleep(1.0)
    state = get_state()
    drones = state["drones"]
    print(f"Step {i}, Drones: {len(drones)}, Alerts: {len(state['alerts'])}")
    if drones:
        print(f"  Drone 1 lat: {drones[0]['lat']}, lon: {drones[0]['lon']}")
