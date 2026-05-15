import time
import math
import threading
import numpy as np
from config import BASE_LAT, BASE_LON, METERS_PER_DEGREE, SIMULATION_STEP_SECONDS
from threat_engine import threat_engine

class Drone:
    def __init__(self, drone_id, name, behavior_type, r_init, theta_init, speed):
        self.id = drone_id
        self.name = name
        self.behavior_type = behavior_type
        self.speed = speed
        self.r = r_init
        self.theta = theta_init
        self.t = 0.0
        self.state = "APPROACH"
        self.alt = 120.0
        self.heading = 0.0
        self.path_history = []
        self.update_position()

    def update_position(self):
        self.lat = BASE_LAT + (self.r / METERS_PER_DEGREE) * math.sin(self.theta)
        self.lon = BASE_LON + (self.r / METERS_PER_DEGREE) * math.cos(self.theta)
        self.path_history.append([self.lat, self.lon])
        if len(self.path_history) > 50:
            self.path_history.pop(0)

class SimulationEngine:
    def __init__(self):
        self.drones = []
        self.is_running = False
        self._thread = None
        self.lock = threading.Lock()
        self._init_drones()

    def _init_drones(self):
        # UAV-01: Normal flight outside all zones (radius 3600m)
        uav1 = Drone("UAV-01", "Alpha Scout", "NORMAL_ORBIT", 3600.0, math.radians(0), 12.0)
        
        # UAV-02: Slow approach toward buffer zone, loiters near boundary
        # Starts at 2800m, approaches to 2100m (within 200m of BUFFER boundary at 2000m)
        uav2 = Drone("UAV-02", "Shadow Loiterer", "LOITERING", 2800.0, math.radians(120), 8.0)
        
        # UAV-03: High-speed intrusion straight into exclusion zone
        uav3 = Drone("UAV-03", "Striker Intruder", "HIGH_SPEED", 3500.0, math.radians(240), 22.0)
        
        # UAV-04: Perimeter testing (sine wave crossing MONITORED boundary at 3000m)
        uav4 = Drone("UAV-04", "Phantom Tester", "PERIMETER_TEST", 3000.0, math.radians(300), 14.0)
        
        self.drones = [uav1, uav2, uav3, uav4]

    def start(self):
        with self.lock:
            if not self.is_running:
                self.is_running = True
                self._thread = threading.Thread(target=self._loop, daemon=True)
                self._thread.start()

    def stop(self):
        with self.lock:
            self.is_running = False

    def _loop(self):
        while self.is_running:
            try:
                self.update_step()
            except Exception as e:
                print(f"Simulation Engine Error: {e}")
            time.sleep(SIMULATION_STEP_SECONDS)

    def update_step(self):
        with self.lock:
            for d in self.drones:
                d.t += SIMULATION_STEP_SECONDS

                if d.id == "UAV-01":
                    # Simple circular orbit using numpy/math
                    d.theta += (d.speed / d.r) * SIMULATION_STEP_SECONDS
                    d.heading = (math.degrees(d.theta) + 90) % 360

                elif d.id == "UAV-02":
                    # Approach and loiter
                    if d.state == "APPROACH":
                        d.r -= d.speed * SIMULATION_STEP_SECONDS
                        if d.r <= 2100.0:
                            d.state = "LOITER"
                            d.speed = 5.0 # Slow down for loitering
                    elif d.state == "LOITER":
                        d.theta += (d.speed / d.r) * SIMULATION_STEP_SECONDS
                    d.heading = (math.degrees(d.theta) + 90) % 360

                elif d.id == "UAV-03":
                    # High speed straight line intrusion towards center
                    d.r -= d.speed * SIMULATION_STEP_SECONDS
                    d.heading = math.degrees(d.theta) % 360
                    # Reset back to start once reaching close to target to simulate repeated waves
                    if d.r < 150.0:
                        d.r = 3500.0

                elif d.id == "UAV-04":
                    # Sine wave overlay to simulate perimeter testing
                    # Base radius 3000m, amplitude 350m, period 14 seconds
                    d.theta += (d.speed / 3000.0) * SIMULATION_STEP_SECONDS
                    d.r = 3000.0 + 350.0 * float(np.sin(2.0 * np.pi * d.t / 14.0))
                    d.heading = (math.degrees(d.theta) + 90) % 360

                d.update_position()

            # Pass the updated state to the threat classification engine
            threat_engine.process_step(self.drones)

    def get_state(self):
        with self.lock:
            drones_snapshot = [
                {
                    "id": d.id,
                    "name": d.name,
                    "behavior_type": d.behavior_type,
                    "lat": d.lat,
                    "lon": d.lon,
                    "alt": round(d.alt, 1),
                    "speed": round(d.speed, 1),
                    "heading": round(d.heading, 1),
                    "path": [pt for pt in d.path_history]
                }
                for d in self.drones
            ]
            
        with threat_engine.lock:
            alerts_snapshot = list(threat_engine.alerts)

        return {
            "drones": drones_snapshot,
            "alerts": alerts_snapshot
        }

# Global singleton instance
simulation_engine = SimulationEngine()
