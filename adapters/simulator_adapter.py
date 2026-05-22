import math
import time
import numpy as np
from typing import Generator

from adapters.base_adapter import SensorAdapter
from data_ingestion import TelemetryFrame
from config import (
    BASE_LAT, BASE_LON,
    METERS_PER_LAT_DEGREE, METERS_PER_LON_DEGREE,
    SIMULATION_STEP_SECONDS,
    GPS_NOISE_STD_M,
    ALTITUDE_PROFILES,
)

class SimDrone:
    """Internal physics model for a simulated UAV."""
    def __init__(self, drone_id: str, behavior_type: str, r_init: float, theta_init: float, speed: float):
        self.id = drone_id
        self.behavior_type = behavior_type
        self.r = r_init
        self.theta = theta_init
        self.speed = speed
        self.t = 0.0
        self.state = "APPROACH"
        
        alt_profile = ALTITUDE_PROFILES[behavior_type]
        self.alt_base = alt_profile["base"]
        self.alt_amp = alt_profile["amplitude"]
        self.alt_period = alt_profile["period"]

    def tick(self, dt: float) -> TelemetryFrame:
        """Advance physics and return a noisy telemetry frame."""
        self.t += dt
        self._apply_behaviour(dt)
        
        # True position
        true_lat = BASE_LAT + (self.r / METERS_PER_LAT_DEGREE) * math.sin(self.theta)
        true_lon = BASE_LON + (self.r / METERS_PER_LON_DEGREE) * math.cos(self.theta)
        
        # Altitude
        alt = max(10.0, self.alt_base + self.alt_amp * math.sin(2 * math.pi * self.t / self.alt_period))
        
        # Add GPS noise
        noise_lat = np.random.normal(0, GPS_NOISE_STD_M / METERS_PER_LAT_DEGREE)
        noise_lon = np.random.normal(0, GPS_NOISE_STD_M / METERS_PER_LON_DEGREE)
        noisy_lat = true_lat + noise_lat
        noisy_lon = true_lon + noise_lon
        
        heading = (math.degrees(self.theta) + 90) % 360 if self.id in ["UAV-01", "UAV-04"] else (math.degrees(self.theta) % 360)
        if self.id == "UAV-02" and self.state == "LOITER":
             heading = (math.degrees(self.theta) + 90) % 360
             
        return TelemetryFrame(
            drone_id=self.id,
            timestamp=time.time(),
            lat=noisy_lat,
            lon=noisy_lon,
            alt=alt,
            speed=self.speed,
            heading=heading,
            source="simulator"
        )

    def _apply_behaviour(self, dt: float) -> None:
        if self.id == "UAV-01":
            self.theta += (self.speed / self.r) * dt
        elif self.id == "UAV-02":
            if self.state == "APPROACH":
                self.r -= self.speed * dt
                if self.r <= 2100.0:
                    self.state = "LOITER"
                    self.speed = 5.0
            else:
                self.theta += (self.speed / self.r) * dt
        elif self.id == "UAV-03":
            self.r -= self.speed * dt
            if self.r < 150.0:
                self.r = 3500.0
        elif self.id == "UAV-04":
            self.theta += (self.speed / 3000.0) * dt
            self.r = 3000.0 + 350.0 * math.sin(2.0 * math.pi * self.t / 14.0)

class SimulatorAdapter(SensorAdapter):
    """
    Simulates drone flights natively as an adapter stream.
    Replaces the old SimulationEngine background thread.
    """
    def __init__(self):
        super().__init__()
        self.drones = []

    def connect(self) -> bool:
        self.drones = [
            SimDrone("UAV-01", "NORMAL_ORBIT", 3600.0, math.radians(0), 12.0),
            SimDrone("UAV-02", "LOITERING", 2800.0, math.radians(120), 8.0),
            SimDrone("UAV-03", "HIGH_SPEED", 3500.0, math.radians(240), 22.0),
            SimDrone("UAV-04", "PERIMETER_TEST", 3000.0, math.radians(300), 14.0)
        ]
        return True

    def disconnect(self) -> None:
        self.drones = []

    def get_stream(self) -> Generator[TelemetryFrame, None, None]:
        while self.is_connected():
            now = time.time()
            for drone in self.drones:
                frame = drone.tick(SIMULATION_STEP_SECONDS)
                frame.timestamp = now  # ensure concurrent timestamps
                yield frame
            time.sleep(SIMULATION_STEP_SECONDS)
