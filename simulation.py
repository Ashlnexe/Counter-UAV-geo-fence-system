# =============================================================================
# simulation.py — Physics engine for UAV behaviour simulation
# =============================================================================
#
# Each Drone object has:
#   - True position (ground truth, perfect)
#   - Noisy GPS reading (true + Gaussian noise)
#   - Kalman-filtered position (what the system actually tracks)
#
# The threat engine and dashboard use FILTERED positions, not ground truth.
# This is the key architectural decision that makes this a realistic IDS demo
# rather than just a visualisation toy.
#
# Altitude is now a real dynamic variable driven per behaviour profile —
# not a hardcoded constant.
# =============================================================================

import math
import threading
import time
import numpy as np
from collections import deque

from config import (
    BASE_LAT, BASE_LON,
    METERS_PER_LAT_DEGREE, METERS_PER_LON_DEGREE,
    SIMULATION_STEP_SECONDS,
    GPS_NOISE_STD_M,
    ALTITUDE_PROFILES,
)
from kalman import KalmanFilter2D
from threat_engine import threat_engine


class Drone:
    """
    Represents a single simulated UAV with true state, noisy GPS, and
    a Kalman filter producing a clean position estimate.
    """

    def __init__(self, drone_id: str, name: str, behavior_type: str,
                 r_init: float, theta_init: float, speed: float):
        self.id            = drone_id
        self.name          = name
        self.behavior_type = behavior_type
        self.speed         = speed          # m/s
        self.r             = r_init         # metres from centre
        self.theta         = theta_init     # radians
        self.t             = 0.0            # elapsed simulation time (s)
        self.state         = "APPROACH"
        self.heading       = 0.0            # degrees

        # Altitude dynamics
        alt_profile   = ALTITUDE_PROFILES[behavior_type]
        self.alt_base = alt_profile["base"]
        self.alt_amp  = alt_profile["amplitude"]
        self.alt_period = alt_profile["period"]
        self.alt      = self.alt_base

        # Path histories — bounded ring buffers
        self.true_path:     deque = deque(maxlen=60)   # ground truth
        self.filtered_path: deque = deque(maxlen=60)   # Kalman output

        # Compute initial true position
        self._update_true_position()

        # Seed Kalman filter at the true starting position
        # (in practice you'd seed from first GPS fix)
        self.kf = KalmanFilter2D(
            init_lat=self.true_lat,
            init_lon=self.true_lon,
            meters_per_lat=METERS_PER_LAT_DEGREE,
            meters_per_lon=METERS_PER_LON_DEGREE,
        )

        # Public-facing filtered position (starts = true position)
        self.lat = self.true_lat
        self.lon = self.true_lon

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _update_true_position(self) -> None:
        """Convert polar (r, theta) to geographic coords. No noise."""
        self.true_lat = BASE_LAT + (self.r / METERS_PER_LAT_DEGREE) * math.sin(self.theta)
        self.true_lon = BASE_LON + (self.r / METERS_PER_LON_DEGREE) * math.cos(self.theta)
        self.true_path.append((self.true_lat, self.true_lon))

    def _inject_gps_noise(self) -> tuple[float, float]:
        """
        Add zero-mean Gaussian noise to the true position.
        GPS_NOISE_STD_M converted to degrees per axis.
        """
        noise_lat = np.random.normal(0, GPS_NOISE_STD_M / METERS_PER_LAT_DEGREE)
        noise_lon = np.random.normal(0, GPS_NOISE_STD_M / METERS_PER_LON_DEGREE)
        return self.true_lat + noise_lat, self.true_lon + noise_lon

    def _update_altitude(self) -> None:
        """
        Drive altitude as a sinusoid around the base value.
        Each behaviour profile has its own amplitude and period.
        Floor at 10m — drones don't go underground.
        """
        self.alt = max(
            10.0,
            self.alt_base + self.alt_amp * math.sin(2 * math.pi * self.t / self.alt_period)
        )

    def tick(self) -> None:
        """
        Advance the drone's physics state by one simulation step,
        then push a noisy GPS reading through the Kalman filter.
        """
        self._update_true_position()
        self._update_altitude()

        noisy_lat, noisy_lon = self._inject_gps_noise()
        filt_lat, filt_lon   = self.kf.step(noisy_lat, noisy_lon, dt=SIMULATION_STEP_SECONDS)

        self.lat = filt_lat
        self.lon = filt_lon
        self.filtered_path.append((filt_lat, filt_lon))

    @property
    def kalman_speed(self) -> float:
        """Speed estimated from Kalman velocity state (m/s). Noise-robust."""
        return self.kf.estimated_speed_mps


# =============================================================================
# Simulation Engine
# =============================================================================

class SimulationEngine:
    """
    Owns all drones, drives the physics loop in a background thread,
    and exposes a thread-safe snapshot for the dashboard.
    """

    def __init__(self):
        self.drones: list[Drone] = []
        self.is_running = False
        self._thread: threading.Thread | None = None
        self.lock = threading.Lock()
        self._init_drones()

    def _init_drones(self) -> None:
        # UAV-01: Normal circular orbit outside all zones
        uav1 = Drone("UAV-01", "Alpha Scout",    "NORMAL_ORBIT",   3600.0, math.radians(0),   12.0)
        # UAV-02: Slow approach, loiters near BUFFER boundary
        uav2 = Drone("UAV-02", "Shadow Loiterer","LOITERING",      2800.0, math.radians(120),  8.0)
        # UAV-03: High-speed straight-line intrusion into EXCLUSION
        uav3 = Drone("UAV-03", "Striker Intruder","HIGH_SPEED",    3500.0, math.radians(240), 22.0)
        # UAV-04: Sine-wave crossing of MONITORED boundary (perimeter probing)
        uav4 = Drone("UAV-04", "Phantom Tester", "PERIMETER_TEST", 3000.0, math.radians(300), 14.0)
        self.drones = [uav1, uav2, uav3, uav4]

    def start(self) -> None:
        with self.lock:
            if not self.is_running:
                self.is_running = True
                self._thread = threading.Thread(target=self._loop, daemon=True)
                self._thread.start()

    def stop(self) -> None:
        with self.lock:
            self.is_running = False

    def _loop(self) -> None:
        while self.is_running:
            self._update_step()
            time.sleep(SIMULATION_STEP_SECONDS)

    def _update_step(self) -> None:
        with self.lock:
            for d in self.drones:
                d.t += SIMULATION_STEP_SECONDS
                self._apply_behaviour(d)
                d.tick()

            # Pass Drone objects directly — no dict conversion
            threat_engine.process_step(self.drones)

    @staticmethod
    def _apply_behaviour(d: Drone) -> None:
        """Update polar coordinates according to the drone's behaviour script."""
        dt = SIMULATION_STEP_SECONDS

        if d.id == "UAV-01":
            # Steady circular orbit
            d.theta  += (d.speed / d.r) * dt
            d.heading = (math.degrees(d.theta) + 90) % 360

        elif d.id == "UAV-02":
            if d.state == "APPROACH":
                d.r -= d.speed * dt
                if d.r <= 2100.0:
                    d.state = "LOITER"
                    d.speed = 5.0
            else:  # LOITER
                d.theta  += (d.speed / d.r) * dt
                d.heading = (math.degrees(d.theta) + 90) % 360

        elif d.id == "UAV-03":
            d.r      -= d.speed * dt
            d.heading = math.degrees(d.theta) % 360
            if d.r < 150.0:
                d.r = 3500.0   # reset — simulate repeated attack waves

        elif d.id == "UAV-04":
            d.theta += (d.speed / 3000.0) * dt
            d.r      = 3000.0 + 350.0 * math.sin(2.0 * math.pi * d.t / 14.0)
            d.heading = (math.degrees(d.theta) + 90) % 360

    def get_state(self) -> dict:
        """
        Return a thread-safe snapshot of all drone states and active alerts.
        Called by the Dash callback on every interval tick.
        """
        with self.lock:
            return {
                "drones": [
                    {
                        "id":            d.id,
                        "name":          d.name,
                        "behavior_type": d.behavior_type,
                        "lat":           d.lat,
                        "lon":           d.lon,
                        "true_lat":      d.true_lat,
                        "true_lon":      d.true_lon,
                        "alt":           round(d.alt, 1),
                        "speed":         round(d.kalman_speed, 1),   # Kalman-derived
                        "heading":       round(d.heading, 1),
                        "path":          list(d.filtered_path),      # filtered track
                        "true_path":     list(d.true_path),
                    }
                    for d in self.drones
                ],
                "alerts": list(threat_engine.alerts)
            }


# ---------------------------------------------------------------------------
# Module-level singleton
# ---------------------------------------------------------------------------
simulation_engine = SimulationEngine()
