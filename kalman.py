# =============================================================================
# kalman.py — 2D Constant-Velocity Kalman Filter for UAV Position Tracking
# =============================================================================
#
# WHY A KALMAN FILTER?
# --------------------
# Real GPS receivers inject Gaussian noise into every position reading.
# Naively plotting raw GPS produces jittery, unreliable tracks — useless for
# precise geofence violation detection near zone boundaries.
#
# The Kalman filter is the industry-standard solution for this exact problem.
# It fuses a physics-based motion model (where we *predict* the drone should be)
# with noisy measurements (where GPS *says* it is) to produce optimal estimates.
#
# STATE VECTOR (4D):
#   x = [lat, lon, lat_velocity, lon_velocity]
#
# MEASUREMENT VECTOR (2D):
#   z = [lat_noisy, lon_noisy]   ← raw GPS reading
#
# PREDICT STEP:  x_k|k-1 = F * x_k-1      (dead-reckoning forward in time)
# UPDATE STEP:   x_k     = x_k|k-1 + K*(z_k - H*x_k|k-1)
#                                           (correct using new measurement)
#
# The Kalman Gain K balances trust between model and sensor.
# When GPS noise is high → K is small → trust the model more.
# When model uncertainty is high → K is large → trust the sensor more.
# =============================================================================

import numpy as np
from config import SIMULATION_STEP_SECONDS, GPS_NOISE_STD_M


class KalmanFilter2D:
    """
    Constant-velocity Kalman filter tracking a single UAV in 2D (lat/lon).

    Units are kept in degrees throughout to stay compatible with the rest
    of the pipeline. Noise parameters are converted from metres at init.
    """

    def __init__(self, init_lat: float, init_lon: float,
                 meters_per_lat: float, meters_per_lon: float):
        """
        Parameters
        ----------
        init_lat / init_lon : initial position in degrees
        meters_per_lat      : conversion factor for latitude axis
        meters_per_lon      : conversion factor for longitude axis
        """
        dt = SIMULATION_STEP_SECONDS

        # --- State transition matrix F (constant-velocity model) ---
        # x_new = x + vx*dt,  vx_new = vx   (and same for y)
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1,  0],
            [0, 0, 0,  1],
        ], dtype=float)

        # --- Measurement matrix H ---
        # We only observe position, not velocity
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=float)

        # --- Process noise covariance Q ---
        # Models uncertainty in the motion model itself (e.g. wind, manoeuvres).
        # Small value → we trust our constant-velocity assumption.
        # Tuned for ~0.3 m/s² random acceleration on each axis.
        sigma_a = 0.3  # m/s²
        sigma_a_lat = sigma_a / meters_per_lat
        sigma_a_lon = sigma_a / meters_per_lon

        self.Q = np.diag([
            0.25 * dt**4 * sigma_a_lat**2,
            0.25 * dt**4 * sigma_a_lon**2,
            dt**2 * sigma_a_lat**2,
            dt**2 * sigma_a_lon**2,
        ])

        # --- Measurement noise covariance R ---
        # Reflects GPS accuracy. GPS_NOISE_STD_M converted to degrees.
        sigma_lat = GPS_NOISE_STD_M / meters_per_lat
        sigma_lon = GPS_NOISE_STD_M / meters_per_lon
        self.R = np.diag([sigma_lat**2, sigma_lon**2])

        # --- State vector and covariance ---
        self.x = np.array([init_lat, init_lon, 0.0, 0.0], dtype=float)
        self.P = np.eye(4) * 1e-4   # small initial uncertainty

    def predict(self) -> None:
        """Propagate state forward one time step using the motion model."""
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, lat_meas: float, lon_meas: float) -> None:
        """Correct state estimate using a new GPS measurement."""
        z = np.array([lat_meas, lon_meas])
        y = z - self.H @ self.x                        # innovation
        S = self.H @ self.P @ self.H.T + self.R        # innovation covariance
        K = self.P @ self.H.T @ np.linalg.inv(S)       # Kalman gain
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P

    def step(self, lat_meas: float, lon_meas: float) -> tuple[float, float]:
        """
        Run one full predict→update cycle.

        Returns
        -------
        (lat_filtered, lon_filtered) : smoothed position estimate in degrees
        """
        self.predict()
        self.update(lat_meas, lon_meas)
        return float(self.x[0]), float(self.x[1])

    @property
    def estimated_speed_mps(self) -> float:
        """
        Magnitude of velocity estimate in m/s.
        Derived from the filter state — more stable than frame-differencing
        raw GPS positions, which amplifies noise.
        """
        from config import METERS_PER_LAT_DEGREE, METERS_PER_LON_DEGREE
        vlat_mps = self.x[2] * METERS_PER_LAT_DEGREE
        vlon_mps = self.x[3] * METERS_PER_LON_DEGREE
        return float(np.hypot(vlat_mps, vlon_mps))
