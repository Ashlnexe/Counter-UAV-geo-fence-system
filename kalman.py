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
#
# NUMERICAL STABILITY NOTE:
# -------------------------
# The naive covariance update P = (I - KH)P is algebraically correct but
# numerically fragile. Floating-point errors accumulate over thousands of steps,
# causing P to lose symmetry or positive-definiteness (eigenvalues go negative),
# which breaks the filter silently.
#
# The Joseph form is used instead:
#   P = (I - KH) P (I - KH)^T + K R K^T
# This is equivalent but guaranteed to preserve symmetry and positive-definiteness
# regardless of floating-point rounding. It is the standard in production
# implementations (NASA, JPL, ArduPilot).
# =============================================================================

import numpy as np
from config import SIMULATION_STEP_SECONDS, GPS_NOISE_STD_M


class KalmanFilter2D:
    """
    Constant-velocity Kalman filter tracking a single UAV in 2D (lat/lon).

    Units are kept in degrees throughout to stay compatible with the rest
    of the pipeline. Noise parameters are converted from metres at init.

    Process noise uses the full Continuous White Noise Acceleration (CWNA)
    discretization, including position-velocity cross-terms (dt³/2 · σ²).
    These cross-terms couple position and velocity uncertainty correctly —
    omitting them (diagonal-only Q) underestimates how position uncertainty
    grows when velocity is uncertain.
    """

    def __init__(self, init_lat: float, init_lon: float,
                 meters_per_lat: float, meters_per_lon: float):
        """
        Parameters
        ----------
        init_lat / init_lon : initial position in degrees
        meters_per_lat      : conversion factor for latitude axis  (~110570 at equator)
        meters_per_lon      : conversion factor for longitude axis  (varies with cos(lat))
        """
        # --- Measurement matrix H ---
        # We only observe position, not velocity.
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=float)

        # Cache the process noise terms for faster Q computation
        sigma_a = 0.3  # m/s²
        self.sa_lat2 = (sigma_a / meters_per_lat)**2   # (deg/s²)²
        self.sa_lon2 = (sigma_a / meters_per_lon)**2   # (deg/s²)²

        # --- Measurement noise covariance R ---
        # Reflects GPS accuracy. GPS_NOISE_STD_M converted to degrees separately
        # per axis — latitude and longitude degrees have different metre lengths,
        # especially at non-equatorial latitudes (Bengaluru: ~2% difference).
        sigma_lat = GPS_NOISE_STD_M / meters_per_lat
        sigma_lon = GPS_NOISE_STD_M / meters_per_lon
        self.R = np.diag([sigma_lat**2, sigma_lon**2])

        # --- State vector and covariance ---
        self.x = np.array([init_lat, init_lon, 0.0, 0.0], dtype=float)
        self.P = np.eye(4) * 1e-4   # small initial uncertainty

    def predict(self, dt: float) -> None:
        """Propagate state forward using the motion model with actual time elapsed."""
        # Update state transition matrix F for this dt
        F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1,  0],
            [0, 0, 0,  1],
        ], dtype=float)

        # Update process noise covariance Q for this dt
        Q = np.array([
            [0.25 * dt**4 * self.sa_lat2,  0,                             0.5 * dt**3 * self.sa_lat2,  0                           ],
            [0,                             0.25 * dt**4 * self.sa_lon2,  0,                             0.5 * dt**3 * self.sa_lon2 ],
            [0.5 * dt**3 * self.sa_lat2,   0,                             dt**2 * self.sa_lat2,        0                           ],
            [0,                             0.5 * dt**3 * self.sa_lon2,   0,                             dt**2 * self.sa_lon2       ],
        ])

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, lat_meas: float, lon_meas: float) -> None:
        """
        Correct state estimate using a new GPS measurement.

        Uses the Joseph-form covariance update for numerical stability:
            P = (I - KH) P (I - KH)^T + K R K^T

        This is algebraically identical to the naive P = (I - KH)P but
        preserves symmetry and positive-definiteness under floating-point
        arithmetic — critical for long-running filters (thousands of ticks).
        """
        z = np.array([lat_meas, lon_meas])
        y = z - self.H @ self.x                        # innovation
        S = self.H @ self.P @ self.H.T + self.R        # innovation covariance
        K = self.P @ self.H.T @ np.linalg.inv(S)       # Kalman gain

        self.x = self.x + K @ y

        # Joseph form — numerically stable covariance update
        I_KH = np.eye(4) - K @ self.H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T

    def step(self, lat_meas: float, lon_meas: float, dt: float) -> tuple[float, float]:
        """
        Run one full predict→update cycle.

        Parameters
        ----------
        dt : time elapsed since last step in seconds

        Returns
        -------
        (lat_filtered, lon_filtered) : smoothed position estimate in degrees
        """
        self.predict(dt)
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
