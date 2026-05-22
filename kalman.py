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

class KalmanFilter2D:
    """
    Constant-velocity Kalman filter tracking a single UAV in 2D (Easting, Northing).
    Operates strictly in Cartesian meters (e.g., UTM projection).
    """
    def __init__(self, init_easting: float, init_northing: float):
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=float)

        # Process noise variance (acceleration variance m^2/s^4)
        sigma_a = 0.3
        self.sa2 = sigma_a**2

        # Measurement noise covariance (GPS error in meters)
        # Standard deviation e.g. 2.0 meters
        gps_noise_std = 2.0
        self.R = np.diag([gps_noise_std**2, gps_noise_std**2])

        self.x = np.array([init_easting, init_northing, 0.0, 0.0], dtype=float)
        self.P = np.eye(4) * 1e-4

    def predict(self, dt: float) -> None:
        """Propagate state forward using the motion model."""
        if dt <= 0: return

        F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1,  0],
            [0, 0, 0,  1],
        ], dtype=float)

        # Q cross-terms correctly couple position and velocity uncertainty
        q_pos = 0.25 * dt**4 * self.sa2
        q_cov = 0.5 * dt**3 * self.sa2
        q_vel = dt**2 * self.sa2

        Q = np.array([
            [q_pos, 0,     q_cov, 0    ],
            [0,     q_pos, 0,     q_cov],
            [q_cov, 0,     q_vel, 0    ],
            [0,     q_cov, 0,     q_vel],
        ])

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, easting_meas: float, northing_meas: float) -> None:
        """Correct state estimate using Joseph form."""
        z = np.array([easting_meas, northing_meas])
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y

        I_KH = np.eye(4) - K @ self.H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T

    def step(self, easting_meas: float, northing_meas: float, dt: float) -> tuple[float, float]:
        self.predict(dt)
        self.update(easting_meas, northing_meas)
        return float(self.x[0]), float(self.x[1])
        
    def project_future(self, steps: int, dt: float) -> list[tuple[float, float]]:
        """
        Phase 3: Trajectory Prediction
        Projects the current state forward 'steps' times without updating covariance.
        Returns a list of predicted (easting, northing) positions.
        """
        F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1,  0],
            [0, 0, 0,  1],
        ], dtype=float)
        
        future_path = []
        x_proj = self.x.copy()
        for _ in range(steps):
            x_proj = F @ x_proj
            future_path.append((float(x_proj[0]), float(x_proj[1])))
        return future_path

    @property
    def estimated_speed_mps(self) -> float:
        """Magnitude of velocity estimate in m/s."""
        return float(np.hypot(self.x[2], self.x[3]))
