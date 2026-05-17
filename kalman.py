import numpy as np
import math

class KalmanFilter:
    """
    Constant-velocity Kalman filter for 2D drone tracking.
    State vector: [x, y, vx, vy]
    - x, y   : position in meters from base coordinate
    - vx, vy : velocity in m/s
    """

    def __init__(self, dt, process_noise=1.0, measurement_noise=5.0):
        self.dt = dt  # time step in seconds

        # State vector [x, y, vx, vy]
        self.x = np.zeros((4, 1))

        # State transition matrix — where will we be next tick?
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1,  0],
            [0, 0, 0,  1]
        ], dtype=float)

        # Measurement matrix — we observe x, y only (not velocity directly)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=float)

        # Process noise covariance — how much we distrust our own model
        q = process_noise
        self.Q = np.array([
            [q,   0,   0,   0],
            [0,   q,   0,   0],
            [0,   0,   q*2, 0],
            [0,   0,   0,   q*2]
        ], dtype=float)

        # Measurement noise covariance — how much we distrust the GPS
        r = measurement_noise
        self.R = np.array([
            [r, 0],
            [0, r]
        ], dtype=float)

        # Initial error covariance — high uncertainty at start
        self.P = np.eye(4) * 100.0

        self.initialized = False

    def initialize(self, x_m, y_m):
        """Seed the filter with first known position."""
        self.x = np.array([[x_m], [y_m], [0.0], [0.0]])
        self.initialized = True

    def predict(self):
        """Step 1: Project state forward one tick."""
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x

    def update(self, x_m, y_m):
        """Step 2: Correct prediction with new noisy measurement."""
        z = np.array([[x_m], [y_m]])
        y = z - self.H @ self.x                          # innovation
        S = self.H @ self.P @ self.H.T + self.R          # innovation covariance
        K = self.P @ self.H.T @ np.linalg.inv(S)         # Kalman gain
        self.x = self.x + K @ y                          # corrected state
        self.P = (np.eye(4) - K @ self.H) @ self.P       # corrected covariance
        return self.x, self.P

    def step(self, x_m, y_m):
        """Predict then update. Returns state and covariance."""
        if not self.initialized:
            self.initialize(x_m, y_m)
            return self.x, self.P
        self.predict()
        return self.update(x_m, y_m)

    def project_future(self, steps):
        """
        Project position N steps ahead.
        Returns list of (x, y) positions and uncertainty radius at each step.
        """
        x_future = self.x.copy()
        P_future = self.P.copy()
        predictions = []

        for _ in range(steps):
            x_future = self.F @ x_future
            P_future = self.F @ P_future @ self.F.T + self.Q
            px = float(x_future[0, 0])
            py = float(x_future[1, 0])
            # Uncertainty radius: 2-sigma from position covariance
            sigma = math.sqrt(float(P_future[0, 0]) + float(P_future[1, 1]))
            predictions.append((px, py, 2 * sigma))

        return predictions
