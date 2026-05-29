import sys
import os
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), ".")))

from counter_uav_core import KalmanFilter6D, GeofenceEngine

def run_stress_test():
    engine = GeofenceEngine()
    engine.add_zone("STRESS_ZONE", 0.0, 0.0, 100.0, 0.0, 1000.0)

    kf = KalmanFilter6D(init_easting=0.0, init_northing=0.0, init_alt=50.0)

    alerts = []
    
    for i in range(10000):
        # Force matrix to become invalid periodically by injecting NaNs
        if i % 10 < 5:
            # Valid but huge noise to expand the matrix
            z_meas = 50.0 + np.random.randn() * 10.0
            noise_std = 1000.0
        else:
            # Invalid data (sensor drop / extreme non-Gaussian collapse)
            z_meas = np.nan
            noise_std = 0.0
            
        kf.step(0.0, 0.0, z_meas, dt=1.0, noise_std_m=noise_std)
        
        breaches = engine.check_breaches("TEST_DRONE", 0.0, 0.0, 50.0, kf.P)
        is_breach = len(breaches) > 0
        alerts.append(is_breach)
        
        # We must reset kf to prevent permanent NaN, to simulate "rapidly toggling between valid and invalid"
        # The python layer or Kalman filter might naturally reject NaNs or reset in a real system,
        # but here we just manually toggle it for the test.
        if np.isnan(kf.P).any():
            # It's invalid! It will return NO_BREACH in check_breaches!
            # Next loop we reset to valid
            kf = KalmanFilter6D(init_easting=0.0, init_northing=0.0, init_alt=50.0)

    flickers = sum(1 for i in range(1, len(alerts)) if alerts[i] != alerts[i-1])

    print(f"Total Steps: {len(alerts)}")
    print(f"Breaches detected: {sum(alerts)} / {len(alerts)}")
    print(f"Flickers (State Toggles): {flickers}")

if __name__ == "__main__":
    run_stress_test()
