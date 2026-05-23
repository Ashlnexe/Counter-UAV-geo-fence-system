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
    
    rng = np.random.default_rng(42)
    
    valid_count = 0
    invalid_count = 0

    for i in range(10000):
        # Malicious non-Gaussian noise in Z-axis
        z_noise = rng.standard_cauchy() * 10000.0
        
        # Inject extremely small noise std to force collapse, then large to expand
        noise_std = 1e-9 if i % 2 == 0 else 1e9
        
        kf.step(0.0, 0.0, 50.0 + z_noise, dt=1.0, noise_std_m=noise_std)
        
        breaches = engine.check_breaches(kf.x[0], kf.x[1], kf.x[2], kf.P)
        is_breach = len(breaches) > 0
        alerts.append(is_breach)
        
        # Check if eigensolver would fail by looking at P
        if np.isnan(kf.P).any() or np.isinf(kf.P).any():
            invalid_count += 1
            # Reset to prevent permanent NaN death
            kf = KalmanFilter6D(init_easting=0.0, init_northing=0.0, init_alt=50.0)
        else:
            try:
                np.linalg.eigh(kf.P[:2, :2])
                valid_count += 1
            except np.linalg.LinAlgError:
                invalid_count += 1

    flickers = sum(1 for i in range(1, len(alerts)) if alerts[i] != alerts[i-1])

    print(f"Total Steps: {len(alerts)}")
    print(f"Breaches detected: {sum(alerts)} / {len(alerts)}")
    print(f"Flickers (State Toggles): {flickers}")
    print(f"Valid matrix updates: {valid_count}")
    print(f"Invalid matrix updates: {invalid_count}")

if __name__ == "__main__":
    run_stress_test()
