import sys
import os
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), ".")))

from counter_uav_core import KalmanFilter6D, GeofenceEngine

def run_stress_test():
    engine = GeofenceEngine()
    engine.add_zone("STRESS_ZONE", 0.0, 0.0, 100.0, 0.0, 1000.0)

    kf = KalmanFilter6D(init_easting=0.0, init_northing=0.0, init_alt=50.0)
    
    # Let's just do a single NaN check
    z_meas = np.nan
    kf.step(0.0, 0.0, z_meas, dt=1.0, noise_std_m=0.0)
    
    print("P matrix:")
    print(kf.P)
    
    breaches = engine.check_breaches("TEST_DRONE", 0.0, 0.0, 50.0, kf.P)
    print("Breaches:", breaches)

if __name__ == "__main__":
    run_stress_test()
