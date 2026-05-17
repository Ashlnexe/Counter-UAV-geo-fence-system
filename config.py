# =============================================================================
# config.py — System-wide configuration for Counter-UAV Geo-Fence Monitor
# =============================================================================
# All tuneable constants live here. Nothing else should define magic numbers.
# =============================================================================

import math

# -----------------------------------------------------------------------------
# Base location — Bengaluru city centre
# -----------------------------------------------------------------------------
BASE_LAT = 12.97   # degrees North
BASE_LON = 77.59   # degrees East

# -----------------------------------------------------------------------------
# Coordinate conversion — NOT a single flat-earth constant.
#
# 1 degree of latitude is nearly constant globally (~110,570 m).
# 1 degree of longitude SHRINKS as you move away from the equator:
#     lon_m = 111,320 * cos(latitude)
#
# At 12.97°N:
#     lat → 110,570 m/deg
#     lon → 111,320 * cos(12.97°) ≈ 108,484 m/deg
#
# Using a single constant for both introduces ~2.7% positional error —
# enough to misplace a drone by 27m at the edge of a 1km exclusion zone.
# -----------------------------------------------------------------------------
METERS_PER_LAT_DEGREE = 110_570.0
METERS_PER_LON_DEGREE = 111_320.0 * math.cos(math.radians(BASE_LAT))  # ≈ 108,484


# -----------------------------------------------------------------------------
# Geo-fence zone definitions
# Each zone is a concentric circle around BASE_LAT / BASE_LON.
# -----------------------------------------------------------------------------
ZONES = {
    "EXCLUSION": {
        "radius": 1000.0,           # metres
        "severity": "CRITICAL",
        "color": "#ff2a2a",
        "fill_color": "rgba(255, 42, 42, 0.15)"
    },
    "BUFFER": {
        "radius": 2000.0,
        "severity": "MEDIUM",
        "color": "#ff9f1a",
        "fill_color": "rgba(255, 159, 26, 0.1)"
    },
    "MONITORED": {
        "radius": 3000.0,
        "severity": "LOW",
        "color": "#fff200",
        "fill_color": "rgba(255, 242, 0, 0.05)"
    }
}

# -----------------------------------------------------------------------------
# Threat classification thresholds
# -----------------------------------------------------------------------------
LOITER_DISTANCE_THRESHOLD_M = 200.0   # metres from any zone boundary
LOITER_TIME_STEPS           = 10      # consecutive steps before LOITERING fires
HIGH_SPEED_THRESHOLD_MPS    = 15.0    # m/s — above this → HIGH_SPEED_INTRUSION
PERIMETER_TEST_LIMIT        = 3       # zone re-entries before PERIMETER_TESTING fires

# -----------------------------------------------------------------------------
# Simulation parameters
# -----------------------------------------------------------------------------
SIMULATION_STEP_SECONDS = 0.5         # physics tick rate

# -----------------------------------------------------------------------------
# GPS noise model
#
# Real GPS receivers output position with zero-mean Gaussian noise.
# Consumer-grade modules (typical on small UAVs): σ ≈ 2–5 m CEP.
# We model 3 m standard deviation in both axes.
# The Kalman filter's job is to recover clean tracks from this noisy signal.
# -----------------------------------------------------------------------------
GPS_NOISE_STD_M = 3.0                 # metres, 1-sigma

# -----------------------------------------------------------------------------
# Altitude dynamics per behaviour type
# Real UAVs don't fly at a constant altitude — they climb, dive, and loiter
# at mission-specific heights. These parameters drive the altitude simulation.
# -----------------------------------------------------------------------------
ALTITUDE_PROFILES = {
    "NORMAL_ORBIT":    {"base": 120.0, "amplitude": 0.0,  "period": 1.0},   # steady cruise
    "LOITERING":       {"base":  80.0, "amplitude": 20.0, "period": 30.0},  # slow descent/ascent
    "HIGH_SPEED":      {"base": 200.0, "amplitude": 170.0,"period": 60.0},  # diving attack profile
    "PERIMETER_TEST":  {"base": 100.0, "amplitude": 40.0, "period": 20.0},  # probing oscillation
}
