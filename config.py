# Configuration parameters for Counter-UAV System

# Base location centered on Bengaluru
BASE_LAT = 12.97
BASE_LON = 77.59

# Approximate conversion factor: meters per degree of latitude/longitude
METERS_PER_DEGREE = 111000.0

# Zone definitions: Radius in meters
ZONES = {
    "EXCLUSION": {
        "radius": 1000.0,
        "severity": "CRITICAL",
        "color": "#ff2a2a",       # Vibrant Red
        "fill_color": "rgba(255, 42, 42, 0.15)"
    },
    "BUFFER": {
        "radius": 2000.0,
        "severity": "MEDIUM",
        "color": "#ff9f1a",       # Vibrant Orange
        "fill_color": "rgba(255, 159, 26, 0.1)"
    },
    "MONITORED": {
        "radius": 3000.0,
        "severity": "LOW",
        "color": "#fff200",       # Vibrant Yellow
        "fill_color": "rgba(255, 242, 0, 0.05)"
    }
}

# Threat & alert thresholds
LOITER_DISTANCE_THRESHOLD_M = 200.0  # meters from any zone boundary
LOITER_TIME_STEPS = 10               # steps (>10 steps triggers LOITERING alert)
HIGH_SPEED_THRESHOLD_MPS = 15.0      # meters per second
PERIMETER_TEST_LIMIT = 3             # 3+ approaches triggers PERIMETER TESTING

# Simulation update interval
SIMULATION_STEP_SECONDS = 0.5
