# Counter-UAV Geo-Fence Monitoring System

A real-time aerial threat detection and tracking simulation built in Python.  
Simulates GPS-noisy UAV tracks, applies a **Kalman filter** for clean position estimation, classifies intrusion behaviour across concentric geo-fence zones, and renders a live tactical dashboard.

![Python](https://img.shields.io/badge/Python-3.11+-blue?style=flat-square)
![Dash](https://img.shields.io/badge/Dash-Plotly-informational?style=flat-square)
![License](https://img.shields.io/badge/License-MIT-green?style=flat-square)

---

## Demo

| Feature | Detail |
|---|---|
| Tracked UAVs | 4 simultaneous targets with distinct behaviour profiles |
| Geo-fence zones | MONITORED → BUFFER → EXCLUSION (concentric, 3km / 2km / 1km) |
| Threat categories | LOITERING, HIGH_SPEED_INTRUSION, PERIMETER_TESTING, UNAUTHORIZED_ENTRY |
| Map | Live Mapbox tactical view with raw GPS + Kalman-filtered dual tracks |
| Update rate | 1 Hz dashboard refresh, 2 Hz physics simulation |

---

## Architecture

```
┌─────────────┐     polar kinematics      ┌──────────────────┐
│  config.py  │ ─────────────────────────▶│  simulation.py   │
│             │                           │                  │
│ Zone radii  │    GPS noise injection     │  Drone objects   │
│ Thresholds  │    + Kalman filter tick   │  Physics loop    │
│ Noise model │                           │  (daemon thread) │
│ Alt profiles│                           └────────┬─────────┘
└─────────────┘                                    │
                                                   │ Drone list (filtered pos)
      ┌────────────────┐                           ▼
      │  geofence.py   │◀────────────────── ┌──────────────────┐
      │                │  zone containment  │ threat_engine.py │
      │ Shapely polys  │  boundary distance │                  │
      │ Correct lat/lon│                    │ Per-drone state  │
      │ degree scaling │                    │ Alert deque      │
      └────────────────┘                    │ Own mutex lock   │
                                            └────────┬─────────┘
      ┌────────────────┐                             │
      │   kalman.py    │                             │ alerts[]
      │                │              ┌──────────────▼──────────┐
      │ 4-state filter │              │        app.py           │
      │ Predict+Update │              │                         │
      │ x=[lat,lon,    │              │  Dash callbacks         │
      │   vlat,vlon]   │              │  Map: raw + filtered    │
      └────────────────┘              │  Telemetry: alt, speed  │
                                      │  Alert feed             │
                                      └─────────────────────────┘
```

---

## Key Design Decisions

### 1. Kalman Filter for Position Estimation

Real GPS receivers inject zero-mean Gaussian noise (~3m CEP on consumer modules).  
Naively plotting raw positions produces jittery tracks — unusable near zone boundaries.

A 4-state constant-velocity Kalman filter (`kalman.py`) estimates the true position by fusing:
- **Prediction** — where physics says the drone *should* be (dead-reckoning)
- **Update** — where GPS *says* it is (noisy measurement)

The Kalman Gain balances these two signals optimally in the least-squares sense.  
Speed fed into the threat engine is derived from the Kalman velocity state — more stable than differencing noisy GPS positions.

The dashboard renders both raw GPS (faint white) and filtered track (vibrant colour) simultaneously — making the filter's effect visually obvious.

### 2. Correct Coordinate Geometry

A single `METERS_PER_DEGREE` constant is a flat-earth approximation that breaks at non-equatorial latitudes:

```
At 12.97°N (Bengaluru):
  1° latitude  ≈ 110,570 m
  1° longitude ≈ 111,320 × cos(12.97°) ≈ 108,484 m   ← ~2% shorter
```

Using one constant for both axes turns circles into ellipses and misplaces boundaries by ~27m at 1km range — enough to generate false alerts or miss real ones.

`geofence.py` uses separate `METERS_PER_LAT_DEGREE` and `METERS_PER_LON_DEGREE` constants and applies them via `shapely.affinity.scale` to produce geometrically correct circular zones.

### 3. Thread Safety

The simulation runs in a daemon thread (`threading.Thread`). The Dash callback runs in a separate thread on every UI interval tick.

- `SimulationEngine` owns a lock that guards all drone state writes **and** the `process_step` call into `ThreatEngine`.
- `ThreatEngine` owns its **own** independent lock, so it is safe to use standalone without relying on the caller's lock.
- `threat_engine.alerts` is a `collections.deque(maxlen=200)` — `appendleft` for O(1) prepend, bounded size, no manual pop logic.

### 4. Altitude as a Real Variable

Each behaviour profile defines altitude dynamics:

| Profile | Base alt | Behaviour |
|---|---|---|
| NORMAL_ORBIT | 120 m | Steady cruise |
| LOITERING | 80 m | Slow oscillation (descent to survey) |
| HIGH_SPEED | 200 m | Diving attack profile → ~30 m at breach |
| PERIMETER_TEST | 100 m | Probing oscillation |

Altitude is surfaced in the threat alert payload and the telemetry table. Low-altitude EXCLUSION breaches are explicitly flagged at CRITICAL severity regardless of speed.

---

## Threat Classification Logic

```
Every simulation tick:
│
├─ For each drone:
│   │
│   ├─ Compute Kalman-filtered position and speed
│   ├─ Query geofence for containing zones
│   │
│   ├─ LOITERING check
│   │     If within 200m of any boundary for >10 consecutive ticks
│   │     → Alert: MEDIUM severity
│   │
│   └─ Zone-entry check (only fires on transition, not while inside)
│         │
│         ├─ speed ≥ 15 m/s → HIGH_SPEED_INTRUSION
│         ├─ entry_count ≥ 3 → PERIMETER_TESTING
│         └─ otherwise → UNAUTHORIZED_ENTRY
│
│         Severity from zone definition (CRITICAL / MEDIUM / LOW)
│         Override to CRITICAL if EXCLUSION + alt < 50m
```

---

## Simulated UAV Profiles

| ID | Callsign | Behaviour | Speed | Threat Pattern |
|---|---|---|---|---|
| UAV-01 | Alpha Scout | NORMAL_ORBIT | 12 m/s | Orbits at 3600m — benign reference |
| UAV-02 | Shadow Loiterer | LOITERING | 8→5 m/s | Approaches to 2100m, loiters near BUFFER |
| UAV-03 | Striker Intruder | HIGH_SPEED | 22 m/s | Straight-line dive into EXCLUSION, resets |
| UAV-04 | Phantom Tester | PERIMETER_TEST | 14 m/s | Sine-wave crossing of MONITORED boundary |

---

## Setup

```bash
git clone https://github.com/Ashlnexe/Counter-UAV-geo-fence-system.git
cd Counter-UAV-geo-fence-system

pip install -r requirements.txt
python app.py
```

Open `http://localhost:8050` in your browser.

---

## Requirements

```
dash
plotly
shapely
numpy
```

---

## File Structure

```
.
├── app.py            # Dash dashboard and callbacks
├── simulation.py     # UAV physics engine (threaded)
├── kalman.py         # 2D Kalman filter for position estimation
├── threat_engine.py  # Behavioural threat classification
├── geofence.py       # Geo-fence zone geometry and spatial queries
├── config.py         # All system constants (single source of truth)
└── requirements.txt
```

---

## Limitations & Future Work

- **No real sensor input** — positions are simulated, not from actual RF/radar sensors
- **2D Kalman filter** — altitude is simulated independently; a full 3D filter would couple all axes
- **Circular zones only** — real operational geofences use irregular polygons (terrain-aware)
- **No counter-measure output** — detection only; jamming/interception response not modelled

---

## Author

**Ashln** — B.Tech Computer Science, KTU  
GitHub: [@Ashlnexe](https://github.com/Ashlnexe)
