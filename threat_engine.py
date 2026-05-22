# =============================================================================
# threat_engine.py — Threat classification and alert management
# =============================================================================
#
# Consumes a stream of Drone objects each simulation tick and classifies
# behaviour into four threat categories:
#
#   LOITERING          — drone orbiting near a zone boundary (surveillance)
#   HIGH_SPEED_INTRUSION — fast penetration of a protected zone
#   PERIMETER_TESTING  — repeated zone entries (probing defences)
#   UNAUTHORIZED_ENTRY — any other zone breach
#
# Thread safety
# -------------
# This module has its OWN lock. It must not rely on SimulationEngine's lock
# for protection — callers should be able to use ThreatEngine independently.
# =============================================================================

import time
import uuid
import threading
from collections import deque

from config import (
    ZONES,
    LOITER_DISTANCE_THRESHOLD_M,
    LOITER_TIME_STEPS,
    HIGH_SPEED_THRESHOLD_MPS,
    PERIMETER_TEST_LIMIT,
)
from geofence import geofence_manager

class ThreatEngine:
    """
    Stateful threat classifier. Maintains per-drone history to detect
    behavioural patterns that a single-frame snapshot would miss.
    """

    def __init__(self):
        self._lock   = threading.Lock()
        self._drone_states: dict[str, dict] = {}
        # deque gives O(1) prepend via appendleft + bounded size for free
        self._alerts: deque = deque(maxlen=200)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def alerts(self) -> list[dict]:
        """Thread-safe snapshot of current alert list (newest first)."""
        with self._lock:
            return list(self._alerts)

    def process_step(self, drones: list) -> None:
        """
        Evaluate each drone against geofence rules for this time step.

        Parameters
        ----------
        drones : list of Drone objects
        """
        with self._lock:
            for drone in drones:
                self._evaluate_drone(drone)

    # ------------------------------------------------------------------
    # Internal logic — all called while holding self._lock
    # ------------------------------------------------------------------

    def _get_state(self, drone_id: str) -> dict:
        if drone_id not in self._drone_states:
            self._drone_states[drone_id] = {
                "prev_zones":             set(),
                "loiter_start_time":      None,
                "loiter_alerted":         False,
                "entry_count":            0,
                "last_entry_alerted_count": 0,
                "kinematic_alert_cooldown": 0.0,
            }
        return self._drone_states[drone_id]

    def _evaluate_drone(self, drone) -> None:
        d_id  = drone.id
        # Use UTM Cartesian coordinates for spatial logic
        easting  = drone.easting
        northing = drone.northing
        
        # Use WGS84 for alerts and dashboard display
        lat   = drone.lat
        lon   = drone.lon
        
        speed = drone.kalman_speed
        alt   = drone.alt
        ts    = drone.last_ts

        state = self._get_state(d_id)
        # ---- 0. Deterministic Kinematic Rules (Replaces ML) ----------------
        MAX_KINEMATIC_SPEED_MPS = 30.0  # e.g., 108 km/h is highly suspicious
        if speed > MAX_KINEMATIC_SPEED_MPS and (ts - state["kinematic_alert_cooldown"] > 10.0):
            self._add_alert(
                drone_id=d_id,
                zone_breached="N/A",
                threat_type="KINEMATIC_ANOMALY",
                severity="HIGH",
                lat=lat, lon=lon,
                speed=speed, alt=alt,
            )
            state["kinematic_alert_cooldown"] = ts

        # Use the expanding covariance bound from the Kalman filter for pessimistic collision checking
        r_uncert = getattr(drone, 'uncertainty_radius_m', 0.0)
        curr_zones = set(geofence_manager.get_containing_zones(easting, northing, radius_m=r_uncert))

        # ---- 1. Loitering detection (Time-based) ---------------------------
        min_dist, closest_zone = geofence_manager.get_closest_boundary(easting, northing, radius_m=r_uncert)

        if min_dist <= LOITER_DISTANCE_THRESHOLD_M and "EXCLUSION" not in curr_zones:
            if state["loiter_start_time"] is None:
                state["loiter_start_time"] = ts
                
            elapsed = ts - state["loiter_start_time"]
            # config.py has LOITER_TIME_STEPS. Let's assume 1 step = 0.5s in the old system, 
            # so LOITER_TIME_STEPS * 0.5 = seconds. Or let's just use 5.0 seconds.
            loiter_threshold_sec = LOITER_TIME_STEPS * 0.5 
            
            if elapsed > loiter_threshold_sec and not state["loiter_alerted"]:
                self._add_alert(
                    drone_id=d_id,
                    zone_breached=f"{closest_zone} Perimeter",
                    threat_type="LOITERING",
                    severity="MEDIUM",
                    lat=lat, lon=lon,
                    speed=speed, alt=alt,
                )
                state["loiter_alerted"] = True
        else:
            if min_dist > LOITER_DISTANCE_THRESHOLD_M:
                state["loiter_start_time"] = None
                state["loiter_alerted"] = False

        # ---- 2. Zone-entry threat classification ---------------------------
        new_zones = curr_zones - state["prev_zones"]
        if new_zones:
            target_zone = next(
                (z for z in ["EXCLUSION", "BUFFER", "MONITORED"] if z in new_zones),
                None
            )
            if target_zone:
                severity = ZONES[target_zone]["severity"]

                # Altitude modifier: low-flying drones in EXCLUSION → escalate
                if target_zone == "EXCLUSION" and alt < 50.0:
                    severity = "CRITICAL"   # already CRITICAL, but explicit

                if not state["prev_zones"]:
                    state["entry_count"] += 1

                if speed >= HIGH_SPEED_THRESHOLD_MPS:
                    self._add_alert(
                        drone_id=d_id,
                        zone_breached=target_zone,
                        threat_type="HIGH_SPEED_INTRUSION",
                        severity=severity,
                        lat=lat, lon=lon,
                        speed=speed, alt=alt,
                    )
                elif (state["entry_count"] >= PERIMETER_TEST_LIMIT
                      and state["entry_count"] > state["last_entry_alerted_count"]):
                    self._add_alert(
                        drone_id=d_id,
                        zone_breached=target_zone,
                        threat_type="PERIMETER_TESTING",
                        severity=severity,
                        lat=lat, lon=lon,
                        speed=speed, alt=alt,
                    )
                    state["last_entry_alerted_count"] = state["entry_count"]
                else:
                    self._add_alert(
                        drone_id=d_id,
                        zone_breached=target_zone,
                        threat_type="UNAUTHORIZED_ENTRY",
                        severity=severity,
                        lat=lat, lon=lon,
                        speed=speed, alt=alt,
                    )

        state["prev_zones"] = curr_zones

    def _add_alert(self, drone_id: str, zone_breached: str, threat_type: str,
                   severity: str, lat: float, lon: float,
                   speed: float, alt: float) -> None:
        alert = {
            "alert_id":     str(uuid.uuid4())[:8],
            "drone_id":     drone_id,
            "timestamp":    time.strftime("%H:%M:%S"),
            "zone_breached": zone_breached,
            "threat_type":  threat_type,
            "severity":     severity,
            "lat":          round(lat, 5),
            "lon":          round(lon, 5),
            "speed":        round(speed, 1),
            "alt":          round(alt, 1),
        }
        self._alerts.appendleft(alert)   # O(1) — newest first


# ---------------------------------------------------------------------------
# Module-level singleton
# ---------------------------------------------------------------------------
threat_engine = ThreatEngine()
