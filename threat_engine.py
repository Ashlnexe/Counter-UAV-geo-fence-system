import time
import uuid
from config import (
    ZONES,
    LOITER_DISTANCE_THRESHOLD_M,
    LOITER_TIME_STEPS,
    HIGH_SPEED_THRESHOLD_MPS,
    PERIMETER_TEST_LIMIT
)
from geofence import geofence_manager

class ThreatEngine:
    def __init__(self):
        self.drone_states = {}
        self.alerts = []

    def _get_drone_state(self, drone_id):
        if drone_id not in self.drone_states:
            self.drone_states[drone_id] = {
                "prev_zones": set(),
                "loiter_steps": 0,
                "loiter_alerted": False,
                "entry_count": 0,
                "last_entry_alerted_count": 0,
                "entered_zones_history": set()
            }
        return self.drone_states[drone_id]

    def process_step(self, drones):
        """
        Evaluate current drone positions/speeds against geofence rules.
        drones: list of dicts or objects containing:
            id, lat, lon, speed
        """
        for drone in drones:
            # Handle both dict and object access safely
            d_id = drone["id"] if isinstance(drone, dict) else drone.id
            lat = drone["lat"] if isinstance(drone, dict) else drone.lat
            lon = drone["lon"] if isinstance(drone, dict) else drone.lon
            speed = drone["speed"] if isinstance(drone, dict) else drone.speed

            state = self._get_drone_state(d_id)
            curr_zones = set(geofence_manager.get_containing_zones(lat, lon))
            
            # 1. Evaluate Loitering Behavior
            # If drone is close to a boundary but not inside the inner critical zones
            min_dist, closest_zone = geofence_manager.get_closest_boundary(lat, lon)
            
            # Loitering condition: within distance threshold and outside EXCLUSION/BUFFER
            # or just generally staying near any boundary
            if min_dist <= LOITER_DISTANCE_THRESHOLD_M and "EXCLUSION" not in curr_zones:
                state["loiter_steps"] += 1
                if state["loiter_steps"] > LOITER_TIME_STEPS and not state["loiter_alerted"]:
                    self._add_alert(
                        drone_id=d_id,
                        zone_breached=f"{closest_zone} Perimeter",
                        threat_type="LOITERING",
                        severity="MEDIUM",
                        lat=lat,
                        lon=lon,
                        speed=speed
                    )
                    state["loiter_alerted"] = True
            else:
                # Reset loitering counter if it moves away
                if min_dist > LOITER_DISTANCE_THRESHOLD_M:
                    state["loiter_steps"] = 0
                    state["loiter_alerted"] = False

            # Detect new zone entries
            new_zones = curr_zones - state["prev_zones"]
            
            if new_zones:
                # Determine highest severity zone entered
                # Order of severity check: EXCLUSION, BUFFER, MONITORED
                target_zone = None
                for z in ["EXCLUSION", "BUFFER", "MONITORED"]:
                    if z in new_zones:
                        target_zone = z
                        break
                
                if target_zone:
                    severity = ZONES[target_zone]["severity"]
                    
                    # Check if this is a fresh transition from completely outside to inside
                    if not state["prev_zones"]:
                        state["entry_count"] += 1

                    # Threat classification priority:
                    # A. High Speed Intrusion
                    if speed >= HIGH_SPEED_THRESHOLD_MPS:
                        self._add_alert(
                            drone_id=d_id,
                            zone_breached=target_zone,
                            threat_type="HIGH_SPEED_INTRUSION",
                            severity=severity,
                            lat=lat,
                            lon=lon,
                            speed=speed
                        )
                    # B. Perimeter Testing (>= 3 approaches)
                    elif state["entry_count"] >= PERIMETER_TEST_LIMIT and state["entry_count"] > state["last_entry_alerted_count"]:
                        self._add_alert(
                            drone_id=d_id,
                            zone_breached=target_zone,
                            threat_type="PERIMETER_TESTING",
                            severity=severity,
                            lat=lat,
                            lon=lon,
                            speed=speed
                        )
                        state["last_entry_alerted_count"] = state["entry_count"]
                    # C. Standard Unauthorized Entry
                    else:
                        self._add_alert(
                            drone_id=d_id,
                            zone_breached=target_zone,
                            threat_type="UNAUTHORIZED_ENTRY",
                            severity=severity,
                            lat=lat,
                            lon=lon,
                            speed=speed
                        )

            state["prev_zones"] = curr_zones

    def _add_alert(self, drone_id, zone_breached, threat_type, severity, lat, lon, speed):
        alert = {
            "alert_id": str(uuid.uuid4())[:8],
            "drone_id": drone_id,
            "timestamp": time.strftime("%H:%M:%S"),
            "zone_breached": zone_breached,
            "threat_type": threat_type,
            "severity": severity,
            "lat": round(lat, 5),
            "lon": round(lon, 5),
            "speed": round(speed, 1)
        }
        # Insert at the beginning so latest alerts are first
        self.alerts.insert(0, alert)
        # Keep alert history bounded to ensure performance
        if len(self.alerts) > 200:
            self.alerts.pop()

# Global singleton instance
threat_engine = ThreatEngine()
