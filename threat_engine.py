import time
import uuid
import threading
from collections import deque

from config import ZONES
from geofence import geofence_manager

class ThreatEngine:
    def __init__(self):
        self._lock = threading.Lock()
        self._alerts = deque(maxlen=200)

    @property
    def alerts(self):
        with self._lock:
            return list(self._alerts)

    def process_step(self, drones):
        with self._lock:
            for drone in drones:
                self._evaluate_drone(drone)

    def _evaluate_drone(self, drone):
        if not drone.is_confirmed:
            return # Ignore unconfirmed radar noise

        # The C++ core calculates the ellipsoid intersection AND handles spatial hysteresis
        breached_zones = geofence_manager.get_containing_zones(
            drone.id, 
            drone.easting, 
            drone.northing, 
            drone.alt, 
            drone.kf.P
        )

        for zone_id in breached_zones:
            # Generate the deterministic alerts. No temporal cooldowns required.
            self._add_alert(
                drone_id=drone.id,
                zone_breached=zone_id,
                threat_type="SPATIAL_BREACH",
                severity=ZONES[zone_id]["severity"],
                lat=drone.lat, 
                lon=drone.lon,
                speed=drone.kalman_speed, 
                alt=drone.alt
            )

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
        self._alerts.appendleft(alert)

threat_engine = ThreatEngine()
