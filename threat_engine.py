import time
import uuid
import threading
import numpy as np
from collections import deque

from config import ZONES, ASSET_POLYGON
from geofence import geofence_manager
import geo_utils

def point_to_segment_dist(P, A, B):
    AB = B - A
    AP = P - A
    dot_product = np.dot(AP, AB)
    length_sq = np.dot(AB, AB)
    if length_sq < 1e-6:
        return np.linalg.norm(P - A)
    t = max(0.0, min(1.0, dot_product / length_sq))
    closest = A + t * AB
    return np.linalg.norm(P - closest)

def calculate_cpa(p_drone, v_drone, p_target):
    """Calculates Time-to-CPA and Distance-at-CPA to a specific point."""
    p_rel = p_drone - p_target
    v_mag_sq = np.dot(v_drone, v_drone)
    
    if v_mag_sq < 1e-6: 
        return 0.0, np.linalg.norm(p_rel)
        
    t_cpa = -np.dot(p_rel, v_drone) / v_mag_sq
    
    if t_cpa < 0:
        return 0.0, np.linalg.norm(p_rel)
        
    d_cpa = np.linalg.norm(p_rel + v_drone * t_cpa)
    return t_cpa, d_cpa

def calculate_segment_cpa(p_drone, v_drone, A, B):
    """Calculates unrestricted CPA between ray and infinite line AB, clamped to segment."""
    AB = B - A
    v_mag_sq = np.dot(v_drone, v_drone)
    
    if v_mag_sq < 1e-6:
        return 0.0, point_to_segment_dist(p_drone, A, B)
        
    # Check intersection with infinite line AB
    # n is normal to AB
    n = np.array([-AB[1], AB[0]])
    n_dot_v = np.dot(n, v_drone)
    
    candidates = []
    
    if abs(n_dot_v) > 1e-6:
        # Not parallel, find intersection with infinite line
        t_int = -np.dot(n, p_drone - A) / n_dot_v
        if t_int >= 0:
            P_int = p_drone + v_drone * t_int
            # Check if intersection is between A and B
            length_sq = np.dot(AB, AB)
            dot_product = np.dot(P_int - A, AB)
            if 0 <= dot_product <= length_sq:
                # Direct intersection with segment
                return t_int, 0.0
                
    # If no valid intersection, CPA to segment must be at t=0 or at the vertices.
    candidates.append((0.0, point_to_segment_dist(p_drone, A, B)))
    candidates.append(calculate_cpa(p_drone, v_drone, A))
    candidates.append(calculate_cpa(p_drone, v_drone, B))
    
    # Return the candidate with the smallest distance. 
    # If tied (e.g. parallel flight), earliest time is preferred.
    candidates.sort(key=lambda x: (round(x[1], 3), x[0]))
    return candidates[0]

def calculate_polygon_cpa(p_drone, v_drone, polygon):
    """Evaluates the CPA against all edges of a distributed perimeter polygon."""
    min_t = 0.0
    min_d = float('inf')
    
    for i in range(len(polygon)):
        A = polygon[i]
        B = polygon[(i + 1) % len(polygon)]
        
        t_cpa, d_cpa = calculate_segment_cpa(p_drone, v_drone, A, B)
        
        # We want the absolute minimum distance.
        # If there are multiple edges with the same minimum distance (e.g. multiple intersections d=0),
        # we care about the FIRST one that happens (smallest t).
        if d_cpa < min_d or (np.isclose(d_cpa, min_d) and t_cpa < min_t):
            min_d = d_cpa
            min_t = t_cpa
            
    return min_t, min_d

class ThreatEngine:
    def __init__(self):
        self._lock = threading.Lock()
        self._alerts = deque(maxlen=200)
        
        # Pre-compute the ASSET_POLYGON in UTM coordinates
        self.asset_polygon_utm = []
        for lat, lon in ASSET_POLYGON:
            e, n = geo_utils.to_utm(lat, lon)
            self.asset_polygon_utm.append(np.array([e, n]))

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
            return

        # Stage 1: Spatial Filtering (Is it within the 3km MONITORED zone?)
        breached_zones = geofence_manager.get_containing_zones(
            drone.id, 
            drone.easting, 
            drone.northing, 
            drone.alt, 
            drone.kf.P
        )
        
        if "MONITORED" not in breached_zones and "BUFFER" not in breached_zones and "EXCLUSION" not in breached_zones:
            return # Too far away, don't waste CPU on Kinematic math
            
        # Stage 2: Kinematic CPA against distributed perimeter
        p_drone = np.array([drone.easting, drone.northing])
        v_drone = np.array([float(drone.imm.x_out[2]), float(drone.imm.x_out[3])])
        
        t_cpa, d_cpa = calculate_polygon_cpa(p_drone, v_drone, self.asset_polygon_utm)
        speed = np.linalg.norm(v_drone)
        
        threat_type = "UNKNOWN"
        severity = "LOW"
        
        if t_cpa <= 15.0 and d_cpa <= 50.0:
            threat_type = "KINEMATIC_CPA"
            severity = "CRITICAL"
        elif t_cpa <= 30.0 and d_cpa <= 150.0:
            threat_type = "KINEMATIC_CPA"
            severity = "HIGH"
        elif speed < 2.0 and ("BUFFER" in breached_zones or "EXCLUSION" in breached_zones):
            threat_type = "LOITERING"
            severity = "MEDIUM"
        else:
            threat_type = "MONITORING"
            severity = "LOW"

        # Only emit alert if severity is Medium or higher to prevent flooding
        if severity != "LOW":
            self._add_alert(
                drone_id=drone.id,
                zone_breached=f"CPA (d={d_cpa:.1f}m, t={t_cpa:.1f}s)",
                threat_type=threat_type,
                severity=severity,
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
