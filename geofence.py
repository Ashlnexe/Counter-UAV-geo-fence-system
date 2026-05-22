# =============================================================================
# geofence.py — Geometric zone management for Counter-UAV system
# =============================================================================

import math
import shapely.geometry
from config import BASE_LAT, BASE_LON, ZONES
import geo_utils
import counter_uav_core

class GeofenceManager:
    """
    Manages concentric geo-fence zones and exposes spatial queries.
    Uses C++ GeofenceEngine for ultra-fast 3D ellipsoid-to-cylinder intersections.
    """
    def __init__(self):
        geo_utils.init_projection(BASE_LAT, BASE_LON)
        self.center_e, self.center_n = geo_utils.to_utm(BASE_LAT, BASE_LON)
        self.cpp_engine = counter_uav_core.GeofenceEngine()
        
        self.zone_polygons = {}
        self._init_zones()

    def _init_zones(self) -> None:
        """Build perfect circles in the Cartesian plane for Plotly, and load them into C++."""
        for zone_name, props in ZONES.items():
            radius_m = props["radius"]
            # Still keep shapely for Plotly mapping exterior generation
            center_point = shapely.geometry.Point(self.center_e, self.center_n)
            self.zone_polygons[zone_name] = center_point.buffer(radius_m, resolution=64)
            
            self.cpp_engine.add_zone(
                zone_name, 
                self.center_e, 
                self.center_n, 
                radius_m, 
                props["alt_floor"], 
                props["alt_ceiling"]
            )

    def get_containing_zones(self, drone_id: str, easting: float, northing: float, alt: float, P) -> list[str]:
        """
        Return zone names that contain the 3D uncertainty ellipsoid of the given position.
        Delegates completely to C++.
        """
        # Call C++ Geofence Engine directly
        return self.cpp_engine.check_breaches(drone_id, easting, northing, alt, P)

    def get_distance_to_boundary_m(self, easting: float, northing: float, zone_name: str) -> float:
        point = shapely.geometry.Point(easting, northing)
        poly = self.zone_polygons[zone_name]
        return point.distance(poly.boundary)

    def get_closest_boundary(self, easting: float, northing: float, radius_m: float = 0.0) -> tuple[float, str]:
        point = shapely.geometry.Point(easting, northing)
        shape = point.buffer(radius_m) if radius_m > 0 else point
        
        min_dist_m = float("inf")
        closest_zone = "MONITORED"

        for zone_name, poly in self.zone_polygons.items():
            dist_m = shape.distance(poly.boundary)
            if dist_m < min_dist_m:
                min_dist_m = dist_m
                closest_zone = zone_name

        return min_dist_m, closest_zone

    def get_mapbox_layers(self) -> list[dict]:
        layers = []
        for zone_name in ["MONITORED", "BUFFER", "EXCLUSION"]:
            props = ZONES[zone_name]
            poly = self.zone_polygons[zone_name]
            coords = []
            for e, n in poly.exterior.coords:
                lat, lon = geo_utils.to_wgs84(e, n)
                coords.append([lon, lat])

            geojson = {
                "type": "Feature",
                "geometry": {"type": "Polygon", "coordinates": [coords]},
                "properties": {"name": zone_name}
            }

            layers.append({
                "sourcetype": "geojson",
                "source": geojson,
                "type": "fill",
                "color": props["fill_color"],
            })
            layers.append({
                "sourcetype": "geojson",
                "source": geojson,
                "type": "line",
                "color": props["color"],
                "line": {"width": 2}
            })

        return layers

geofence_manager = GeofenceManager()
