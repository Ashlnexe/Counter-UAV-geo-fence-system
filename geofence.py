import shapely.geometry
from config import BASE_LAT, BASE_LON, METERS_PER_DEGREE, ZONES

class GeofenceManager:
    def __init__(self):
        self.center_point = shapely.geometry.Point(BASE_LON, BASE_LAT)
        self.zone_polygons = {}
        self._init_zones()

    def _init_zones(self):
        # Create regular buffered polygons to simulate circular geo-fence zones
        # Order from largest to smallest or vice versa
        for zone_name, props in ZONES.items():
            radius_deg = props["radius"] / METERS_PER_DEGREE
            # resolution=16 gives a smooth 64-vertex polygon
            poly = self.center_point.buffer(radius_deg, resolution=16)
            self.zone_polygons[zone_name] = poly

    def get_containing_zones(self, lat, lon):
        """
        Returns a list of zone names that contain the given (lat, lon).
        Ordered from highest severity (EXCLUSION) to lowest (MONITORED).
        """
        point = shapely.geometry.Point(lon, lat)
        contained = []
        # Check EXCLUSION, BUFFER, MONITORED in order of severity
        for zone_name in ["EXCLUSION", "BUFFER", "MONITORED"]:
            if zone_name in self.zone_polygons and self.zone_polygons[zone_name].contains(point):
                contained.append(zone_name)
        return contained

    def get_min_distance_to_any_boundary(self, lat, lon):
        """
        Returns the minimum distance in meters from the point to any zone boundary.
        Useful for detecting LOITERING behavior near boundaries.
        """
        point = shapely.geometry.Point(lon, lat)
        min_dist_m = float('inf')
        for poly in self.zone_polygons.values():
            # Distance to the perimeter (boundary) in degrees
            dist_deg = point.distance(poly.boundary)
            dist_m = dist_deg * METERS_PER_DEGREE
            if dist_m < min_dist_m:
                min_dist_m = dist_m
        return min_dist_m

    def get_closest_boundary(self, lat, lon):
        """
        Returns (min_dist_m, closest_zone_name) from the point to any zone boundary.
        Useful for highly detailed LOITERING alert messages.
        """
        point = shapely.geometry.Point(lon, lat)
        min_dist_m = float('inf')
        closest_zone = "MONITORED"
        for zone_name, poly in self.zone_polygons.items():
            dist_deg = point.distance(poly.boundary)
            dist_m = dist_deg * METERS_PER_DEGREE
            if dist_m < min_dist_m:
                min_dist_m = dist_m
                closest_zone = zone_name
        return min_dist_m, closest_zone

    def get_mapbox_layers(self):
        """
        Generates a list of mapbox layout layers for Plotly Dash.
        Draws from largest (MONITORED) to smallest (EXCLUSION) so smaller inner zones
        render nicely on top of larger outer zones.
        """
        layers = []
        # Order: MONITORED -> BUFFER -> EXCLUSION for proper visual stacking
        for zone_name in ["MONITORED", "BUFFER", "EXCLUSION"]:
            props = ZONES[zone_name]
            poly = self.zone_polygons[zone_name]
            
            # Extract exterior coordinates as a list of [lon, lat]
            coords = list(poly.exterior.coords)
            
            geojson_data = {
                "type": "Feature",
                "geometry": {
                    "type": "Polygon",
                    "coordinates": [coords]
                },
                "properties": {
                    "name": zone_name
                }
            }
            
            layers.append({
                "sourcetype": "geojson",
                "source": geojson_data,
                "type": "fill",
                "color": props["fill_color"],
            })
            
            # Add boundary line layer for premium distinct outline
            layers.append({
                "sourcetype": "geojson",
                "source": geojson_data,
                "type": "line",
                "color": props["color"],
                "line": {"width": 2}
            })
            
        return layers

# Global singleton instance for easy imports
geofence_manager = GeofenceManager()
