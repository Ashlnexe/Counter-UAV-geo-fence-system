# =============================================================================
# geofence.py — Geometric zone management for Counter-UAV system
# =============================================================================
#
# Zones are modelled as circles in geographic space. Because 1° of longitude
# is shorter than 1° of latitude at non-equatorial latitudes, we must use
# separate conversion factors per axis — otherwise the "circles" become
# ellipses and boundary distances are wrong.
#
# We handle this by working in a locally-flat Cartesian frame (metres),
# building Shapely polygons there, then converting back to degrees for
# Plotly rendering. This gives correct geometry without requiring a full
# projected CRS dependency.
# =============================================================================

import math
import shapely.geometry

from config import BASE_LAT, BASE_LON, ZONES
import geo_utils


class GeofenceManager:
    """
    Manages concentric geo-fence zones and exposes spatial queries.
    All internal geometry and queries operate in UTM Cartesian meters.
    Converts back to WGS84 solely for UI rendering.
    """

    def __init__(self):
        # Ensure projection is initialized
        geo_utils.init_projection(BASE_LAT, BASE_LON)
        center_e, center_n = geo_utils.to_utm(BASE_LAT, BASE_LON)
        self.center_point = shapely.geometry.Point(center_e, center_n)
        
        self.zone_polygons: dict[str, shapely.geometry.Polygon] = {}
        self._init_zones()

    def _init_zones(self) -> None:
        """Build perfect circles in the Cartesian plane."""
        for zone_name, props in ZONES.items():
            radius_m = props["radius"]
            # buffer creates a circle in the Cartesian plane
            self.zone_polygons[zone_name] = self.center_point.buffer(radius_m, resolution=64)

    def get_containing_zones(self, easting: float, northing: float, radius_m: float = 0.0) -> list[str]:
        """
        Return zone names that contain the given position, accounting for uncertainty.
        Ordered from highest severity to lowest.
        """
        point = shapely.geometry.Point(easting, northing)
        if radius_m > 0:
            shape = point.buffer(radius_m)
            return [
                z for z in ["EXCLUSION", "BUFFER", "MONITORED"]
                if z in self.zone_polygons and self.zone_polygons[z].intersects(shape)
            ]
        else:
            return [
                z for z in ["EXCLUSION", "BUFFER", "MONITORED"]
                if z in self.zone_polygons and self.zone_polygons[z].contains(point)
            ]

    def get_distance_to_boundary_m(self, easting: float, northing: float, zone_name: str) -> float:
        """
        Return the exact Cartesian distance in metres from the point to a zone boundary.
        """
        point = shapely.geometry.Point(easting, northing)
        poly = self.zone_polygons[zone_name]
        return point.distance(poly.boundary)

    def get_closest_boundary(self, easting: float, northing: float, radius_m: float = 0.0) -> tuple[float, str]:
        """Return (min_distance_m, closest_zone_name), taking into account uncertainty radius."""
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
        """
        Generate Plotly Dash mapbox layer specs for all zones.
        Converts internal UTM Cartesian coordinates back to WGS84 for mapping.
        """
        layers = []
        for zone_name in ["MONITORED", "BUFFER", "EXCLUSION"]:
            props = ZONES[zone_name]
            poly = self.zone_polygons[zone_name]
            
            # Convert UTM boundary coordinates back to WGS84 (lon, lat) for Plotly GeoJSON
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
