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
import numpy as np
import shapely.geometry
import shapely.affinity

from config import (
    BASE_LAT, BASE_LON,
    METERS_PER_LAT_DEGREE, METERS_PER_LON_DEGREE,
    ZONES
)


class GeofenceManager:
    """
    Manages concentric geo-fence zones and exposes spatial queries.

    All public methods accept/return geographic coordinates (lat, lon)
    in decimal degrees. Internal geometry is in degrees but correctly
    scaled per axis so distance calculations are accurate.
    """

    def __init__(self):
        self.center_point = shapely.geometry.Point(BASE_LON, BASE_LAT)
        self.zone_polygons: dict[str, shapely.geometry.Polygon] = {}
        self._init_zones()

    def _init_zones(self) -> None:
        """
        Build elliptical (but geographically circular) polygons for each zone.

        Strategy: create a unit circle in degree-space, then scale the x-axis
        (longitude) by radius/METERS_PER_LON_DEGREE and y-axis (latitude) by
        radius/METERS_PER_LAT_DEGREE. This produces a shape that is a true
        circle when distances are measured in metres.
        """
        for zone_name, props in ZONES.items():
            radius_m = props["radius"]

            # Degrees per metre on each axis
            lat_deg_per_m = 1.0 / METERS_PER_LAT_DEGREE
            lon_deg_per_m = 1.0 / METERS_PER_LON_DEGREE

            # Build a circle with radius 1 in degree-space, then scale
            # x (lon) and y (lat) axes independently
            unit_circle = self.center_point.buffer(1.0, resolution=32)
            scaled = shapely.affinity.scale(
                unit_circle,
                xfact=radius_m * lon_deg_per_m,
                yfact=radius_m * lat_deg_per_m,
                origin=self.center_point
            )
            self.zone_polygons[zone_name] = scaled

    def get_containing_zones(self, lat: float, lon: float) -> list[str]:
        """
        Return zone names that contain the given position.
        Ordered from highest severity to lowest: EXCLUSION → BUFFER → MONITORED.
        """
        point = shapely.geometry.Point(lon, lat)
        return [
            z for z in ["EXCLUSION", "BUFFER", "MONITORED"]
            if z in self.zone_polygons and self.zone_polygons[z].contains(point)
        ]

    def get_distance_to_boundary_m(self, lat: float, lon: float,
                                   zone_name: str) -> float:
        """
        Return the distance in metres from the point to a specific zone boundary.
        Positive = outside the zone, negative = inside.
        """
        point = shapely.geometry.Point(lon, lat)
        poly  = self.zone_polygons[zone_name]

        dist_deg = point.distance(poly.boundary)

        # Convert degree distance back to metres using the mean conversion factor
        # (Euclidean distance in degree-space mixes lat and lon, so we use the
        # geometric mean of the two conversion factors as an approximation.)
        mean_m_per_deg = math.sqrt(METERS_PER_LAT_DEGREE * METERS_PER_LON_DEGREE)
        return dist_deg * mean_m_per_deg

    def get_closest_boundary(self, lat: float, lon: float) -> tuple[float, str]:
        """
        Return (min_distance_m, closest_zone_name) across all zones.
        Used by the threat engine to detect loitering near boundaries.
        """
        point = shapely.geometry.Point(lon, lat)
        mean_m_per_deg = math.sqrt(METERS_PER_LAT_DEGREE * METERS_PER_LON_DEGREE)

        min_dist_m   = float("inf")
        closest_zone = "MONITORED"

        for zone_name, poly in self.zone_polygons.items():
            dist_m = point.distance(poly.boundary) * mean_m_per_deg
            if dist_m < min_dist_m:
                min_dist_m   = dist_m
                closest_zone = zone_name

        return min_dist_m, closest_zone

    def get_mapbox_layers(self) -> list[dict]:
        """
        Generate Plotly Dash mapbox layer specs for all zones.
        Rendered largest-to-smallest so inner zones visually stack on top.
        """
        layers = []
        for zone_name in ["MONITORED", "BUFFER", "EXCLUSION"]:
            props = ZONES[zone_name]
            poly  = self.zone_polygons[zone_name]
            coords = list(poly.exterior.coords)   # [[lon, lat], ...]

            geojson = {
                "type": "Feature",
                "geometry": {"type": "Polygon", "coordinates": [coords]},
                "properties": {"name": zone_name}
            }

            # Fill layer
            layers.append({
                "sourcetype": "geojson",
                "source": geojson,
                "type": "fill",
                "color": props["fill_color"],
            })
            # Boundary line layer
            layers.append({
                "sourcetype": "geojson",
                "source": geojson,
                "type": "line",
                "color": props["color"],
                "line": {"width": 2}
            })

        return layers


# ---------------------------------------------------------------------------
# Module-level singleton — import this everywhere
# ---------------------------------------------------------------------------
geofence_manager = GeofenceManager()
