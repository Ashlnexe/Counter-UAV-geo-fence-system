import pyproj

# Standard WGS84 Coordinate System
wgs84 = pyproj.CRS("EPSG:4326")

# We will use a dynamic UTM projection based on the first received coordinate,
# or we can hardcode a specific UTM zone if we know the operational area.
# For a globally applicable system without zone boundary issues, a local 
# Transverse Mercator centered on the BASE_LAT/BASE_LON is technically best, 
# but UTM is standard.
# 
# PyProj provides a convenient way to get the correct UTM CRS from a lat/lon:
# utm_crs = pyproj.CRS.from_dict({'proj': 'utm', 'zone': zone, 'datum': 'WGS84'})
# 
# But an easier robust method is `pyproj.Proj.from_proj_string` or using a Transformer
# directly with an automatically determined UTM zone.

_transformer_to_utm = None
_transformer_to_wgs84 = None

def init_projection(lat: float, lon: float) -> None:
    """
    Initialize the UTM projection based on the central operational coordinate.
    This guarantees accurate Cartesian meters for the area of operation.
    """
    global _transformer_to_utm, _transformer_to_wgs84
    
    # Calculate UTM Zone
    zone_number = int((lon + 180) / 6) + 1
    is_south = lat < 0
    
    # EPSG codes for UTM:
    # North: 32600 + zone
    # South: 32700 + zone
    epsg = (32700 if is_south else 32600) + zone_number
    utm_crs = pyproj.CRS(f"EPSG:{epsg}")
    
    _transformer_to_utm = pyproj.Transformer.from_crs(wgs84, utm_crs, always_xy=True)
    _transformer_to_wgs84 = pyproj.Transformer.from_crs(utm_crs, wgs84, always_xy=True)

def to_utm(lat: float, lon: float) -> tuple[float, float]:
    """Convert WGS84 (Lat, Lon) to UTM (Easting, Northing) in meters."""
    if not _transformer_to_utm:
        raise RuntimeError("UTM Projection not explicitly initialized. Call init_projection(BASE_LAT, BASE_LON) on startup to prevent teleportation flaws.")
    # Pyproj always_xy=True expects (lon, lat)
    easting, northing = _transformer_to_utm.transform(lon, lat)
    return easting, northing

def to_wgs84(easting: float, northing: float) -> tuple[float, float]:
    """Convert UTM (Easting, Northing) in meters back to WGS84 (Lat, Lon)."""
    if not _transformer_to_wgs84:
        raise RuntimeError("Projection not initialized. Call init_projection or to_utm first.")
    lon, lat = _transformer_to_wgs84.transform(easting, northing)
    return lat, lon
