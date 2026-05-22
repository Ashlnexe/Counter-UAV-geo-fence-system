import time
import requests
import math
import logging
from typing import Generator

from adapters.base_adapter import SensorAdapter
from data_ingestion import TelemetryFrame
from config import BASE_LAT, BASE_LON

logger = logging.getLogger(__name__)

class OpenSkyAdapter(SensorAdapter):
    """
    Adapter that polls the live OpenSky Network API.
    Demonstrates the Kalman Filter's coasting logic by pulling data
    every 10 seconds (the free-tier rate limit).
    """
    def __init__(self, poll_interval: int = 10):
        super().__init__()
        self.poll_interval = poll_interval
        
        # Calculate ~10km x 10km bounding box around base
        lat_offset = 0.045  # ~5km
        lon_offset = 0.045 / math.cos(math.radians(BASE_LAT))
        
        self.lamin = BASE_LAT - lat_offset
        self.lamax = BASE_LAT + lat_offset
        self.lomin = BASE_LON - lon_offset
        self.lomax = BASE_LON + lon_offset
        
        self.url = f"https://opensky-network.org/api/states/all?lamin={self.lamin}&lomin={self.lomin}&lamax={self.lamax}&lomax={self.lomax}"

    def connect(self) -> bool:
        try:
            r = requests.get(self.url, timeout=5)
            r.raise_for_status()
            self._is_connected = True
            return True
        except Exception as e:
            logger.error(f"OpenSky connection failed: {e}")
            return False

    def disconnect(self) -> None:
        pass

    def get_stream(self) -> Generator[TelemetryFrame, None, None]:
        while self.is_connected():
            try:
                r = requests.get(self.url, timeout=5)
                r.raise_for_status()
                data = r.json()
                
                if data and data.get("states"):
                    for state in data["states"]:
                        icao24 = state[0]
                        lon = state[5]
                        lat = state[6]
                        alt = state[13] or state[7] or 0.0
                        velocity = state[9] or 0.0
                        heading = state[10] or 0.0
                        
                        if lat is None or lon is None:
                            continue
                            
                        yield TelemetryFrame(
                            drone_id=f"OS-{icao24}",
                            timestamp=time.time(),
                            lat=lat,
                            lon=lon,
                            alt=alt,
                            speed=velocity,
                            heading=heading,
                            source="opensky"
                        )
            except Exception as e:
                logger.warning(f"OpenSky poll failed: {e}")
                
            time.sleep(self.poll_interval)
