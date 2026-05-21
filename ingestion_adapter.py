# =============================================================================
# ingestion_adapter.py — Adapts real GPS log readers to the dashboard interface
# =============================================================================
#
# Wraps a CSVGPSReader or MAVLinkReader so that it exposes the same
# get_state() API as SimulationEngine.  This allows app.py to swap between
# live simulation and recorded-data replay with zero changes to the
# dashboard callbacks.
#
# The adapter runs a background thread that pulls TelemetryFrames from
# the reader and feeds them through a per-drone Kalman filter and the
# threat engine — exactly mirroring what the simulation path does.
# =============================================================================

from __future__ import annotations

import threading
import time
import logging
from collections import deque
from typing import Optional

from config import (
    BASE_LAT, BASE_LON,
    METERS_PER_LAT_DEGREE, METERS_PER_LON_DEGREE,
)
from kalman import KalmanFilter2D
from threat_engine import threat_engine
from data_ingestion import CSVGPSReader, MAVLinkReader, TelemetryFrame

logger = logging.getLogger(__name__)


class _TrackedDrone:
    """Internal bookkeeping for a single real drone being replayed."""

    def __init__(self, drone_id: str):
        self.id            = drone_id
        self.name          = drone_id
        self.behavior_type = "REAL_DATA"
        self.lat           = 0.0
        self.lon           = 0.0
        self.true_lat      = 0.0
        self.true_lon      = 0.0
        self.alt           = 0.0
        self.speed         = 0.0
        self.heading       = 0.0
        self.kf: Optional[KalmanFilter2D] = None
        self.filtered_path: deque = deque(maxlen=60)
        self.true_path:     deque = deque(maxlen=60)

    @property
    def kalman_speed(self) -> float:
        """Match the Drone.kalman_speed interface used by threat_engine."""
        if self.kf is not None:
            return self.kf.estimated_speed_mps
        return self.speed

    def update(self, frame: TelemetryFrame) -> None:
        """Push a new telemetry frame through the Kalman filter."""
        self.true_lat = frame.lat
        self.true_lon = frame.lon
        self.true_path.append((frame.lat, frame.lon))

        # Lazy-init Kalman filter on first fix
        if self.kf is None:
            self.kf = KalmanFilter2D(
                init_lat=frame.lat,
                init_lon=frame.lon,
                meters_per_lat=METERS_PER_LAT_DEGREE,
                meters_per_lon=METERS_PER_LON_DEGREE,
            )

        filt_lat, filt_lon = self.kf.step(frame.lat, frame.lon)
        self.lat = filt_lat
        self.lon = filt_lon
        self.filtered_path.append((filt_lat, filt_lon))

        self.alt     = frame.alt
        self.speed   = frame.speed
        self.heading = frame.heading
        self.name    = frame.drone_id

    def snapshot(self) -> dict:
        """Return a dict matching SimulationEngine.get_state() drone format."""
        return {
            "id":            self.id,
            "name":          self.name,
            "behavior_type": self.behavior_type,
            "lat":           self.lat,
            "lon":           self.lon,
            "true_lat":      self.true_lat,
            "true_lon":      self.true_lon,
            "alt":           round(self.alt, 1),
            "speed":         round(self.speed, 1),
            "heading":       round(self.heading, 1),
            "path":          list(self.filtered_path),
            "true_path":     list(self.true_path),
        }


class IngestionAdapter:
    """
    Drop-in replacement for SimulationEngine when replaying recorded logs.

    Usage
    -----
        reader  = open_log("flight.csv", drone_id="UAV-REAL")
        adapter = IngestionAdapter(reader)
        adapter.start()            # begins background replay
        state   = adapter.get_state()   # same shape as SimulationEngine
    """

    def __init__(self, reader: CSVGPSReader | MAVLinkReader,
                 realtime: bool = True, speed_multiplier: float = 1.0):
        self.reader           = reader
        self.realtime         = realtime
        self.speed_multiplier = speed_multiplier
        self._drones: dict[str, _TrackedDrone] = {}
        self._lock            = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self.is_running       = False
        self._finished        = False

    def start(self) -> None:
        """Begin replaying frames in a background thread."""
        if not self.is_running:
            self.is_running = True
            self._thread = threading.Thread(target=self._replay_loop, daemon=True)
            self._thread.start()
            logger.info("IngestionAdapter: replay started.")

    def stop(self) -> None:
        self.is_running = False

    def _replay_loop(self) -> None:
        try:
            for frame in self.reader.stream(
                realtime=self.realtime,
                speed_multiplier=self.speed_multiplier,
            ):
                if not self.is_running:
                    break
                self._ingest_frame(frame)
        except Exception:
            logger.exception("IngestionAdapter: replay error")
        finally:
            self._finished = True
            logger.info("IngestionAdapter: replay finished.")

    def _ingest_frame(self, frame: TelemetryFrame) -> None:
        with self._lock:
            drone_id = frame.drone_id
            if drone_id not in self._drones:
                self._drones[drone_id] = _TrackedDrone(drone_id)
            self._drones[drone_id].update(frame)

            # Feed the threat engine (expects list of drone-like objects)
            threat_engine.process_step(list(self._drones.values()))

    def get_state(self) -> dict:
        """Return a snapshot matching SimulationEngine.get_state() format."""
        with self._lock:
            return {
                "drones": [d.snapshot() for d in self._drones.values()],
                "alerts": list(threat_engine.alerts),
            }
