import queue
import threading
import logging
import math
import time
from typing import Dict, List, Any

from data_ingestion import TelemetryFrame
from adapters.base_adapter import SensorAdapter
from kalman import KalmanFilter6D
import geo_utils
from threat_engine import threat_engine

logger = logging.getLogger(__name__)

class TrackedDrone:
    """
    State container for a drone being tracked by the central engine.
    Maintains history, true path vs filtered path, and the Kalman filter.
    """
    def __init__(self, frame: TelemetryFrame):
        self.id = frame.drone_id
        self.name = frame.drone_id
        self.last_ts = frame.timestamp
        self.behavior_type = "REAL_FLIGHT"
        self.source = frame.source
        
        # Convert initial WGS84 to UTM
        easting, northing = geo_utils.to_utm(frame.lat, frame.lon)
        
        self.true_path = [(easting, northing, frame.alt)]
        self.filtered_path = [(easting, northing, frame.alt)]
        self.easting = easting
        self.northing = northing
        self.alt = frame.alt
        self.speed = frame.speed
        self.heading = frame.heading
        
        self._update_wgs84_cache()
        self.kf = KalmanFilter6D(init_easting=easting, init_northing=northing, init_alt=frame.alt)

    def _update_wgs84_cache(self):
        self._lat, self._lon = geo_utils.to_wgs84(self.easting, self.northing)
        self._true_lat, self._true_lon = geo_utils.to_wgs84(self.true_path[-1][0], self.true_path[-1][1])

    def update(self, frame: TelemetryFrame) -> None:
        easting, northing = geo_utils.to_utm(frame.lat, frame.lon)
        self.true_path.append((easting, northing, frame.alt))
        if len(self.true_path) > 100:
            self.true_path.pop(0)

        dt = frame.timestamp - self.last_ts
        if dt > 0:
            self.kf.predict(dt)
            dist = self.kf.compute_mahalanobis_distance(easting, northing, frame.alt, noise_std_m=frame.noise_std_m)
            # 7.815 is the chi-squared 95% confidence interval for 3 degrees of freedom
            if dist > 7.815:
                logger.warning(f"Measurement rejected by Mahalanobis gate: dist={dist:.2f} > 7.815")
            else:
                self.kf.update(easting, northing, frame.alt, noise_std_m=frame.noise_std_m)
            filt_e, filt_n, filt_alt = float(self.kf.x[0]), float(self.kf.x[1]), float(self.kf.x[2])
        else:
            filt_e, filt_n, filt_alt = self.easting, self.northing, self.alt

        self.last_ts = frame.timestamp
        self.easting = filt_e
        self.northing = filt_n
        self.alt = filt_alt
        self.filtered_path.append((filt_e, filt_n, filt_alt))
        if len(self.filtered_path) > 100:
            self.filtered_path.pop(0)

        # We keep the original raw speed and heading for reference, but use KF metrics for threats
        self.speed = frame.speed
        self.heading = frame.heading
        self._update_wgs84_cache()

    def coast(self, current_time: float) -> None:
        """Phase 3: Coasting Logic. Predicts forward without a measurement update."""
        dt = current_time - self.last_ts
        if dt > 0:
            self.kf.predict(dt)
            self.easting = float(self.kf.x[0])
            self.northing = float(self.kf.x[1])
            self.alt = float(self.kf.x[2])
            self.last_ts = current_time
            self.filtered_path.append((self.easting, self.northing, self.alt))
            if len(self.filtered_path) > 100:
                self.filtered_path.pop(0)
            self._update_wgs84_cache()

    @property
    def lat(self) -> float:
        return self._lat

    @property
    def lon(self) -> float:
        return self._lon

    @property
    def true_lat(self) -> float:
        return self._true_lat

    @property
    def true_lon(self) -> float:
        return self._true_lon

    @property
    def wgs84_filtered_path(self) -> list[tuple[float, float]]:
        return [geo_utils.to_wgs84(e, n) for e, n in self.filtered_path]

    @property
    def wgs84_true_path(self) -> list[tuple[float, float]]:
        return [geo_utils.to_wgs84(e, n) for e, n in self.true_path]

    @property
    def kalman_speed(self) -> float:
        return self.kf.estimated_speed_mps

    @property
    def uncertainty_radius_m(self) -> float:
        # P[0,0] is Easting variance, P[1,1] is Northing variance
        max_var = max(self.kf.P[0, 0], self.kf.P[1, 1])
        # Return 3-sigma bound
        return 3.0 * math.sqrt(max(max_var, 0.0))


class TrackingEngine:
    """
    Central consumer of telemetry data. Pulls frames from a thread-safe queue,
    updates tracking states, applies Kalman filtering, and passes states to the Threat Engine.
    """
    def __init__(self):
        self.q = queue.Queue(maxsize=1000)
        self.adapters: List[SensorAdapter] = []
        self.drones: Dict[str, TrackedDrone] = {}
        self.is_running = False
        self._thread: threading.Thread | None = None
        self.lock = threading.Lock()
        
    def start(self, adapters: List[SensorAdapter]) -> None:
        self.adapters = adapters
        for adapter in self.adapters:
            adapter.start(self.q)
            
        self.is_running = True
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self.is_running = False
        for adapter in self.adapters:
            adapter.stop()
        if self._thread:
            self._thread.join(timeout=1.0)

    def _run_loop(self) -> None:
        while self.is_running:
            # 1. Drain the queue of all immediately available frames
            frames_processed = 0
            while not self.q.empty() and frames_processed < 50: # Batch limit to prevent starvation
                try:
                    frame: TelemetryFrame = self.q.get_nowait()
                    self._process_frame(frame)
                    frames_processed += 1
                except queue.Empty:
                    break
            
            # 2. If the queue was empty, wait a fraction of a second so we don't redline the CPU
            if frames_processed == 0:
                time.sleep(0.05)
                
            # 3. Evaluate threats exactly ONCE per batch cycle
            with self.lock:
                now = time.time()
                for drone in self.drones.values():
                    # Phase 3 Coasting: Predict forward if no data received recently
                    # We coast if it's been more than 1.5s (to avoid jitter with 0.5s sim steps).
                    if now - drone.last_ts > 1.5:
                        drone.coast(now)
                        
                threat_engine.process_step(list(self.drones.values()))

    def _process_frame(self, frame: TelemetryFrame) -> None:
        with self.lock:
            if frame.drone_id not in self.drones:
                self.drones[frame.drone_id] = TrackedDrone(frame)
            else:
                drone = self.drones[frame.drone_id]
                # Drop out-of-order packets
                if frame.timestamp < drone.last_ts:
                    logger.warning(f"Dropping out of order packet for {frame.drone_id}")
                    return
                drone.update(frame)

    def get_state(self) -> Dict[str, Any]:
        """Snapshot of system state for the dashboard."""
        with self.lock:
            return {
                "drones": [
                    {
                        "id": d.id,
                        "name": d.name,
                        "behavior_type": d.behavior_type,
                        "lat": d.lat,
                        "lon": d.lon,
                        "true_lat": d.true_lat,
                        "true_lon": d.true_lon,
                        "alt": round(d.alt, 1),
                        "speed": round(d.kalman_speed, 1),
                        "heading": round(d.heading, 1),
                        "path": d.wgs84_filtered_path,
                        "true_path": d.wgs84_true_path,
                    }
                    for d in self.drones.values()
                ],
                "alerts": list(threat_engine.alerts)
            }

tracking_engine = TrackingEngine()
