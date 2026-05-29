import queue
import threading
import logging
import math
import time
from typing import Dict, List, Any

from data_ingestion import TelemetryFrame
from adapters.base_adapter import SensorAdapter
from counter_uav_core import KalmanFilter6D, IMMFilter
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
        
        # Track lifecycle properties
        self.hits = 1
        self.is_confirmed = False
        
        # Dynamic inception
        base_window = 30.0 if frame.source == "opensky" else 5.0
        self.promotion_deadline = frame.timestamp + base_window
        
        self._update_wgs84_cache()

        # IMM Filter for position/velocity estimation (2D: easting/northing)
        self.imm = IMMFilter(noise_cv=0.01, noise_ca=10.0, meas_noise=3.0)

        # Legacy KalmanFilter6D retained ONLY for OOSM Mahalanobis gating
        self.kf = KalmanFilter6D(init_easting=easting, init_northing=northing, init_alt=frame.alt)
        self.kf.step_oosm(frame.timestamp, easting, northing, frame.alt, frame.noise_std_m, frame.timestamp)

    def _update_wgs84_cache(self):
        self._lat, self._lon = geo_utils.to_wgs84(self.easting, self.northing)
        self._true_lat, self._true_lon = geo_utils.to_wgs84(self.true_path[-1][0], self.true_path[-1][1])

    def update(self, frame: TelemetryFrame) -> None:
        easting, northing = geo_utils.to_utm(frame.lat, frame.lon)
        self.true_path.append((easting, northing, frame.alt))
        if len(self.true_path) > 100:
            self.true_path.pop(0)

        self.hits += 1
        
        # Sensor-dominant inheritance
        current_time = time.time()
        if frame.source == "opensky":
            self.promotion_deadline = max(self.promotion_deadline, current_time + 30.0)
        
        
        # Update OOSM gating filter (for track association only)
        self.kf.step_oosm(frame.timestamp, easting, northing, frame.alt, frame.noise_std_m, current_time)

        # IMM Filter update with dynamic dt
        dt = frame.timestamp - self.last_ts
        if dt > 0:
            self.imm.update(easting, northing, dt)
        
        self.easting = float(self.imm.x_out[0])
        self.northing = float(self.imm.x_out[1])
        self.alt = frame.alt  # Altitude passed through unfiltered (IMM is 2D)
        
        if self.hits >= 3 and current_time <= self.promotion_deadline and not self.is_confirmed:
            self.is_confirmed = True
            logger.info(f"Track {self.id} PROMOTED to ConfirmedTrack.")
            
        self.last_ts = max(self.last_ts, frame.timestamp)
        # Velocity directly from IMM state: x_out = [x, y, vx, vy, ax, ay]
        vx = float(self.imm.x_out[2])
        vy = float(self.imm.x_out[3])
        self.speed = math.sqrt(vx*vx + vy*vy)
        
        self.filtered_path.append((self.easting, self.northing, self.alt))
        if len(self.filtered_path) > 100:
            self.filtered_path.pop(0)
            
        self._update_wgs84_cache()

    def coast(self, current_time: float) -> None:
        """Phase 3: Coasting Logic. Predicts forward without a measurement update."""
        dt = current_time - self.last_ts
        if dt > 0:
            self.imm.predict(dt)
            self.kf.predict(dt)  # Keep gating filter in sync
            self.easting = float(self.imm.x_out[0])
            self.northing = float(self.imm.x_out[1])
            # alt unchanged during coast (no altitude model)
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
        return [geo_utils.to_wgs84(pt[0], pt[1]) for pt in self.filtered_path]

    @property
    def wgs84_true_path(self) -> list[tuple[float, float]]:
        return [geo_utils.to_wgs84(pt[0], pt[1]) for pt in self.true_path]

    @property
    def kalman_speed(self) -> float:
        vx = float(self.imm.x_out[2])
        vy = float(self.imm.x_out[3])
        return math.sqrt(vx*vx + vy*vy)

    @property
    def uncertainty_radius_m(self) -> float:
        # P_out[0,0] is Easting variance, P_out[1,1] is Northing variance
        max_var = max(float(self.imm.P_out[0, 0]), float(self.imm.P_out[1, 1]))
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
                
                # Cull ghost tracks that missed their promotion deadline
                dead_tracks = [d_id for d_id, d in self.drones.items() if not d.is_confirmed and now > d.promotion_deadline]
                for d_id in dead_tracks:
                    del self.drones[d_id]
                    logger.debug(f"Ghost track {d_id} culled.")

                if self.drones:
                    for drone in list(self.drones.values()):
                        # We coast if it's been more than 1.5s (to avoid jitter with 0.5s sim steps).
                        if now - drone.last_ts > 1.5:
                            drone.coast(now)
                            
                    # Threat engine only processes confirmed tracks
                    confirmed_drones = [d for d in self.drones.values() if d.is_confirmed]
                    threat_engine.process_step(confirmed_drones)

    def _process_frame(self, frame: TelemetryFrame) -> None:
        with self.lock:
            # Drop very old packets before doing distance checks
            now = time.time()
            if frame.timestamp < now - 15.0:
                return

            easting, northing = geo_utils.to_utm(frame.lat, frame.lon)
            
            best_drone = None
            best_dist = float('inf')
            
            for drone in self.drones.values():
                # Don't associate if packet is hopelessly old (>15s), the OOSM buffer only goes back 15s
                if frame.timestamp < now - 15.0:
                    continue
                    
                dist = drone.kf.compute_mahalanobis_oosm(
                    easting, northing, frame.alt, frame.timestamp, frame.noise_std_m
                )
                
                # Chi-squared 95% confidence interval for 3 DOF = 7.815
                if dist < 7.815 and dist < best_dist:
                    best_dist = dist
                    best_drone = drone
                    
            if best_drone is not None:
                best_drone.update(frame)
            else:
                # No track within Mahalanobis gate, spawn new TentativeTrack
                # Assign a synthetic ID based on the frame timestamp if not from a reliable radar
                new_id = frame.drone_id if frame.drone_id.startswith("UAV-") else f"TRK-{int(frame.timestamp*1000)}"
                frame.drone_id = new_id
                self.drones[new_id] = TrackedDrone(frame)
                logger.info(f"Spawned new TentativeTrack: {new_id} at dist {best_dist:.2f}")

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
                        "uncertainty_radius_m": round(d.uncertainty_radius_m, 1),
                        "source": d.source,
                        "hits": d.hits,
                    }
                    for d in self.drones.values() if d.is_confirmed
                ],
                "alerts": list(threat_engine.alerts)
            }

tracking_engine = TrackingEngine()
