# =============================================================================
# data_ingestion.py — Real GPS Data Ingestion for Counter-UAV IDS
# =============================================================================
#
# PURPOSE
# -------
# This module provides a real data ingestion path so the system can process
# actual UAV telemetry — not just synthetic simulation.
#
# Two sources are supported:
#
#   1. CSV GPS Log  — universal format, works with any GPS logger or flight
#                     controller that can export to CSV (ArduPilot DataFlash,
#                     Mission Planner, u-blox u-center, etc.)
#
#   2. MAVLink tlog — binary telemetry logs produced by Mission Planner,
#                     QGroundControl, or any GCS recording a MAVLink stream.
#                     Requires: pip install pymavlink
#
# USAGE EXAMPLE
# -------------
#   # CSV replay:
#   reader = CSVGPSReader("logs/flight_001.csv", drone_id="UAV-01")
#   for frame in reader.stream(realtime=True):
#       kf.step(frame.lat, frame.lon)
#       threat_engine.process_frame(frame)
#
#   # MAVLink replay:
#   reader = MAVLinkReader("logs/2025-01-15.tlog", drone_id="UAV-01")
#   for frame in reader.stream(realtime=False):   # fast replay
#       ...
#
# CSV FORMAT
# ----------
# Required columns (case-insensitive, flexible naming):
#   timestamp  — ISO 8601 or Unix epoch (seconds). Also accepts: time, ts, t
#   lat        — Decimal degrees. Also accepts: latitude
#   lon        — Decimal degrees. Also accepts: lon, longitude, lng
#
# Optional columns (used if present, ignored if absent):
#   alt        — Altitude in metres (MSL)
#   speed      — Ground speed in m/s
#   heading    — Track heading in degrees (0–360)
#
# Example CSV:
#   timestamp,lat,lon,alt,speed,heading
#   2025-01-15T10:00:00,12.9716,77.5946,120.0,12.5,045.0
#   2025-01-15T10:00:01,12.9717,77.5947,121.2,12.6,046.0
# =============================================================================

from __future__ import annotations

import csv
import time
import logging
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Generator, Optional

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Unified telemetry frame — output type for all readers
# ---------------------------------------------------------------------------

@dataclass
class TelemetryFrame:
    """
    A single telemetry snapshot from a UAV.
    All readers normalise their source format into this structure.
    """
    drone_id: str
    timestamp: float           # Unix epoch seconds (UTC)
    lat: float                 # decimal degrees
    lon: float                 # decimal degrees
    alt: float = 0.0           # metres MSL
    speed: float = 0.0         # ground speed m/s
    heading: float = 0.0       # degrees, 0 = north
    source: str = "unknown"    # "csv" | "mavlink"
    received_time: float = 0.0 # set by the adapter thread when pushed to queue


# ---------------------------------------------------------------------------
# Column name normalisation helpers
# ---------------------------------------------------------------------------

_LAT_ALIASES  = {"lat", "latitude"}
_LON_ALIASES  = {"lon", "longitude", "lng"}
_TIME_ALIASES = {"timestamp", "time", "ts", "t", "datetime"}
_ALT_ALIASES  = {"alt", "altitude", "alt_m"}
_SPD_ALIASES  = {"speed", "groundspeed", "spd", "velocity"}
_HDG_ALIASES  = {"heading", "hdg", "track", "course"}


def _find_col(header: list[str], aliases: set[str]) -> Optional[str]:
    """Return the first header name that matches one of the aliases (case-insensitive)."""
    for h in header:
        if h.strip().lower() in aliases:
            return h
    return None


def _parse_timestamp(raw: str) -> float:
    """
    Parse a timestamp string to Unix epoch (float seconds).
    Handles:
      - Unix epoch as a float/int string  ("1705312800.0")
      - ISO 8601 with or without timezone ("2025-01-15T10:00:00Z")
      - Common date-time formats          ("2025-01-15 10:00:00")
    """
    raw = raw.strip()
    # try numeric first
    try:
        return float(raw)
    except ValueError:
        pass
    # try ISO / common formats
    for fmt in ("%Y-%m-%dT%H:%M:%S%z", "%Y-%m-%dT%H:%M:%SZ",
                "%Y-%m-%dT%H:%M:%S", "%Y-%m-%d %H:%M:%S",
                "%Y-%m-%d %H:%M:%S.%f"):
        try:
            dt = datetime.strptime(raw, fmt)
            if dt.tzinfo is None:
                dt = dt.replace(tzinfo=timezone.utc)
            return dt.timestamp()
        except ValueError:
            continue
    raise ValueError(f"Cannot parse timestamp: {raw!r}")


# ---------------------------------------------------------------------------
# CSV reader
# ---------------------------------------------------------------------------

class CSVGPSReader:
    """
    Replay a CSV GPS log through the Kalman filter and threat engine.

    Parameters
    ----------
    path      : Path to the CSV file.
    drone_id  : ID to stamp on emitted TelemetryFrames.
    delimiter : CSV delimiter (default: auto-detect from first line).
    """

    def __init__(self, path: str | Path, drone_id: str = "UAV-REAL",
                 delimiter: Optional[str] = None):
        self.path = Path(path)
        self.drone_id = drone_id
        self.delimiter = delimiter
        self._validate()

    def _validate(self) -> None:
        if not self.path.exists():
            raise FileNotFoundError(f"GPS log not found: {self.path}")
        if self.path.suffix.lower() not in {".csv", ".txt", ".log"}:
            logger.warning("File extension is not .csv — attempting to parse anyway.")

    def _sniff_delimiter(self, sample: str) -> str:
        """Auto-detect delimiter from a sample of the file."""
        for candidate in (",", "\t", ";", "|"):
            if candidate in sample:
                return candidate
        return ","

    def stream(self, realtime: bool = False,
               speed_multiplier: float = 1.0) -> Generator[TelemetryFrame, None, None]:
        """
        Yield TelemetryFrame objects from the CSV, one per row.

        Parameters
        ----------
        realtime         : If True, sleep between frames to match original timing.
        speed_multiplier : Replay speed (1.0 = real-time, 2.0 = double speed).
                          Only used when realtime=True.
        """
        with self.path.open(newline="", encoding="utf-8-sig") as f:
            sample = f.read(2048)
            f.seek(0)

            delim = self.delimiter or self._sniff_delimiter(sample)
            reader = csv.DictReader(f, delimiter=delim)

            # Normalise header
            if reader.fieldnames is None:
                raise ValueError("CSV appears to have no header row.")
            headers = list(reader.fieldnames)

            col_lat  = _find_col(headers, _LAT_ALIASES)
            col_lon  = _find_col(headers, _LON_ALIASES)
            col_time = _find_col(headers, _TIME_ALIASES)
            col_alt  = _find_col(headers, _ALT_ALIASES)
            col_spd  = _find_col(headers, _SPD_ALIASES)
            col_hdg  = _find_col(headers, _HDG_ALIASES)

            if col_lat is None or col_lon is None:
                raise ValueError(
                    f"CSV must have lat and lon columns. "
                    f"Found: {headers}. "
                    f"Accepted lat names: {_LAT_ALIASES}, lon names: {_LON_ALIASES}"
                )

            prev_ts: Optional[float] = None

            for row_num, row in enumerate(reader, start=2):
                try:
                    lat = float(row[col_lat])
                    lon = float(row[col_lon])
                except (ValueError, KeyError) as e:
                    logger.warning(f"Row {row_num}: skipping bad lat/lon — {e}")
                    continue

                ts = None
                if col_time:
                    try:
                        ts = _parse_timestamp(row[col_time])
                    except ValueError as e:
                        logger.warning(f"Row {row_num}: timestamp parse failed — {e}")

                alt     = float(row[col_alt])     if col_alt and row.get(col_alt)     else 0.0
                speed   = float(row[col_spd])     if col_spd and row.get(col_spd)     else 0.0
                heading = float(row[col_hdg])     if col_hdg and row.get(col_hdg)     else 0.0

                frame = TelemetryFrame(
                    drone_id=self.drone_id,
                    timestamp=ts or time.time(),
                    lat=lat,
                    lon=lon,
                    alt=alt,
                    speed=speed,
                    heading=heading,
                    source="csv",
                )

                if realtime and ts is not None and prev_ts is not None:
                    delta = (ts - prev_ts) / speed_multiplier
                    if 0 < delta < 10:   # sanity cap: ignore gaps > 10 s
                        time.sleep(delta)

                prev_ts = ts
                yield frame

    def frame_count(self) -> int:
        """Return total number of data rows (excludes header)."""
        with self.path.open(encoding="utf-8-sig") as f:
            return sum(1 for _ in f) - 1


# ---------------------------------------------------------------------------
# MAVLink tlog reader  (optional — requires pymavlink)
# ---------------------------------------------------------------------------

class MAVLinkReader:
    """
    Replay a MAVLink .tlog binary file.

    Extracts GPS_RAW_INT or GLOBAL_POSITION_INT messages and emits
    TelemetryFrame objects compatible with the rest of the pipeline.

    Requires:
        pip install pymavlink

    Supported log types:
        .tlog — Mission Planner / QGroundControl binary telemetry logs
        .bin  — ArduPilot DataFlash logs (if converted to tlog format)

    Parameters
    ----------
    path       : Path to the .tlog file.
    drone_id   : ID to stamp on emitted TelemetryFrames.
    sysid      : MAVLink system ID to filter (None = accept all).
    """

    def __init__(self, path: str | Path, drone_id: str = "UAV-REAL",
                 sysid: Optional[int] = None):
        self.path = Path(path)
        self.drone_id = drone_id
        self.sysid = sysid
        self._check_pymavlink()

    @staticmethod
    def _check_pymavlink() -> None:
        try:
            import pymavlink  # noqa: F401
        except ImportError:
            raise ImportError(
                "pymavlink is required for MAVLink log replay.\n"
                "Install it with: pip install pymavlink"
            )

    def stream(self, realtime: bool = False,
               speed_multiplier: float = 1.0) -> Generator[TelemetryFrame, None, None]:
        """
        Yield TelemetryFrame objects from a MAVLink tlog.

        Processes GLOBAL_POSITION_INT messages (preferred — includes heading
        and relative altitude) with fallback to GPS_RAW_INT.

        Parameters
        ----------
        realtime         : Replay at original recording speed if True.
        speed_multiplier : Replay speed factor (only used when realtime=True).
        """
        from pymavlink import mavutil

        mlog = mavutil.mavlink_connection(str(self.path))
        prev_ts: Optional[float] = None

        while True:
            msg = mlog.recv_match(
                type=["GLOBAL_POSITION_INT", "GPS_RAW_INT"],
                blocking=False
            )
            if msg is None:
                break

            if self.sysid is not None and msg.get_srcSystem() != self.sysid:
                continue

            ts = getattr(msg, "_timestamp", None) or time.time()

            if msg.get_type() == "GLOBAL_POSITION_INT":
                # lat/lon in 1e-7 degrees, alt in mm
                lat     = msg.lat  / 1e7
                lon     = msg.lon  / 1e7
                alt     = msg.alt  / 1000.0          # mm → m
                speed   = (msg.vx**2 + msg.vy**2)**0.5 / 100.0  # cm/s → m/s
                heading = msg.hdg / 100.0 if msg.hdg != 65535 else 0.0

            elif msg.get_type() == "GPS_RAW_INT":
                lat     = msg.lat  / 1e7
                lon     = msg.lon  / 1e7
                alt     = msg.alt  / 1000.0
                speed   = msg.vel  / 100.0 if msg.vel != 65535 else 0.0
                heading = msg.cog  / 100.0 if msg.cog != 65535 else 0.0

            else:
                continue

            # Skip invalid fixes (0,0 position or no fix)
            if lat == 0.0 and lon == 0.0:
                continue

            frame = TelemetryFrame(
                drone_id=self.drone_id,
                timestamp=ts,
                lat=lat,
                lon=lon,
                alt=alt,
                speed=speed,
                heading=heading,
                source="mavlink",
            )

            if realtime and prev_ts is not None:
                delta = (ts - prev_ts) / speed_multiplier
                if 0 < delta < 10:
                    time.sleep(delta)

            prev_ts = ts
            yield frame


# ---------------------------------------------------------------------------
# Convenience: auto-detect reader from file extension
# ---------------------------------------------------------------------------

def open_log(path: str | Path, drone_id: str = "UAV-REAL",
             **kwargs) -> CSVGPSReader | MAVLinkReader:
    """
    Return the appropriate reader based on file extension.

        .csv / .txt / .log  → CSVGPSReader
        .tlog / .bin        → MAVLinkReader

    Example
    -------
        reader = open_log("flight.tlog", drone_id="UAV-01")
        for frame in reader.stream(realtime=True):
            lat_f, lon_f = kf.step(frame.lat, frame.lon)
    """
    p = Path(path)
    ext = p.suffix.lower()

    if ext in {".tlog", ".bin"}:
        return MAVLinkReader(p, drone_id=drone_id, **kwargs)
    elif ext in {".csv", ".txt", ".log"}:
        return CSVGPSReader(p, drone_id=drone_id, **kwargs)
    else:
        raise ValueError(
            f"Unrecognised file extension '{ext}'. "
            f"Supported: .csv, .txt, .log, .tlog, .bin"
        )
