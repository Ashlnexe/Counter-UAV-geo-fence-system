from pathlib import Path
from typing import Generator

from adapters.base_adapter import SensorAdapter
from data_ingestion import MAVLinkReader, TelemetryFrame

class MAVLinkAdapter(SensorAdapter):
    """
    Adapter that replays a MAVLink telemetry log (.tlog).
    Wraps the existing MAVLinkReader to conform to the SensorAdapter interface.
    """
    def __init__(self, path: str | Path, drone_id: str = "UAV-MAV"):
        super().__init__()
        self.reader = MAVLinkReader(path=path, drone_id=drone_id)

    def connect(self) -> bool:
        # File validation is handled by MAVLinkReader.__init__
        return True

    def disconnect(self) -> None:
        pass

    def get_stream(self) -> Generator[TelemetryFrame, None, None]:
        """
        Yields frames matching the real-time spacing of the original MAVLink log.
        """
        if not self.is_connected():
            return
            
        # MAVLinkReader handles the `time.sleep` logic when realtime=True
        for frame in self.reader.stream(realtime=True):
            if not self.is_connected():
                break
            yield frame
