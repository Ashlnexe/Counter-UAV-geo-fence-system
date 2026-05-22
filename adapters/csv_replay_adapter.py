from pathlib import Path
from typing import Generator

from adapters.base_adapter import SensorAdapter
from data_ingestion import CSVGPSReader, TelemetryFrame

class CSVReplayAdapter(SensorAdapter):
    """
    Adapter that replays a CSV GPS log.
    Wraps the existing CSVGPSReader to conform to the SensorAdapter interface.
    """
    def __init__(self, path: str | Path, drone_id: str = "UAV-REAL", speed_multiplier: float = 1.0):
        super().__init__()
        self.reader = CSVGPSReader(path=path, drone_id=drone_id)
        self.speed_multiplier = speed_multiplier

    def connect(self) -> bool:
        self._is_connected = True
        return True

    def disconnect(self) -> None:
        pass

    def get_stream(self) -> Generator[TelemetryFrame, None, None]:
        """
        Yields frames matching the real-time spacing of the original CSV log.
        """
        if not self.is_connected():
            return
            
        # CSVGPSReader handles the `time.sleep` logic when realtime=True
        for frame in self.reader.stream(realtime=True, speed_multiplier=self.speed_multiplier):
            if not self.is_connected():
                break
            yield frame
