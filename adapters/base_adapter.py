from abc import ABC, abstractmethod
from typing import Generator
import threading
import queue
import time

from data_ingestion import TelemetryFrame

class SensorAdapter(ABC):
    """
    Abstract base class for all sensor ingestion methods.
    Adapters act as threaded producers, pushing TelemetryFrames onto a central queue.
    """
    def __init__(self):
        self.out_queue: queue.Queue | None = None
        self._thread: threading.Thread | None = None
        self._is_connected = False

    def start(self, out_queue: queue.Queue) -> bool:
        """Starts the background thread to poll the sensor and push to the queue."""
        self.out_queue = out_queue
        if self.connect():
            self._is_connected = True
            self._thread = threading.Thread(target=self._run_loop, daemon=True)
            self._thread.start()
            return True
        return False

    def stop(self) -> None:
        """Stops the background thread and disconnects."""
        self._is_connected = False
        self.disconnect()
        if self._thread:
            self._thread.join(timeout=1.0)

    def _run_loop(self) -> None:
        """The core producer loop running in the background thread."""
        for frame in self.get_stream():
            if not self._is_connected:
                break
            frame.received_time = time.time()
            if self.out_queue:
                self.out_queue.put(frame, block=True, timeout=None)

    @abstractmethod
    def connect(self) -> bool:
        """Open connection to data source. Returns True on success."""
        pass
        
    @abstractmethod
    def disconnect(self) -> None:
        """Clean up and close connection."""
        pass

    @abstractmethod
    def get_stream(self) -> Generator[TelemetryFrame, None, None]:
        """
        Yields telemetry frames. Blocking generator.
        """
        pass

    def is_connected(self) -> bool:
        """Returns True if the adapter is actively connected to its data source."""
        return self._is_connected
