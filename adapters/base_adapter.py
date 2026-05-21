from abc import ABC, abstractmethod
from typing import Generator
import time

from data_ingestion import TelemetryFrame

class SensorAdapter(ABC):
    """
    Abstract base class for all sensor ingestion methods (Simulator, CSV, MAVLink, OpenSky, etc.).
    This layer decouples HOW data is collected from HOW it is tracked and evaluated.
    """
    
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
        Yields telemetry frames.
        """
        pass

    @abstractmethod
    def is_connected(self) -> bool:
        """Returns True if the adapter is actively connected to its data source."""
        pass
