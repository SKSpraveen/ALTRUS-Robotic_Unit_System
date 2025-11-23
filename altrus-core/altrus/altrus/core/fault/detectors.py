from abc import ABC, abstractmethod
import time
from altrus.core.fault.state_machine import FaultSeverity



class FaultDetector(ABC):
    """
    Base class for all fault detectors.
    """

    @abstractmethod
    def check(self) -> bool:
        pass

    @abstractmethod
    def severity(self) -> FaultSeverity:
        pass


class HeartbeatDetector(FaultDetector):
    def __init__(self, timeout_seconds: int = 5, severity=FaultSeverity.HIGH):
        self.timeout_seconds = timeout_seconds
        self._severity = severity
        self.last_heartbeat = time.time()

    def heartbeat(self):
        self.last_heartbeat = time.time()

    def check(self) -> bool:
        return (time.time() - self.last_heartbeat) > self.timeout_seconds

    def severity(self) -> FaultSeverity:
        return self._severity
