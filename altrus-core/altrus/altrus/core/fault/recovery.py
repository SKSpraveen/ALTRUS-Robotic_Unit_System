from abc import ABC, abstractmethod


class RecoveryStrategy(ABC):
    """
    Base class for fault recovery strategies.
    """

    @abstractmethod
    def execute(self) -> None:
        pass


class RestartModuleStrategy(RecoveryStrategy):
    def execute(self) -> None:
        print("[Recovery] Restarting module...")


class SafeModeStrategy(RecoveryStrategy):
    def execute(self) -> None:
        print("[Recovery] Switching system to safe mode...")
