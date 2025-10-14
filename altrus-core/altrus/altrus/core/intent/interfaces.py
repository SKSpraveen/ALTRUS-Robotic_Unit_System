from abc import ABC, abstractmethod
from typing import List
from .intent import Intent


class IntentSource(ABC):
    """
    Any system that produces intents (ML model, API, CLI)
    must implement this interface.
    """

    @abstractmethod
    def fetch_intents(self) -> List[Intent]:
        pass


class IntentResolver(ABC):
    """
    Resolves conflicts between multiple intents.
    """

    @abstractmethod
    def resolve(self, intents: List[Intent]) -> Intent:
        pass
