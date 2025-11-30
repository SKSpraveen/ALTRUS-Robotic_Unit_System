from abc import ABC, abstractmethod
from altrus.core.intent.intent import Intent


class Adapter(ABC):
    """
    Base contract for all external adapters.
    """

    @abstractmethod
    def translate(self, external_input) -> Intent:
        """
        Translate external input into a middleware Intent.
        """
        pass
