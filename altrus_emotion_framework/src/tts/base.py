from abc import ABC, abstractmethod

class TTS(ABC):
    @abstractmethod
    def speak_async(self, text: str) -> None:
        raise NotImplementedError
