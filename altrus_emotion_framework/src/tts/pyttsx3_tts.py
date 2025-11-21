import threading
import pyttsx3
from .base import TTS
from ..utils.text import clean_reply

class Pyttsx3TTS(TTS):
    def __init__(self, rate: int = 170, volume: float = 1.0):
        self.rate = rate
        self.volume = volume
        self._lock = threading.Lock()
        self.speaking_event = threading.Event()

    def speak_async(self, text: str) -> None:
        text = clean_reply(text)
        if not text:
            return

        def _run():
            with self._lock:
                self.speaking_event.set()
                try:
                    print(f"\n[ROBOT]: {text}\n")
                    engine = pyttsx3.init()
                    engine.setProperty("rate", self.rate)
                    engine.setProperty("volume", self.volume)
                    engine.say(text)
                    engine.runAndWait()
                    engine.stop()
                finally:
                    self.speaking_event.clear()

        threading.Thread(target=_run, daemon=True).start()
