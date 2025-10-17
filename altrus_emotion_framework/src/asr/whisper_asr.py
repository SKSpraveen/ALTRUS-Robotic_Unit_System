import os
import tempfile
import numpy as np
import soundfile as sf

class WhisperASR:
    def __init__(self, model_name: str = "base"):
        self.model_name = model_name
        self._model = None

    def _load(self):
        import whisper
        if self._model is None:
            self._model = whisper.load_model(self.model_name)

    def transcribe(self, audio_signal: np.ndarray, sr: int = 16000) -> str:
        try:
            self._load()
        except Exception as e:
            print("[WHISPER] load/import failed:", e)
            return ""

        fd, path = tempfile.mkstemp(suffix=".wav")
        os.close(fd)

        try:
            sf.write(path, audio_signal, sr)
            result = self._model.transcribe(path, fp16=False, language="en", task="transcribe")
            return (result.get("text") or "").strip()
        except Exception as e:
            print("[WHISPER] transcription failed:", e)
            return ""
        finally:
            try:
                os.remove(path)
            except Exception:
                pass
