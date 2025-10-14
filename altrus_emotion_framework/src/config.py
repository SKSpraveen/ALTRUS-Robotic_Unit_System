from dataclasses import dataclass
from pathlib import Path

@dataclass(frozen=True)
class AppConfig:
    ROOT: Path = Path(__file__).resolve().parents[1]
    MODELS_DIR: Path = ROOT / "models"

    FACE_MODEL_PATH: Path = MODELS_DIR / "face_emotion_detection_model.h5"
    VOICE_MODEL_PATH: Path = MODELS_DIR / "voice_model_best.h5"

    OLLAMA_URL: str = "http://localhost:11434/api/generate"
    OLLAMA_MODEL: str = "gemma3:1b"
    OLLAMA_TIMEOUT: int = 120

    SR: int = 16000
    N_MFCC: int = 40
    MAX_LEN: int = 300

    BLOCK_DUR: float = 0.1
    VAD_THRESHOLD: float = 0.01
    SILENCE_LIMIT: float = 0.7
    MIN_UTTERANCE: float = 1.0

    W_FACE: float = 0.6
    W_VOICE: float = 0.4

    LABELS: tuple = ("happy", "sad")

    FFMPEG_EXE: str | None = None
