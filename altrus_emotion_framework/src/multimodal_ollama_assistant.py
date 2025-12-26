from .config import AppConfig
from .assistant import MultimodalAssistant
from .llm import OllamaClient
from .asr import WhisperASR
from .tts import Pyttsx3TTS

def main():
    cfg = AppConfig()

    llm = OllamaClient(cfg.OLLAMA_URL, cfg.OLLAMA_MODEL, cfg.OLLAMA_TIMEOUT)
    asr = WhisperASR("base")
    tts = Pyttsx3TTS(rate=170, volume=1.0)

    app = MultimodalAssistant(cfg=cfg, llm=llm, asr=asr, tts=tts)
    app.run()

if __name__ == "__main__":
    main()
