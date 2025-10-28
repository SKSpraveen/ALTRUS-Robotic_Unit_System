import requests
from .base import LLMClient

class OllamaClient(LLMClient):
    def __init__(self, url: str, model: str, timeout: int = 120):
        self.url = url
        self.model = model
        self.timeout = timeout

    def generate(self, prompt: str) -> str:
        payload = {
            "model": self.model,
            "prompt": prompt,
            "stream": False,
            "options": {
                "temperature": 0.7,
                "top_p": 0.9,
                "num_predict": 120,
            },
        }
        r = requests.post(self.url, json=payload, timeout=self.timeout)
        r.raise_for_status()
        data = r.json()
        return (data.get("response") or "").strip()
