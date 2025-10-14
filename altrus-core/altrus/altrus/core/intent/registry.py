from pathlib import Path
import yaml
from .definition import IntentDefinition


DEFAULT_INTENTS_YAML = """\
intents:
  NAVIGATE:
    priority: 3
    timeout: 30
    ttl: 20
    preemptible: true
    route:
      capability: navigation.move
      fallback: safe_stop

  EMERGENCY_STOP:
    priority: 4
    timeout: 5
    ttl: 5
    preemptible: false
    route:
      capability: emergency_control

  TELEMEDICINE_CALL:
    priority: 3
    timeout: 60
    ttl: 5
    preemptible: true
    route:
      capability: telemedicine.call
"""


class IntentDefinitionRegistry:
    def __init__(self, path: Path | None = None):
        base_dir = Path.home() / ".altrus"
        base_dir.mkdir(exist_ok=True)

        self.path = path or (base_dir / "intents.yaml")
        self.definitions = {}

        self._ensure_file_exists()
        self._load()

    def _ensure_file_exists(self):
        if not self.path.exists():
            with self.path.open("w") as f:
                f.write(DEFAULT_INTENTS_YAML)

    def _load(self):
        with self.path.open("r") as f:
            raw = yaml.safe_load(f) or {}

        for name, cfg in raw.get("intents", {}).items():
            self.definitions[name] = IntentDefinition(
                name=name,
                priority=cfg["priority"],
                timeout=cfg["timeout"],
                ttl=cfg["ttl"],
                preemptible=cfg["preemptible"],
                route=cfg["route"],
            )

    def get(self, name: str):
        return self.definitions.get(name)
