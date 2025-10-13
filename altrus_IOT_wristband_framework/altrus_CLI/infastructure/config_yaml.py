from __future__ import annotations

from pathlib import Path

from altrus_cli.ports.interfaces import ConfigReader, RuntimeConfig

DEFAULT_ACTIVITIES = ["sleep", "rest", "walk", "run"]
DEFAULT_ANOMALIES = [
    "tachycardia",
    "bradycardia",
    "fever",
    "heart_attack",
    "cardiac_arrest",
]


class YamlConfigReader(ConfigReader):
    def __init__(self, path: Path) -> None:
        self._path = path

    def load(self) -> RuntimeConfig:
        if not self._path.exists():
            return RuntimeConfig(DEFAULT_ACTIVITIES, DEFAULT_ANOMALIES)

        activities: list[str] = []
        anomalies: list[str] = []
        current_key: str | None = None

        for line in self._path.read_text(encoding="utf-8").splitlines():
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            if not stripped.startswith("-") and stripped.endswith(":"):
                current_key = stripped[:-1].strip()
                continue
            if stripped.startswith("-") and current_key == "activities":
                activities.append(stripped.lstrip("-").strip())
            if stripped.startswith("-") and current_key == "anomalies":
                anomalies.append(stripped.lstrip("-").strip())

        return RuntimeConfig(activities or DEFAULT_ACTIVITIES, anomalies or DEFAULT_ANOMALIES)
