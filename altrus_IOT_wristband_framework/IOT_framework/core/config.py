from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from ruamel.yaml import YAML


yaml = YAML()


@dataclass
class ProjectConfig:
    project_name: str
    project_path: Path
    transport: str
    sample_rate_hz: int
    display: str | None = None
    sensors: list[str] = field(default_factory=list)
    sensor_settings: dict[str, dict[str, Any]] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "project_name": self.project_name,
            "transport": self.transport,
            "sample_rate_hz": self.sample_rate_hz,
            "display": self.display,
            "sensors": self.sensors,
            "sensor_settings": self.sensor_settings,
        }

    def save(self) -> None:
        config_path = self.project_path / "project_config.yaml"
        config_path.parent.mkdir(parents=True, exist_ok=True)
        with config_path.open("w", encoding="utf-8") as handle:
            yaml.dump(self.to_dict(), handle)

    def update_from_file(self, path: Path) -> None:
        with path.open("r", encoding="utf-8") as handle:
            data = yaml.load(handle) or {}
        self.project_name = data.get("project_name", self.project_name)
        self.transport = data.get("transport", self.transport)
        self.sample_rate_hz = data.get("sample_rate_hz", self.sample_rate_hz)
        self.display = data.get("display", self.display)
        self.sensors = data.get("sensors", self.sensors)
        self.sensor_settings = data.get("sensor_settings", self.sensor_settings)

    def get_sensor_sample_rate(self, sensor_key: str) -> int:
        settings = self.sensor_settings.get(sensor_key, {})
        value = settings.get("sample_rate_hz")
        if value is None:
            return self.sample_rate_hz
        try:
            return int(value)
        except (TypeError, ValueError):
            return self.sample_rate_hz

    def get_sensor_pins(self, sensor_key: str, defaults: dict) -> dict:
        settings = self.sensor_settings.get(sensor_key, {})
        resolved = {}
        for pin, default_value in defaults.items():
            value = settings.get(pin, default_value)
            resolved[pin] = value
        return resolved
