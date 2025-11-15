from __future__ import annotations

from dataclasses import dataclass

from app.core.config import ProjectConfig
from app.sensors.models import SensorDefinition


@dataclass
class SensorPlugin:
    definition: SensorDefinition

    @property
    def key(self) -> str:
        return self.definition.key

    @property
    def display_name(self) -> str:
        return self.definition.display_name

    def validate(self, config: ProjectConfig) -> None:
        if not config.get_sensor_sample_rate(self.key):
            raise ValueError("Sample rate must be set")

    def template_context(self, config: ProjectConfig) -> dict:
        self.validate(config)
        return {
            "key": self.definition.key,
            "display_name": self.definition.display_name,
            "bus": self.definition.bus,
            "default_pins": self.definition.default_pins,
            "pins": config.get_sensor_pins(self.key, self.definition.default_pins),
            "sample_rate_hz": config.get_sensor_sample_rate(self.key),
        }
