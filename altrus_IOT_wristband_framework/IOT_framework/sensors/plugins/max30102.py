from __future__ import annotations

from app.core.config import ProjectConfig
from app.sensors.plugins.base import SensorPlugin


class MAX30102(SensorPlugin):
    def validate(self, config: ProjectConfig) -> None:
        super().validate(config)
        if config.sample_rate_hz > 400:
            raise ValueError("MAX30102 sample rate should be <= 400Hz")
