from __future__ import annotations

from app.core.config import ProjectConfig
from app.sensors.plugins.base import SensorPlugin


class BMI270(SensorPlugin):
    def validate(self, config: ProjectConfig) -> None:
        super().validate(config)
        if config.sample_rate_hz > 2000:
            raise ValueError("BMI270 sample rate should be <= 2000Hz")
