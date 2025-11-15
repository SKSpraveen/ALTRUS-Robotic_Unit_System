from __future__ import annotations

from app.core.config import ProjectConfig
from app.sensors.plugins.base import SensorPlugin


class TMP117(SensorPlugin):
    def validate(self, config: ProjectConfig) -> None:
        super().validate(config)
        if config.get_sensor_sample_rate(self.key) > 10:
            raise ValueError("TMP117 sample rate should be <= 10Hz")
