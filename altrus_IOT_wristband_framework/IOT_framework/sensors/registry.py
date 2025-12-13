from __future__ import annotations

from pathlib import Path

from ruamel.yaml import YAML

from app.sensors.models import SensorDefinition
from app.sensors.plugins.base import SensorPlugin
from app.sensors.plugins.bmi270 import BMI270
from app.sensors.plugins.max30102 import MAX30102
from app.sensors.plugins.tmp117 import TMP117


yaml = YAML()


class SensorRegistry:
    def __init__(self, sensors: list[SensorPlugin]) -> None:
        self.sensors = sensors
        self._map = {sensor.key: sensor for sensor in sensors}

    @classmethod
    def load_default(cls) -> "SensorRegistry":
        definitions_dir = Path(__file__).parent / "definitions"
        definitions = {p.stem: cls._load_definition(p) for p in definitions_dir.glob("*.yaml")}
        sensors = [
            MAX30102(definitions["max30102"]),
            BMI270(definitions["bmi270"]),
            TMP117(definitions["tmp117"]),
        ]
        return cls(sensors)

    @staticmethod
    def _load_definition(path: Path) -> SensorDefinition:
        with path.open("r", encoding="utf-8") as handle:
            data = yaml.load(handle)
        return SensorDefinition(
            key=data["key"],
            display_name=data["display_name"],
            bus=data["bus"],
            default_pins=data.get("default_pins", {}),
            description=data.get("description", ""),
            category=data.get("category", "Other"),
            default_sample_rate_hz=data.get("default_sample_rate_hz"),
        )

    def get(self, key: str) -> SensorPlugin:
        if key not in self._map:
            raise KeyError(f"Unknown sensor: {key}")
        return self._map[key]
