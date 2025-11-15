from __future__ import annotations

from dataclasses import dataclass


@dataclass
class SensorDefinition:
    key: str
    display_name: str
    bus: str
    default_pins: dict
    description: str
    category: str
    default_sample_rate_hz: int | None = None
