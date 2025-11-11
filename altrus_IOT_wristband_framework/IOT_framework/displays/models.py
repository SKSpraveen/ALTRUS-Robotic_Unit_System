from __future__ import annotations

from dataclasses import dataclass


@dataclass
class DisplayDefinition:
    key: str
    display_name: str
    bus: str
    description: str
    category: str
