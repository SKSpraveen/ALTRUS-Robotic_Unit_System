from __future__ import annotations

from pathlib import Path

from ruamel.yaml import YAML

from app.displays.models import DisplayDefinition


yaml = YAML()


class DisplayRegistry:
    def __init__(self, displays: list[DisplayDefinition]) -> None:
        self.displays = displays
        self._map = {display.key: display for display in displays}

    @classmethod
    def load_default(cls) -> "DisplayRegistry":
        definitions_dir = Path(__file__).parent / "definitions"
        definitions = [cls._load_definition(p) for p in definitions_dir.glob("*.yaml")]
        return cls(definitions)

    @staticmethod
    def _load_definition(path: Path) -> DisplayDefinition:
        with path.open("r", encoding="utf-8") as handle:
            data = yaml.load(handle)
        return DisplayDefinition(
            key=data["key"],
            display_name=data["display_name"],
            bus=data["bus"],
            description=data.get("description", ""),
            category=data.get("category", "Other"),
        )

    def get(self, key: str) -> DisplayDefinition:
        if key not in self._map:
            raise KeyError(f"Unknown display: {key}")
        return self._map[key]
