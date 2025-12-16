# registry/storage.py
import json
from pathlib import Path
from typing import List
from altrus.core.registry.module import Module  # Import Module class

class RegistryStorage:
    def __init__(self, path: Path | None = None):
        base_dir = Path.home() / ".altrus"
        base_dir.mkdir(exist_ok=True)

        self.path = path or (base_dir / "registry.json")

    def load(self) -> List[Module]:
        if not self.path.exists():
            return []

        with self.path.open("r") as f:
            raw = json.load(f)

        # Use from_dict to reconstruct Module objects
        return [Module.from_dict(m) for m in raw]

    def save(self, modules: List[Module]):
        # Use to_dict for proper serialization
        serializable_modules = [m.to_dict() for m in modules]

        with self.path.open("w") as f:
            json.dump(
                serializable_modules,
                f,
                indent=2
            )