import yaml
from pathlib import Path


class PolicyConfigLoader:
    @staticmethod
    def load(path: str | Path) -> dict:
        with open(path, "r") as f:
            return yaml.safe_load(f)
