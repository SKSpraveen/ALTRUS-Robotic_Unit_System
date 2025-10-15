from __future__ import annotations

import importlib
import sys
from pathlib import Path

from altrus_cli.ports.interfaces import Predictor


class ProjectPredictor(Predictor):
    """Loads the generated project's inference pipeline dynamically."""

    def __init__(self, project_root: Path) -> None:
        self._project_root = project_root

    def predict(self, payload: dict, activities: list[str], anomalies: list[str]) -> dict:
        if str(self._project_root) not in sys.path:
            sys.path.insert(0, str(self._project_root))
        module = importlib.import_module("pipelines.inference")
        return module.run_inference(payload, activities, anomalies)
