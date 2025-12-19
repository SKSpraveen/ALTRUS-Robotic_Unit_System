from __future__ import annotations

import difflib
import shutil
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

from jinja2 import Environment, FileSystemLoader, select_autoescape
from ruamel.yaml import YAML

from app.core.config import ProjectConfig
from app.sensors.registry import SensorRegistry


@dataclass
class PreviewResult:
    files: list[str]
    diff: str


yaml = YAML()


class Generator:
    def __init__(self) -> None:
        templates_path = Path(__file__).resolve().parents[1] / "templates" / "project"
        self.env = Environment(
            loader=FileSystemLoader(str(templates_path)),
            autoescape=select_autoescape(),
            keep_trailing_newline=True,
        )

    def preview(self, config: ProjectConfig, registry: SensorRegistry) -> PreviewResult:
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            self._render_all(config, registry, temp_path)
            files, diff = self._diff_tree(config.project_path, temp_path)
        return PreviewResult(files=files, diff=diff)

    def generate(self, config: ProjectConfig, registry: SensorRegistry, log: callable) -> None:
        project_path = config.project_path
        project_path.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            self._render_all(config, registry, temp_path)
            self._apply_generated(project_path, temp_path, log)
        config.save()

    def _render_all(
        self,
        config: ProjectConfig,
        registry: SensorRegistry,
        destination: Path,
    ) -> None:
        sensors = [registry.get(sensor) for sensor in config.sensors]
        sensor_context = [sensor.template_context(config) for sensor in sensors]
        context = {
            "project_name": config.project_name,
            "transport": config.transport,
            "sample_rate_hz": config.sample_rate_hz,
            "display": config.display,
            "sensors": sensor_context,
        }

        self._render_template("CMakeLists.txt.j2", destination / "CMakeLists.txt", context)
        self._render_template("main/CMakeLists.txt.j2", destination / "main" / "CMakeLists.txt", context)
        self._render_template("main/app_main.c.j2", destination / "generated" / "main" / "app_main.c", context)
        self._render_template(
            "components/sensors/CMakeLists.txt.j2",
            destination / "generated" / "components" / "sensors" / "CMakeLists.txt",
            context,
        )
        self._render_template(
            "components/sensors/sensors.c.j2",
            destination / "generated" / "components" / "sensors" / "sensors.c",
            context,
        )
        self._render_template(
            "components/sensors/sensors.h.j2",
            destination / "generated" / "components" / "sensors" / "sensors.h",
            context,
        )
        for sensor in sensor_context:
            self._render_template(
                "components/sensors/sensor_stub.c.j2",
                destination / "generated" / "components" / "sensors" / f"{sensor['key']}.c",
                {"sensor": sensor},
            )
            self._render_template(
                "components/sensors/sensor_stub.h.j2",
                destination / "generated" / "components" / "sensors" / f"{sensor['key']}.h",
                {"sensor": sensor},
            )
        self._render_template(
            "components/transport/CMakeLists.txt.j2",
            destination / "generated" / "components" / "transport" / "CMakeLists.txt",
            context,
        )
        self._render_template(
            "components/transport/transport.c.j2",
            destination / "generated" / "components" / "transport" / "transport.c",
            context,
        )
        self._render_template(
            "components/transport/transport.h.j2",
            destination / "generated" / "components" / "transport" / "transport.h",
            context,
        )
        self._render_template(
            "components/config/config.h.j2",
            destination / "generated" / "components" / "config" / "config.h",
            context,
        )
        config_path = destination / "project_config.yaml"
        config_path.parent.mkdir(parents=True, exist_ok=True)
        with config_path.open("w", encoding="utf-8") as handle:
            yaml.dump(config.to_dict(), handle)

    def _render_template(self, template_name: str, output_path: Path, context: dict) -> None:
        template = self.env.get_template(template_name)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(template.render(context), encoding="utf-8")

    def _diff_tree(self, current_root: Path, temp_root: Path) -> tuple[list[str], str]:
        files = []
        diffs = []
        for temp_file in temp_root.rglob("*"):
            if temp_file.is_dir():
                continue
            relative = temp_file.relative_to(temp_root)
            target = current_root / relative
            files.append(str(relative))
            existing = target.read_text(encoding="utf-8") if target.exists() else ""
            new = temp_file.read_text(encoding="utf-8")
            if existing != new:
                diff = difflib.unified_diff(
                    existing.splitlines(),
                    new.splitlines(),
                    fromfile=str(relative),
                    tofile=str(relative),
                    lineterm="",
                )
                diffs.append("\n".join(diff))
        return files, "\n".join(diffs)

    def _apply_generated(self, project_root: Path, temp_root: Path, log: callable) -> None:
        for temp_file in temp_root.rglob("*"):
            if temp_file.is_dir():
                continue
            relative = temp_file.relative_to(temp_root)
            target = project_root / relative
            if not self._can_update(target):
                log(f"Skipping existing user file: {relative}")
                continue
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(temp_file, target)
            log(f"Updated {relative}")

    def _can_update(self, path: Path) -> bool:
        if path.name == "project_config.yaml":
            return True
        if "generated" in path.parts:
            return True
        return not path.exists()
