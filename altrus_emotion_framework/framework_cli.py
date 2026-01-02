from __future__ import annotations

import re
import sys
import json
import shutil
import subprocess
from pathlib import Path
from typing import List, Optional

import typer
from rich.console import Console
from rich.prompt import Prompt, Confirm

app = typer.Typer(add_completion=False)
console = Console()

def _slugify(name: str) -> str:
    name = name.strip().lower()
    name = re.sub(r"\s+", "_", name)
    name = re.sub(r"[^a-z0-9_]+", "", name)
    return name or "emotion_project"

MAX_TF_SAFE_VERSION = (3, 13, 3)

def _python_version_ok() -> bool:
    v = sys.version_info
    return (v.major, v.minor, v.micro) <= MAX_TF_SAFE_VERSION


def _write_file(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def _create_venv(project_dir: Path) -> None:
    venv_dir = project_dir / ".venv"
    if venv_dir.exists():
        console.print("[yellow].venv already exists, skipping[/yellow]")
        return
    subprocess.check_call([sys.executable, "-m", "venv", str(venv_dir)])


def _copy_file(src: Path, dst: Path) -> None:
    if not src.exists():
        console.print(f"[yellow]Missing template file: {src} (skipping)[/yellow]")
        return
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)


def _copy_dir(src: Path, dst: Path, ignore: Optional[callable] = None) -> None:
    if not src.exists():
        console.print(f"[yellow]Missing template dir: {src} (skipping)[/yellow]")
        return
    if dst.exists():
        shutil.rmtree(dst)
    shutil.copytree(src, dst, ignore=ignore)


def _update_appconfig_ollama(config_path: Path, url: str, model: str, timeout: int) -> None:
    if not config_path.exists():
        console.print(f"[yellow]{config_path} not found; skipping config update[/yellow]")
        return

    text = config_path.read_text(encoding="utf-8")

    text, n1 = re.subn(
        r'(^\s*OLLAMA_URL\s*:\s*str\s*=\s*")[^"]*(")',
        rf'\1{url}\2',
        text,
        flags=re.MULTILINE,
    )

    text, n2 = re.subn(
        r'(^\s*OLLAMA_MODEL\s*:\s*str\s*=\s*")[^"]*(")',
        rf'\1{model}\2',
        text,
        flags=re.MULTILINE,
    )

    text, n3 = re.subn(
        r'(^\s*OLLAMA_TIMEOUT\s*:\s*int\s*=\s*)\d+',
        rf'\g<1>{timeout}',
        text,
        flags=re.MULTILINE,
    )

    config_path.write_text(text, encoding="utf-8")

    if n1 and n2 and n3:
        console.print("[green]Updated Ollama settings in src/config.py[/green]")
    else:
        console.print(
            f"[yellow]Config update partially applied (url:{n1}, model:{n2}, timeout:{n3}). "
            f"Check src/config.py formatting.[/yellow]"
        )

@app.command()
def init():
    """Initialize a new Emotion Detection Framework"""

    v = sys.version_info
    if not _python_version_ok():
        console.print(
            f"[yellow]Warning: Python {v.major}.{v.minor}.{v.micro} detected. "
            f"TensorFlow may not work properly. "
            f"Recommended: Python <= 3.13.3 (max supported: 3.13.3).[/yellow]"
        )

    TEMPLATE_ROOT = Path(__file__).resolve().parent

    project_name_raw = Prompt.ask("Project name", default="emotion_detection_framework")
    project_name = _slugify(project_name_raw)

    parent = Path(Prompt.ask("Create project in which folder?", default=str(Path.cwd())))
    project_dir = parent / project_name

    if project_dir.exists():
        if not Confirm.ask("Folder exists. Overwrite?", default=False):
            raise typer.Exit()
        shutil.rmtree(project_dir)

    project_dir.mkdir(parents=True, exist_ok=True)

    console.print("\n[bold]LLM (Ollama)[/bold]")
    ollama_url = Prompt.ask("Ollama URL", default="http://localhost:11434/api/generate")
    ollama_model = Prompt.ask("Model name", default="gemma3:1b")
    ollama_timeout = int(Prompt.ask("Timeout (seconds)", default="120"))

    create_venv = Confirm.ask("Create virtual environment (.venv)?", default=False)

    console.print("\n[cyan]Creating project from template...[/cyan]")

    ignore_common = shutil.ignore_patterns(
        "__pycache__",
        "*.pyc",
        "*.pyo",
        "*.pyd",
        ".pytest_cache",
        ".mypy_cache",
        ".ruff_cache",
        ".DS_Store",
        "Thumbs.db",
        ".git",
        ".idea",
        ".vscode",
        "venv",
        ".venv",
        "dist",
        "build",
        "*.egg-info",
    )

    _copy_dir(TEMPLATE_ROOT / "models", project_dir / "models", ignore=ignore_common)
    _copy_dir(TEMPLATE_ROOT / "notebooks", project_dir / "notebooks", ignore=ignore_common)
    _copy_dir(TEMPLATE_ROOT / "src", project_dir / "src", ignore=ignore_common)

    _copy_file(TEMPLATE_ROOT / ".gitignore", project_dir / ".gitignore")
    _copy_file(TEMPLATE_ROOT / "requirements.txt", project_dir / "requirements.txt")
    _copy_file(TEMPLATE_ROOT / "README.md", project_dir / "README.md")

    config_path = project_dir / "src" / "config.py"
    _update_appconfig_ollama(config_path, ollama_url, ollama_model, ollama_timeout)

    _write_file(
        project_dir / "project_config.json",
        json.dumps(
            {
                "project": project_name,
                "ollama_model": ollama_model,
                "ollama_url": ollama_url,
                "ollama_timeout": ollama_timeout,
            },
            indent=2,
        ),
    )

    if create_venv:
        _create_venv(project_dir)

    console.print("[green]Project created successfully![/green]")
    console.print(f"cd {project_dir}")
    console.print("pip install -r requirements.txt")
    console.print("python -m src.multimodal_ollama_assistant")


@app.callback()
def main():
    """Emotion Detection Framework CLI"""
    return


if __name__ == "__main__":
    app()
