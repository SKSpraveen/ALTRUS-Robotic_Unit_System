from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Iterable


class IdfRunner:
    def resolve_idf(self) -> tuple[list[str], dict[str, str], Path | None, Path | None] | None:
        idf_on_path = shutil.which("idf.py")
        if idf_on_path:
            repo_path = self._resolve_repo_from_idf(Path(idf_on_path))
            if repo_path:
                env, python = self._idf_env(repo_path)
                if python:
                    return ([str(python), idf_on_path], env, repo_path, python)
            env = os.environ.copy()
            env.pop("XTENSA_GNU_CONFIG", None)
            return ([idf_on_path], env, repo_path, None)

        repo_path = self._default_idf_repo()
        idf_py = repo_path / "tools" / "idf.py"
        if idf_py.exists():
            env, python = self._idf_env(repo_path)
            return ([str(python or sys.executable), str(idf_py)], env, repo_path, python)
        return None

    def is_available(self) -> bool:
        return self.resolve_idf() is not None

    def install(self, install_root: Path) -> Iterable[str]:
        if self.is_available():
            return ["ESP-IDF already available. Skipping installation."]
        if not shutil.which("git"):
            return ["git not found on PATH. Please install git to install ESP-IDF."]

        install_root = self._default_idf_repo().parent
        install_root.mkdir(parents=True, exist_ok=True)
        repo_path = install_root / "esp-idf"
        if not repo_path.exists():
            yield f"Cloning ESP-IDF into {repo_path}..."
            yield from self._run_process(
                ["git", "clone", "--depth", "1", "https://github.com/espressif/esp-idf.git", str(repo_path)]
            )
        else:
            yield f"ESP-IDF already exists at {repo_path}."

        if sys.platform.startswith("win"):
            installer = repo_path / "install.bat"
            command = [str(installer)]
        else:
            installer = repo_path / "install.sh"
            command = ["bash", str(installer)]
        if not installer.exists():
            return ["ESP-IDF installer not found. Please verify the repository was cloned correctly."]
        yield "Running ESP-IDF installer..."
        yield from self._run_process(command, cwd=repo_path)
        yield from self._ensure_submodules(repo_path)
        yield "ESP-IDF installation finished. Please restart the app so PATH updates take effect."

    def run(self, command: str, project_path: Path, port: str | None = None) -> Iterable[str]:
        resolved = self.resolve_idf()
        if not resolved:
            return ["idf.py not found. Please ensure ESP-IDF is installed and available on PATH."]
        return self._run_idf_with_recovery(command, project_path, port, resolved)

    def _run_process(
        self, command: list[str], cwd: Path | None = None, env: dict[str, str] | None = None
    ) -> Iterable[str]:
        process = subprocess.Popen(
            command,
            cwd=cwd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            env=env or os.environ,
        )
        if not process.stdout:
            return ["Failed to start process."]
        for line in process.stdout:
            yield line.rstrip()
        process.wait()
        if process.returncode != 0:
            yield f"Command failed with code {process.returncode}: {' '.join(command)}"

    def _run_idf_with_recovery(
        self,
        command: str,
        project_path: Path,
        port: str | None,
        resolved: tuple[list[str], dict[str, str], Path | None, Path | None],
    ) -> Iterable[str]:
        cmd, env, repo_path, python = resolved
        env.pop("XTENSA_GNU_CONFIG", None)
        if repo_path:
            yield from self._ensure_submodules(repo_path, env=env)
            yield from self._prepare_tools(repo_path, env, python)
        exit_code, output = yield from self._execute_idf(cmd, env, project_path, command, port)
        if exit_code == 0:
            return
        if self._has_dynconfig_conflict(output) and repo_path:
            yield "Detected ESP-IDF toolchain config conflict. Repairing toolchain and retrying..."
            yield from self._repair_toolchain(repo_path, env, python, project_path)
            exit_code, output = yield from self._execute_idf(cmd, env, project_path, command, port)
        if exit_code != 0:
            yield f"idf.py {command} exited with code {exit_code}"

    def _execute_idf(
        self,
        cmd: list[str],
        env: dict[str, str],
        project_path: Path,
        command: str,
        port: str | None,
    ) -> Iterable[str]:
        run_cmd = cmd.copy()
        if port:
            run_cmd.extend(["-p", port])
        run_cmd.append(command)
        process = subprocess.Popen(
            run_cmd,
            cwd=project_path,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            env=env,
        )
        if not process.stdout:
            return 1, ["Failed to start idf.py"]
        output: list[str] = []
        for line in process.stdout:
            cleaned = line.rstrip()
            output.append(cleaned)
            yield cleaned
        process.wait()
        return process.returncode or 0, output

    @staticmethod
    def _has_dynconfig_conflict(output: list[str]) -> bool:
        return any(
            "XTENSA_GNU_CONFIG" in line and "-dynconfig=" in line
            for line in output
        )

    def _repair_toolchain(
        self,
        repo_path: Path,
        env: dict[str, str],
        python: Path | None,
        project_path: Path,
    ) -> Iterable[str]:
        idf_tools = repo_path / "tools" / "idf_tools.py"
        if not idf_tools.exists():
            yield "ESP-IDF tools missing. Unable to repair toolchain."
            return
        python_exec = python or Path(sys.executable)
        yield "Removing xtensa-esp-elf toolchain directory..."
        tools_root = Path(env.get("IDF_TOOLS_PATH", Path.home() / ".espressif")) / "tools"
        toolchain_dir = tools_root / "xtensa-esp-elf"
        if toolchain_dir.exists():
            shutil.rmtree(toolchain_dir, ignore_errors=True)
        yield "Reinstalling ESP-IDF tools..."
        yield from self._run_process(
            [str(python_exec), str(idf_tools), "install"],
            cwd=repo_path,
            env=env,
        )
        self._export_idf_env(repo_path, env, python_exec, idf_tools)
        build_dir = project_path / "build"
        if build_dir.exists():
            yield f"Removing stale build directory: {build_dir}"
            shutil.rmtree(build_dir, ignore_errors=True)
    @staticmethod
    def _default_idf_repo() -> Path:
        return Path.home() / ".esp-idf-tools" / "esp-idf"

    @staticmethod
    def _resolve_repo_from_idf(idf_path: Path) -> Path | None:
        resolved = idf_path.resolve()
        if resolved.name != "idf.py":
            return None
        if resolved.parent.name != "tools":
            return None
        return resolved.parent.parent

    def _idf_env(self, repo_path: Path) -> tuple[dict[str, str], Path | None]:
        env = os.environ.copy()
        env["IDF_PATH"] = str(repo_path)
        env.pop("XTENSA_GNU_CONFIG", None)
        tools_path = Path(env.get("IDF_TOOLS_PATH", Path.home() / ".espressif"))
        env["IDF_TOOLS_PATH"] = str(tools_path)
        python_env = self._find_idf_python_env(tools_path)
        python_bin = None
        if python_env:
            env["IDF_PYTHON_ENV_PATH"] = str(python_env)
            python_bin = self._python_from_env(python_env)
            if python_bin:
                path_entries = env.get("PATH", "").split(os.pathsep)
                python_root = str(python_bin.parent)
                if python_root not in path_entries:
                    env["PATH"] = os.pathsep.join([python_root, *path_entries])
        return env, python_bin

    def _prepare_tools(self, repo_path: Path, env: dict[str, str], python: Path | None) -> Iterable[str]:
        idf_tools = repo_path / "tools" / "idf_tools.py"
        if not idf_tools.exists():
            return
        python_exec = python or Path(sys.executable)
        export_error = self._export_idf_env(repo_path, env, python_exec, idf_tools)
        if export_error:
            yield export_error
        if shutil.which("cmake", path=env.get("PATH")):
            return
        yield "ESP-IDF tools missing. Installing required tools..."
        yield from self._run_process([str(python_exec), str(idf_tools), "install"], cwd=repo_path, env=env)
        export_error = self._export_idf_env(repo_path, env, python_exec, idf_tools)
        if export_error:
            yield export_error

    @staticmethod
    def _export_idf_env(
        repo_path: Path, env: dict[str, str], python_exec: Path, idf_tools: Path
    ) -> str | None:
        original_path = env.get("PATH", "")
        process = subprocess.run(
            [str(python_exec), str(idf_tools), "export", "--format=key-value"],
            cwd=repo_path,
            env=env,
            text=True,
            capture_output=True,
        )
        output = (process.stdout or "").splitlines()
        for line in output:
            if "=" not in line:
                continue
            key, value = line.split("=", 1)
            env[key] = value
        git_path = shutil.which("git", path=original_path)
        if git_path:
            git_dir = str(Path(git_path).parent)
            path_entries = env.get("PATH", "").split(os.pathsep)
            if git_dir not in path_entries:
                env["PATH"] = os.pathsep.join([git_dir, *path_entries])
        env.pop("XTENSA_GNU_CONFIG", None)
        if process.returncode != 0:
            error_text = (process.stderr or "").strip()
            return error_text or "Failed to export ESP-IDF environment."
        return None

    def _ensure_submodules(self, repo_path: Path, env: dict[str, str] | None = None) -> Iterable[str]:
        if not shutil.which("git"):
            yield "git not found on PATH. Unable to update ESP-IDF submodules."
            return
        git_env = (env or os.environ).copy()
        git_env.setdefault("GIT_TERMINAL_PROMPT", "0")
        git_env.setdefault("GIT_ASKPASS", "echo")
        git_env.setdefault("GIT_TRACE", "1")
        git_env.setdefault("GIT_CURL_VERBOSE", "1")
        command = ["git", "submodule", "update", "--init", "--recursive"]
        for attempt in range(1, 3):
            if attempt == 1:
                yield "Ensuring ESP-IDF submodules are available..."
            else:
                yield "Retrying ESP-IDF submodule fetch..."
            yield f"Running: {' '.join(command)}"
            yield from self._run_process(command, cwd=repo_path, env=git_env)

    def _find_idf_python_env(self, tools_path: Path) -> Path | None:
        python_env_root = tools_path / "python_env"
        if not python_env_root.exists():
            return None
        candidates = [path for path in python_env_root.iterdir() if path.is_dir()]
        if not candidates:
            return None
        candidates.sort(key=lambda path: path.stat().st_mtime)
        return candidates[-1]

    @staticmethod
    def _python_from_env(python_env: Path) -> Path | None:
        if sys.platform.startswith("win"):
            python_bin = python_env / "Scripts" / "python.exe"
        else:
            python_bin = python_env / "bin" / "python"
        return python_bin if python_bin.exists() else None
