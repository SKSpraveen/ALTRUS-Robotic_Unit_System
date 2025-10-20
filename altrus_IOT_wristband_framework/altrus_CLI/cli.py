from __future__ import annotations

import argparse
from pathlib import Path

from altrus_cli.generator import ProjectConfig, create_project
from altrus_cli.runtime import run_scanner

PREDEFINED_SENSORS = [
    "accelerometer",
    "gyroscope",
    "heart_rate",
    "body_temperature",
    "fall_detection_sensor",
    "ppg",
    "ecg",
    "spo2",
    "eda",
    "barometer",
]

PREDEFINED_ANOMALIES = [
    "tachycardia",
    "bradycardia",
    "fever",
    "heart_attack",
    "cardiac_arrest",
    "arrhythmia",
    "fall_detection",
    "stress_spike",
    "overexertion",
    "sleep_disruption",
]

PREDEFINED_ACTIVITIES = [
    "sleep",
    "rest",
    "walk",
    "run",
]

MODEL_CHOICES = [
    "default",
    "custom",
]

ENVIRONMENTS = [
    "raspberry_pi",
    "linux",
    "windows",
]


def _prompt_text(prompt: str, default: str | None = None) -> str:
    suffix = f" [{default}]" if default else ""
    while True:
        value = input(f"{prompt}{suffix}: ").strip()
        if not value and default:
            return default
        if value:
            return value
        print("Please enter a value.")


def _prompt_multi_select(title: str, options: list[str]) -> list[str]:
    print(f"\n{title}")
    for index, option in enumerate(options, start=1):
        print(f"  {index}. {option}")
    print("  c. Add custom entries")
    print("  a. Select all")

    selections: list[str] = []
    while not selections:
        raw = input("Enter selection numbers (comma-separated): ").strip().lower()
        if raw == "a":
            return options.copy()

        entries = [entry.strip() for entry in raw.split(",") if entry.strip()]
        custom_requested = "c" in entries
        selected_indices = [entry for entry in entries if entry.isdigit()]

        for entry in selected_indices:
            idx = int(entry)
            if 1 <= idx <= len(options):
                selections.append(options[idx - 1])

        if custom_requested:
            custom_raw = _prompt_text("Enter custom values (comma-separated)")
            custom_values = [value.strip() for value in custom_raw.split(",") if value.strip()]
            selections.extend(custom_values)

        selections = list(dict.fromkeys(selections))
        if not selections:
            print("Please make at least one selection.")

    return selections


def _prompt_choice(title: str, options: list[str]) -> str:
    print(f"\n{title}")
    for index, option in enumerate(options, start=1):
        print(f"  {index}. {option}")
    while True:
        raw = input("Enter selection number: ").strip()
        if raw.isdigit():
            idx = int(raw)
            if 1 <= idx <= len(options):
                return options[idx - 1]
        print("Please enter a valid selection.")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate a wristband sensor fusion project scaffold.",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    init_parser = subparsers.add_parser("init", help="Create a new project scaffold")
    init_parser.add_argument("--name", help="Project name")
    init_parser.add_argument(
        "--output",
        help="Directory to create the project in",
        default=str(Path.cwd()),
    )

    run_parser = subparsers.add_parser("run", help="Run the live scanner in a project")
    run_parser.add_argument(
        "--protocol",
        choices=["udp", "tcp"],
        default="udp",
        help="Network protocol for incoming sensor data",
    )
    run_parser.add_argument(
        "--host",
        default="0.0.0.0",
        help="Host/IP to bind for incoming sensor data",
    )
    run_parser.add_argument(
        "--port",
        type=int,
        default=5055,
        help="Port to bind for incoming sensor data",
    )
    run_parser.add_argument(
        "--interval",
        type=float,
        default=0.5,
        help="Minimum seconds between terminal updates",
    )

    return parser.parse_args()


def _run_init(args: argparse.Namespace) -> None:
    name = args.name or _prompt_text("Project name", default="wristband_project")
    output_dir = Path(args.output).expanduser().resolve()

    sensors = _prompt_multi_select("Select wristband sensors", PREDEFINED_SENSORS)
    anomalies = _prompt_multi_select("Select anomalies to detect", PREDEFINED_ANOMALIES)
    activities = _prompt_multi_select("Select activities to classify", PREDEFINED_ACTIVITIES)
    environments = _prompt_multi_select("Select target environments", ENVIRONMENTS)
    model_choice = _prompt_choice("Select model setup", MODEL_CHOICES)

    config = ProjectConfig(
        name=name,
        sensors=sensors,
        anomalies=anomalies,
        activities=activities,
        environments=environments,
        model_choice=model_choice,
    )

    project_path = create_project(config, output_dir)
    print(f"\nProject created at: {project_path}")


def main() -> None:
    args = _parse_args()
    if args.command == "init":
        _run_init(args)
    if args.command == "run":
        project_root = Path.cwd()
        print(
            "Starting live scanner. "
            "Send JSON sensor payloads over the selected protocol."
        )
        run_scanner(
            project_root=project_root,
            protocol=args.protocol,
            host=args.host,
            port=args.port,
            output_interval=args.interval,
        )


if __name__ == "__main__":
    main()
