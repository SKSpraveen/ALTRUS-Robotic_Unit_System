from __future__ import annotations

from pathlib import Path

from altrus_cli.app.scanner_app import ScannerApp


def run_scanner(
    project_root: Path,
    protocol: str,
    host: str,
    port: int,
    output_interval: float,
) -> None:
    """Run the live scanner with structured dependencies."""
    del output_interval
    print(
        f"Listening for {protocol.upper()} sensor data on {host}:{port}. "
        "Press Ctrl+C to stop."
    )
    app = ScannerApp(project_root, protocol, host, port)
    try:
        app.run()
    except KeyboardInterrupt:
        print("\nScanner stopped.")
