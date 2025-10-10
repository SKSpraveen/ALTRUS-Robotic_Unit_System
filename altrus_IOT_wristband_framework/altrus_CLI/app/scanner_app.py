from __future__ import annotations

from pathlib import Path

from altrus_cli.domain.scanner_service import ScannerService
from altrus_cli.infrastructure.config_yaml import YamlConfigReader
from altrus_cli.infrastructure.formatter import ConsolePayloadFormatter
from altrus_cli.infrastructure.predictors import ProjectPredictor
from altrus_cli.infrastructure.receivers import TcpReceiver, UdpReceiver


class ScannerApp:
    """Composition root for the live scanner."""

    def __init__(self, project_root: Path, protocol: str, host: str, port: int) -> None:
        self._project_root = project_root
        self._protocol = protocol
        self._host = host
        self._port = port

    def run(self) -> None:
        config_reader = YamlConfigReader(
            self._project_root / "config" / "wristband_config.yaml"
        )
        receiver = self._build_receiver()
        predictor = ProjectPredictor(self._project_root)
        formatter = ConsolePayloadFormatter()
        service = ScannerService(config_reader, receiver, predictor, formatter)
        service.run()

    def _build_receiver(self):
        if self._protocol == "tcp":
            return TcpReceiver(self._host, self._port)
        return UdpReceiver(self._host, self._port)
