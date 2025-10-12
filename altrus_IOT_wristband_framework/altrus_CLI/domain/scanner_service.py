from __future__ import annotations

import json
from dataclasses import dataclass

from altrus_cli.ports.interfaces import ConfigReader, PayloadFormatter, PayloadReceiver, Predictor


@dataclass
class ScannerService:
    """Coordinates payload ingestion, inference, and formatting."""

    config_reader: ConfigReader
    receiver: PayloadReceiver
    predictor: Predictor
    formatter: PayloadFormatter

    def run(self) -> None:
        config = self.config_reader.load()
        for raw in self.receiver.iter_messages():
            payload = self._parse_payload(raw)
            if payload is None:
                continue
            prediction = self.predictor.predict(payload, config.activities, config.anomalies)
            output = self.formatter.format(payload, prediction)
            if output:
                print(output)

    @staticmethod
    def _parse_payload(raw: bytes) -> dict | None:
        try:
            payload = json.loads(raw.decode("utf-8"))
        except json.JSONDecodeError:
            return None
        if not isinstance(payload, dict):
            return None
        return payload
