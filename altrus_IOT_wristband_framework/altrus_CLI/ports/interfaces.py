from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Protocol


@dataclass(frozen=True)
class RuntimeConfig:
    activities: list[str]
    anomalies: list[str]


class PayloadReceiver(Protocol):
    """Receives raw payload bytes from a transport."""

    def iter_messages(self) -> Iterable[bytes]:
        ...


class Predictor(Protocol):
    """Runs inference on a payload."""

    def predict(self, payload: dict, activities: list[str], anomalies: list[str]) -> dict:
        ...


class ConfigReader(Protocol):
    """Loads runtime configuration for activities/anomalies."""

    def load(self) -> RuntimeConfig:
        ...


class PayloadFormatter(Protocol):
    """Formats payload + prediction for display."""

    def format(self, payload: dict, prediction: dict) -> str:
        ...
