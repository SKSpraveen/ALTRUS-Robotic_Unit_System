from __future__ import annotations

from altrus_cli.ports.interfaces import PayloadFormatter


class ConsolePayloadFormatter(PayloadFormatter):
    def format(self, payload: dict, prediction: dict) -> str:
        summary = self._format_payload(payload)
        status = "ANOMALY" if prediction.get("anomaly") else "normal"
        return (
            f"{summary} -> {status} "
            f"activity={prediction.get('activity')} "
            f"type={prediction.get('anomaly_type')} (score={prediction.get('score')})"
        )

    @staticmethod
    def _format_payload(payload: dict) -> str:
        parts = []
        if "heart_rate" in payload:
            parts.append(f"hr={payload['heart_rate']}")
        if "body_temperature" in payload:
            parts.append(f"temp={payload['body_temperature']}")
        if all(key in payload for key in ("accel_x", "accel_y", "accel_z")):
            parts.append(
                "acc=({:.2f},{:.2f},{:.2f})".format(
                    payload["accel_x"], payload["accel_y"], payload["accel_z"]
                )
            )
        if not parts:
            parts.append(f"payload={payload}")
        return " ".join(parts)
