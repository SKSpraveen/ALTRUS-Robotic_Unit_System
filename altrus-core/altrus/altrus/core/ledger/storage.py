import json
from pathlib import Path
from typing import List
from .block import Block


class LedgerStorage:
    def __init__(self, path: Path | None = None):
        base_dir = Path.home() / ".altrus"
        base_dir.mkdir(exist_ok=True)
        self.path = path or (base_dir / "ledger.json")

    def load(self) -> List[Block]:
        if not self.path.exists():
            return []

        with self.path.open("r") as f:
            raw = json.load(f)

        return [
            Block(
                index=b["index"],
                previous_hash=b["previous_hash"],
                data=b["data"],
                timestamp=b["timestamp"],
            )
            for b in raw
        ]

    def save(self, chain: List[Block]):
        with self.path.open("w") as f:
            json.dump(
                [
                    {
                        "index": b.index,
                        "previous_hash": b.previous_hash,
                        "timestamp": b.timestamp,
                        "data": b.data,
                        "hash": b.hash,
                    }
                    for b in chain
                ],
                f,
                indent=2,
            )
