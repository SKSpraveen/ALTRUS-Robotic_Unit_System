import hashlib
import json
import time
from typing import Any, Dict


class Block:
    """
    Represents a single immutable block in the ledger.
    """

    def __init__(
        self,
        index: int,
        previous_hash: str,
        data: Dict[str, Any],
        timestamp: float = None
    ):
        self.index = index
        self.previous_hash = previous_hash
        self.timestamp = timestamp or time.time()
        self.data = data
        self.hash = self.compute_hash()

    def compute_hash(self) -> str:
        block_string = json.dumps({
            "index": self.index,
            "previous_hash": self.previous_hash,
            "timestamp": self.timestamp,
            "data": self.data
        }, sort_keys=True)

        return hashlib.sha256(block_string.encode()).hexdigest()
