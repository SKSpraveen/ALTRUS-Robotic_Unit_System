from .storage import LedgerStorage
from typing import List, Dict, Any
from .block import Block
import time  # Add this missing import
import hashlib
import json

class BlockchainLedger:
    def __init__(self, storage: LedgerStorage | None = None):
        self._storage = storage or LedgerStorage()
        self.chain = self._storage.load()

        if not self.chain:
            self._create_genesis_block()
            self._persist()

    def _persist(self):
        self._storage.save(self.chain)

    def _create_genesis_block(self):
        genesis_block = Block(
            index=0,
            previous_hash="0",
            data={"event_type": "GENESIS"}
        )
        self.chain.append(genesis_block)

    def add_block(self, data: Dict[str, Any]) -> Block:
        last_block = self.chain[-1]
        block = Block(
            index=len(self.chain),
            previous_hash=last_block.hash,
            data=data
        )
        self.chain.append(block)
        self._persist()
        return block

    def get_chain(self) -> List[Block]:
        return self.chain

    def log_event(self, event_type, actor_type, actor_id, data, cause_id=None):
        event_data = {
            "event_type": event_type,
            "actor_type": actor_type,
            "actor_id": actor_id,
            "timestamp": time.time(),
            "data": data,
            "cause_id": cause_id
        }
        self.add_block(event_data)
