from dataclasses import dataclass
from typing import Dict, Any


@dataclass
class IntentDefinition:
    name: str
    priority: int
    timeout: int
    ttl: int
    preemptible: bool
    route: Dict[str, Any]
