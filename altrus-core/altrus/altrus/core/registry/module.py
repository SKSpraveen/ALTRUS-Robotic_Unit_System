# core/registry/module.py
from enum import Enum
from dataclasses import dataclass, field, asdict
from typing import List, Dict
import time
import json

class ModuleState(Enum):
    REGISTERED = "REGISTERED"
    ACTIVE = "ACTIVE"
    DEGRADED = "DEGRADED"
    FAILED = "FAILED"
    OFFLINE = "OFFLINE"

    # Add this method to make enums JSON-serializable
    def __str__(self):
        return self.value

class ModuleHealth(Enum):
    HEALTHY = "HEALTHY"
    UNSTABLE = "UNSTABLE"
    CRITICAL = "CRITICAL"
    UNKNOWN = "UNKNOWN"

    def __str__(self):
        return self.value

@dataclass
class Module:
    module_id: str
    name: str
    version: str
    capabilities: List[str]

    state: ModuleState = ModuleState.REGISTERED
    health: ModuleHealth = ModuleHealth.UNKNOWN

    last_heartbeat: float = field(default_factory=time.time)
    metadata: Dict = field(default_factory=dict)

    activation_timestamp: float | None = None


    # Add this serialization method
    def to_dict(self) -> dict:
        """Convert to JSON-serializable dictionary"""
        data = asdict(self)
        # Convert enums to their string values
        data['state'] = self.state.value
        data['health'] = self.health.value
        return data

    @classmethod
    def from_dict(cls, data: dict) -> "Module":
        """Create Module from dictionary"""
        # Convert string values back to enums
        data['state'] = ModuleState(data['state'])
        data['health'] = ModuleHealth(data['health'])
        return cls(**data)