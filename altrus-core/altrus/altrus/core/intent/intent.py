from dataclasses import dataclass, field
from enum import IntEnum, Enum
import time
from typing import Optional


# -------------------------------
# Priority & State Enums
# -------------------------------

class IntentPriority(IntEnum):
    LOW = 1
    NORMAL = 2
    HIGH = 3
    CRITICAL = 4



class IntentState(Enum):
    CREATED = "CREATED"
    VALIDATED = "VALIDATED"
    EXECUTING = "EXECUTING"
    PREEMPTED = "PREEMPTED"
    REJECTED = "REJECTED"
    EXPIRED = "EXPIRED"


# -------------------------------
# Intent Domain Object
# -------------------------------

@dataclass
class Intent:
    intent_id: str
    name: str
    capability: str
    priority: IntentPriority
    preemptible: bool = True

    created_at: float = field(default_factory=time.time)
    ttl: Optional[int] = None
    state: IntentState = IntentState.CREATED

    def __init__(
        self,
        intent_id: str,
        name: str,
        capability: str,
        priority: IntentPriority = IntentPriority.NORMAL,
        preemptible: bool = True,
        ttl: Optional[int] = None,
        state: IntentState = IntentState.CREATED,
    ):
        self.intent_id = intent_id
        self.name = name
        self.capability = capability
        self.priority = priority
        self.preemptible = preemptible
        self.ttl = ttl
        self.state = state
        self.created_at = time.time()

        self.validate()

    # -------------------------------
    # Lifecycle Helpers
    # -------------------------------

    def validate(self) -> bool:
        if not self.name or not self.capability:
            return False

        if not isinstance(self.priority, IntentPriority):
            return False

        return True


    def mark_executing(self):
        self.state = IntentState.EXECUTING

    def mark_preempted(self):
        self.state = IntentState.PREEMPTED

    def is_expired(self) -> bool:
        if self.ttl is None:
            return False
        return time.time() > self.created_at + self.ttl

    # -------------------------------
    # Persistence (CRITICAL)
    # -------------------------------

    def to_dict(self) -> dict:
        return {
            "intent_id": self.intent_id,
            "name": self.name,
            "capability": self.capability,
            "priority": int(self.priority),
            "state": self.state.value,
            "preemptible": self.preemptible,
            "ttl": self.ttl,
            "created_at": self.created_at,
        }

    @classmethod
    def from_dict(cls, data: dict) -> "Intent":
        intent = cls(
            intent_id=data["intent_id"],
            name=data["name"],
            capability=data["capability"],
            priority=IntentPriority(data["priority"]),
            preemptible=data.get("preemptible", True),
            ttl=data.get("ttl"),
            state=IntentState(data["state"]),
        )
        intent.created_at = data.get("created_at", time.time())
        return intent

