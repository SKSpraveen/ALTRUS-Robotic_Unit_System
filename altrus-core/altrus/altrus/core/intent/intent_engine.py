from typing import List, Optional

from altrus.core.intent.intent import Intent, IntentState
from altrus.core.intent.policies.default_policy import DefaultRoutingPolicy
from altrus.core.intent.storage import IntentStorage
from altrus.core.observability.metrics import (
    intents_total,
    intents_created_total,
    intent_executing,
)




class IntentEngine:
    """
    Authoritative runtime intent execution engine.
    Manages intent lifecycle transitions.
    """

    def __init__(
        self,
        registry,
        ledger,
        policy: Optional[DefaultRoutingPolicy] = None,
        preemption_config: dict | None = None,
    ):
        self.registry = registry
        self.ledger = ledger

        self.policy = policy or DefaultRoutingPolicy()
        self.preemption_config = preemption_config or {}

        # Persistent storage
        self.storage = IntentStorage()

        # Load persisted intents
        loaded_intents = self.storage.load()
        self.intent_store: dict[str, Intent] = {
            i.intent_id: i for i in loaded_intents
        }

        # Restore currently executing intent (if any)
        self.current_intent: Optional[Intent] = next(
            (i for i in self.intent_store.values()
            if i.state == IntentState.EXECUTING),
            None
        )


    # -------------------------------
    # Public API
    # -------------------------------

    def submit_intent(self, intent: Intent):
        """
        Register intent.
        Validation already happened inside Intent.__init__().
        """
        intent.validate()
        intent.state = IntentState.VALIDATED

        # 🔁 Reject if same capability already executing
        if (
            self.current_intent
            and self.current_intent.capability == intent.capability
        ):
            intent.state = IntentState.REJECTED

            self.ledger.log_event(
                event_type="INTENT_REJECTED",
                actor_type="INTENT",
                actor_id=intent.intent_id,
                data={"reason": "DUPLICATE_CAPABILITY"},
            )

            self.intent_store[intent.intent_id] = intent
            self.storage.save(self.intent_store.values())
            return

        # Otherwise, accept the intent
        self.intent_store[intent.intent_id] = intent
        self.storage.save(self.intent_store.values())

         # ✅ METRICS
        intents_created_total.inc()
        intents_total.set(len(self.intent_store))


        self.ledger.log_event(
            event_type="INTENT_SUBMITTED",
            actor_type="INTENT",
            actor_id=intent.intent_id,
            data={
                "name": intent.name,
                "capability": intent.capability,
                "priority": intent.priority.name,
                "state": intent.state.value,
            },
        )

    def list_intents(self) -> List[Intent]:
        return list(self.intent_store.values())

    # -------------------------------
    # Fault Interaction
    # -------------------------------

    def on_module_failed(self, module_id: str):
        if not self.current_intent:
            return

        self.current_intent.mark_preempted()

        self.ledger.log_event(
            event_type="INTENT_PREEMPTED",
            actor_type="INTENT",
            actor_id=self.current_intent.intent_id,
            data={
                "reason": "MODULE_FAILURE",
                "module_id": module_id,
            },
        )

        self.storage.save(self.intent_store.values())
        self.current_intent = None

    # -------------------------------
    # Execution Logic
    # -------------------------------

    def route_intent(self, intent: Intent):
        candidates = self.registry.find_by_capability(intent.capability)
        module = self.policy.select_module(intent, candidates)

        if not module:
            return None

        self.ledger.log_event(
            event_type="INTENT_ROUTED",
            actor_type="INTENT",
            actor_id=intent.intent_id,
            data={
                "capability": intent.capability,
                "selected_module": module.module_id,
                "policy": self.policy.__class__.__name__,
            },
        )

        return module

    def should_preempt(self, new_intent: Intent) -> bool:
        if not self.preemption_config.get("enabled", True):
            return False

        min_priority = self.preemption_config.get("min_priority", "HIGH")

        if not self.current_intent:
            return True

        return (
            new_intent.priority.name >= min_priority
            and new_intent.priority > self.current_intent.priority
        )

    def execute_pending_intents(self):
        """
        Promote highest-priority VALIDATED intent to EXECUTING
        if no intent is currently executing and a compatible module exists.
        """

        # ❗ Prevent multiple EXECUTING intents (CLI restarts safe)
        for i in self.intent_store.values():
            if i.state == IntentState.EXECUTING:
                self.current_intent = i
                return

        # ❗ HARD GUARD (redundant but safe)
        if self.current_intent:
            return

        pending = [
            i for i in self.intent_store.values()
            if i.state == IntentState.VALIDATED
        ]

        if not pending:
            return

        pending.sort(key=lambda i: i.priority, reverse=True)
        intent = pending[0]

        candidates = self.registry.find_by_capability(intent.capability)
        if not candidates:
            return

        module = candidates[0]

        intent.mark_executing()
        self.current_intent = intent

        self.ledger.log_event(
            event_type="INTENT_EXECUTING",
            actor_type="INTENT",
            actor_id=intent.intent_id,
            data={
                "name": intent.name,
                "capability": intent.capability,
                "priority": intent.priority.name,
                "module": module.module_id,
            },
        )

        self.storage.save(self.intent_store.values())

        intent_executing.set(1 if self.current_intent else 0)



    def handle_intent(self, intent: Intent):
            """
            Authoritative intent handling pipeline.
            """

            # 1️⃣ Validate
            if not intent.validate():
                intent.state = IntentState.REJECTED
                self.intent_store[intent.intent_id] = intent
                self.storage.save(self.intent_store.values())
                return

            # 2️⃣ Preemption decision
            if self.current_intent and not self.should_preempt(intent):
                intent.state = IntentState.REJECTED
                self.intent_store[intent.intent_id] = intent

                self.ledger.log_event(
                    event_type="INTENT_REJECTED",
                    actor_type="INTENT",
                    actor_id=intent.intent_id,
                    data={
                        "reason": "LOWER_PRIORITY_THAN_CURRENT",
                        "current_intent": self.current_intent.intent_id,
                    },
                )

                # Persist rejection
                self.storage.save(self.intent_store.values())
                return

            # 3️⃣ Preempt current intent if exists
            if self.current_intent:
                self.current_intent.mark_preempted()

                self.ledger.log_event(
                    event_type="INTENT_PREEMPTED",
                    actor_type="INTENT",
                    actor_id=self.current_intent.intent_id,
                    data={
                        "preempted_by": intent.intent_id,
                    },
                )

            # 4️⃣ Route intent
            module = self.route_intent(intent)
            if not module:
                intent.state = IntentState.REJECTED
                self.intent_store[intent.intent_id] = intent
                self.storage.save(self.intent_store.values())
                return

            # 5️⃣ Activate intent
            intent.mark_executing()
            self.current_intent = intent
            self.intent_store[intent.intent_id] = intent

            self.ledger.log_event(
                event_type="INTENT_EXECUTING",
                actor_type="INTENT",
                actor_id=intent.intent_id,
                data={
                    "name": intent.name,
                    "capability": intent.capability,
                    "priority": intent.priority.name,
                    "module": module.module_id,
                },
            )

            # Persist execution state
            self.storage.save(self.intent_store.values())
