from typing import List
import time

from .module import Module, ModuleState, ModuleHealth
from .exceptions import ModuleNotFoundError
from altrus.core.registry.storage import RegistryStorage
from altrus.core.registry.health import HealthEvaluator
from altrus.core.observability.metrics import (
    modules_total,
    modules_active,
    modules_failed,
)


class ModuleRegistry:
    def __init__(self, ledger=None, storage=None):
        self._ledger = ledger
        self._storage = storage or RegistryStorage()
        self._modules = {
            m.module_id: m
            for m in self._storage.load()
        }

    # -----------------------------
    # Basic CRUD
    # -----------------------------

    def register(self, module: Module):
        self._modules[module.module_id] = module
        self._storage.save(self.list_modules())

    def unregister(self, module_id: str):
        if module_id not in self._modules:
            raise ModuleNotFoundError(module_id)

        del self._modules[module_id]
        self._storage.save(self.list_modules())

    def get(self, module_id: str) -> Module:
        if module_id not in self._modules:
            raise ModuleNotFoundError(module_id)
        return self._modules[module_id]

    def list_modules(self) -> List[Module]:
        return list(self._modules.values())

    # -----------------------------
    # Authoritative State Mutators
    # -----------------------------

    def update_state(self, module_id: str, state: ModuleState):
        module = self.get(module_id)
        prev = module.state

        if prev == state:
            return

        module.state = state
        self._storage.save(self.list_modules())

        if self._ledger:
            self._ledger.log_event(
                event_type="MODULE_STATE_CHANGE",
                actor_type="MODULE",
                actor_id=module_id,
                data={
                    "previous": prev.value,
                    "new": state.value,
                },
            )

    def update_health(self, module_id: str, health: ModuleHealth):
        module = self.get(module_id)
        prev = module.health

        if prev == health:
            return

        module.health = health
        self._storage.save(self.list_modules())

        if self._ledger:
            self._ledger.log_event(
                event_type="MODULE_HEALTH_CHANGE",
                actor_type="MODULE",
                actor_id=module_id,
                data={
                    "previous": prev.value,
                    "new": health.value,
                },
            )

    # -----------------------------
    # Capability Lookup
    # -----------------------------

    def find_by_capability(self, capability: str) -> List[Module]:
        return [
            m for m in self._modules.values()
            if capability in m.capabilities
            and m.state == ModuleState.ACTIVE
        ]

    # -----------------------------
    # Heartbeat (CRITICAL FIX)
    # -----------------------------

    def heartbeat(self, module_id: str):
        module = self.get(module_id)
        module.last_heartbeat = time.time()

        self._storage.save(self.list_modules())

        if self._ledger:
            self._ledger.log_event(
                event_type="MODULE_HEARTBEAT",
                actor_type="MODULE",
                actor_id=module_id,
                data={
                    "timestamp": module.last_heartbeat
                },
            )


    # -----------------------------
    # Health Evaluation (FINAL FIX)
    # -----------------------------

    def evaluate_health(self):
        active = 0
        failed = 0

        for module in self._modules.values():

            if module.state in {ModuleState.OFFLINE, ModuleState.FAILED}:
                continue

            prev_health = module.health
            prev_state = module.state

            health, state = HealthEvaluator.evaluate(module)

            if prev_health != health or prev_state != state:
                module.health = health
                module.state = state

                self._storage.save(self.list_modules())

                if self._ledger:
                    self._ledger.log_event(
                        event_type="MODULE_HEALTH_CHANGE",
                        actor_type="MODULE",
                        actor_id=module.module_id,
                        data={
                            "prev_health": prev_health.value,
                            "new_health": health.value,
                            "prev_state": prev_state.value,
                            "new_state": state.value,
                        },
                    )

                if state == ModuleState.FAILED and hasattr(self, "_intent_engine"):
                    self._intent_engine.on_module_failed(module.module_id)

            # Count states
            if module.state == ModuleState.ACTIVE:
                active += 1
            if module.state == ModuleState.FAILED:
                failed += 1

        # ✅ METRICS UPDATED ONCE
        modules_total.set(len(self._modules))
        modules_active.set(active)
        modules_failed.set(failed)




    # -----------------------------
    # Activation
    # -----------------------------

    def activate(self, module_id: str):
        module = self.get(module_id)

        module.activation_timestamp = time.time()
        module.last_heartbeat = time.time()

        self.update_state(module_id, ModuleState.ACTIVE)
        self.update_health(module_id, ModuleHealth.HEALTHY)
