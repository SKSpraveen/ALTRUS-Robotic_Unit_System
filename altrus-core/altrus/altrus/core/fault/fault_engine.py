from typing import List
from .state_machine import FaultState, FaultSeverity
from .detectors import FaultDetector
from .recovery import RecoveryStrategy
from altrus.core.registry.module import Module, ModuleState, ModuleHealth

class FaultEngine:
    """
    Severity-aware fault detection and escalation engine.
    """

    def __init__(
        self,
        registry,
        intent_engine,
        detectors,
        recovery_strategy,
        max_retries=3
    ):
        self.registry = registry
        self.intent_engine = intent_engine
        self.detectors = detectors
        self.recovery_strategy = recovery_strategy
        self.state = FaultState.NORMAL
        self.retry_count = 0
        self.max_retries = max_retries

    def evaluate(self):
        detected = [d for d in self.detectors if d.check()]

        if not detected:
            self.state = FaultState.NORMAL
            self.retry_count = 0
            return self.state

        highest = max(detected, key=lambda d: d.severity().value)

        if highest.severity() == FaultSeverity.CRITICAL:
            self.state = FaultState.FAILED

            # 🔥 HARD FAIL ALL ACTIVE MODULES
            for module in self.registry.list_modules():
                if module.state == ModuleState.ACTIVE:
                    self.registry.update_state(module.module_id, ModuleState.FAILED)
                    self.registry.update_health(module.module_id, ModuleHealth.CRITICAL)

                    # 🔥 PREEMPT INTENT
                    self.intent_engine.on_module_failed(module.module_id)

            return self.state

        # recovery path
        if self.retry_count < self.max_retries:
            self.state = FaultState.RECOVERING
            self.recovery_strategy.execute()
            self.retry_count += 1
        else:
            self.state = FaultState.FAILED

        return self.state

