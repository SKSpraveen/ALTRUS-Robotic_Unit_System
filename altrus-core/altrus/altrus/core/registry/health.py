import time
from altrus.core.registry.module import ModuleHealth, ModuleState


HEARTBEAT_TIMEOUT_SEC = 5
DEGRADED_THRESHOLD_SEC = 30
GRACE_PERIOD_SECONDS = 10



class HealthEvaluator:
    @staticmethod
    def evaluate(module):
        now = time.time()

        # 🔒 1. Activation grace period (CRITICAL FIX)
        if (
            module.activation_timestamp is not None
            and now - module.activation_timestamp < GRACE_PERIOD_SECONDS
        ):
            return ModuleHealth.HEALTHY, ModuleState.ACTIVE

        # 🔍 2. NORMAL heartbeat-based evaluation
        delta = now - module.last_heartbeat

        if delta <= HEARTBEAT_TIMEOUT_SEC:
            return ModuleHealth.HEALTHY, ModuleState.ACTIVE

        if delta <= DEGRADED_THRESHOLD_SEC:
            return ModuleHealth.UNSTABLE, ModuleState.DEGRADED

        return ModuleHealth.CRITICAL, ModuleState.FAILED
