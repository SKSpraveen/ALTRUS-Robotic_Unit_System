from altrus.core.registry.module import ModuleHealth, ModuleState
from .base import RoutingPolicy


class DefaultRoutingPolicy(RoutingPolicy):
    def __init__(self, config: dict | None = None):
        self.config = config or {}

    def select_module(self, intent, candidate_modules):
        if not candidate_modules:
            return None

        prefer_health = self.config.get("prefer_health", "HEALTHY")
        allow_degraded = self.config.get("allow_degraded", False)
        deterministic = self.config.get("deterministic", True)

        filtered = [
            m for m in candidate_modules
            if m.health.name == prefer_health
            and (allow_degraded or m.state == ModuleState.ACTIVE)
        ]

        pool = filtered if filtered else candidate_modules

        if deterministic:
            pool.sort(key=lambda m: m.module_id)

        return pool[0]
