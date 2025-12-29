from pathlib import Path

from altrus.core.registry.registry import ModuleRegistry
from altrus.core.intent.intent_engine import IntentEngine
from altrus.core.fault.fault_engine import FaultEngine
from altrus.core.fault.recovery import RestartModuleStrategy
from altrus.core.fault.detectors import HeartbeatDetector
from altrus.core.ledger.blockchain import BlockchainLedger
from altrus.core.registry.health_monitor import HealthMonitor
from altrus.core.observability.server import start_metrics_server
from altrus.core.intent.registry import IntentDefinitionRegistry

from altrus.core.intent.policies.default_policy import DefaultRoutingPolicy
from altrus.core.intent.policies.config_loader import PolicyConfigLoader


class AltrusKernel:
    """
    Middleware kernel responsible for wiring all core subsystems.
    """

    def __init__(self):
        # -----------------------------
        # Observability
        # -----------------------------
        start_metrics_server(8000)

        # -----------------------------
        # Load Policy Config
        # -----------------------------
        BASE_DIR = Path(__file__).resolve().parent

        policy_path = (
            BASE_DIR
            / "intent"
            / "policies"
            / "intent_routing.yaml"
        )
        policy_config = PolicyConfigLoader.load(policy_path)

        routing_policy = DefaultRoutingPolicy(
            config=policy_config.get("routing", {})
        )

        preemption_config = policy_config.get("preemption", {})

        # -----------------------------
        # Ledger (audit backbone)
        # -----------------------------
        self.ledger = BlockchainLedger()

        # -----------------------------
        # Registry (system truth)
        # -----------------------------
        self.registry = ModuleRegistry(ledger=self.ledger)

        # -----------------------------
        # Intent Engine
        # -----------------------------
        self.intent_engine = IntentEngine(
            registry=self.registry,
            ledger=self.ledger,
            policy=routing_policy,
            preemption_config=preemption_config,
        )
        self.registry._intent_engine = self.intent_engine


        # -----------------------------
        # Fault Engine
        # -----------------------------
        heartbeat_detector = HeartbeatDetector()

        self.fault_engine = FaultEngine(
            registry=self.registry,
            intent_engine=self.intent_engine,
            detectors=[heartbeat_detector],
            recovery_strategy=RestartModuleStrategy(),
        )


        # -----------------------------
        # Health Monitoring
        # -----------------------------
        self.health_monitor = HealthMonitor(self.registry, fault_engine=self.fault_engine)
        self.health_monitor.start()

        # -----------------------------
        # Intent Definitions
        # -----------------------------
        self.intent_definitions = IntentDefinitionRegistry(
            path=Path("intents.yaml")
        )
