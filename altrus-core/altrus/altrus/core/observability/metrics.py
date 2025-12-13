from prometheus_client import Gauge, Counter, CollectorRegistry

# 🔒 Single isolated registry (CLI-safe, restart-safe)
REGISTRY = CollectorRegistry(auto_describe=True)

# =============================
# MODULE METRICS
# =============================
modules_total = Gauge(
    "altrus_modules_total",
    "Total registered modules",
    registry=REGISTRY,
)

modules_active = Gauge(
    "altrus_modules_active",
    "Number of active modules",
    registry=REGISTRY,
)

modules_failed = Gauge(
    "altrus_modules_failed",
    "Number of failed modules",
    registry=REGISTRY,
)

# =============================
# INTENT METRICS
# =============================

# Current number of intents in system
intents_total = Gauge(
    "altrus_intents_total",
    "Current number of intents in system",
    registry=REGISTRY,
)

# Monotonic counter
intents_created_total = Counter(
    "altrus_intents_created_total",
    "Total number of intents created",
    registry=REGISTRY,
)

# 1 if intent executing, else 0
intent_executing = Gauge(
    "altrus_intent_executing",
    "Whether an intent is currently executing",
    registry=REGISTRY,
)
