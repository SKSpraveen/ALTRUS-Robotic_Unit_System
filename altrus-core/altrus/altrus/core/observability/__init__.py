from prometheus_client import Counter, Gauge

# Intent metrics
INTENTS_TOTAL = Counter(
    "altrus_intents_total",
    "Total number of intents received"
)

INTENTS_PREEMPTED = Counter(
    "altrus_intents_preempted_total",
    "Total number of intents preempted"
)

# Fault metrics
FAULTS_TOTAL = Counter(
    "altrus_faults_total",
    "Total number of faults detected",
    ["severity"]
)

RECOVERY_ATTEMPTS = Counter(
    "altrus_recovery_attempts_total",
    "Total number of recovery attempts"
)

# Module metrics
MODULES_ACTIVE = Gauge(
    "altrus_modules_active",
    "Number of active modules"
)

MODULES_DEGRADED = Gauge(
    "altrus_modules_degraded",
    "Number of degraded modules"
)

# Ledger metrics
LEDGER_BLOCKS = Gauge(
    "altrus_ledger_blocks",
    "Total number of blocks in ledger"
)
