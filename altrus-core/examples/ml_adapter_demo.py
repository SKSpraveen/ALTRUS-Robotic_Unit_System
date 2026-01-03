from altrus.core.intent.intent import Intent, IntentPriority
from altrus.core.kernel import AltrusKernel
import uuid

kernel = AltrusKernel()

# Simulated ML output
intent = Intent(
    intent_id=str(uuid.uuid4()),
    name="NAVIGATE",
    capability="navigation.move",
    priority=IntentPriority.HIGH
)

kernel.intent_engine.submit_intent(intent)

print("ML Adapter injected intent:", intent.name)
