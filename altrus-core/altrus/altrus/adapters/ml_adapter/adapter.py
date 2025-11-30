from altrus.adapters.base.adapter import Adapter
from altrus.core.intent.intent import Intent, IntentPriority


class MLAdapter(Adapter):
    """
    Translates ML model outputs into intents.
    """

    def translate(self, prediction) -> Intent:
        """
        prediction example:
        {
            "intent": "NAVIGATE",
            "confidence": 0.92
        }
        """
        priority = (
            IntentPriority.HIGH
            if prediction.get("confidence", 0) > 0.8
            else IntentPriority.NORMAL
        )

        return Intent(
            intent_id=None,
            capability=prediction["intent"],
            priority=priority
        )
