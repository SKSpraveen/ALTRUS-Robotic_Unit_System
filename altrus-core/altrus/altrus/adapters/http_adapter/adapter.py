from altrus.adapters.base.adapter import Adapter
from altrus.core.intent.intent import Intent, IntentPriority


class HTTPAdapter(Adapter):
    """
    Translates HTTP API requests into intents.
    """

    def translate(self, request_json) -> Intent:
        return Intent(
            intent_id=None,
            capability=request_json["capability"],
            priority=IntentPriority[request_json.get("priority", "NORMAL")]
        )
