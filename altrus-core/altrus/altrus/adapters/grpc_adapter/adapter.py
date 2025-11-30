from altrus.adapters.base.adapter import Adapter
from altrus.core.intent.intent import Intent, IntentPriority


class GRPCAdapter(Adapter):
    """
    Translates gRPC messages into intents.
    """

    def translate(self, grpc_message) -> Intent:
        return Intent(
            intent_id=None,
            capability=grpc_message.capability,
            priority=IntentPriority.NORMAL
        )
