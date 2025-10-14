from typing import List
from .intent import Intent
from .interfaces import IntentResolver


class PriorityBasedResolver(IntentResolver):
    """
    Resolves intents based on priority and timestamp.
    """

    def resolve(self, intents: List[Intent]) -> Intent:
        if not intents:
            raise ValueError("No intents to resolve")

        # Highest priority wins
        intents.sort(
            key=lambda i: (i.priority.value, i.timestamp),
            reverse=True
        )
        return intents[0]
