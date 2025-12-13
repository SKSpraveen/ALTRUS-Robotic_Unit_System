from typing import Callable, Dict, List, Any


class EventBus:
    """
    Simple in-process publish/subscribe event bus.
    """

    def __init__(self):
        self._subscribers: Dict[str, List[Callable]] = {}

    def subscribe(self, event_type: str, handler: Callable):
        self._subscribers.setdefault(event_type, []).append(handler)

    def publish(self, event_type: str, data: Any):
        for handler in self._subscribers.get(event_type, []):
            handler(data)
