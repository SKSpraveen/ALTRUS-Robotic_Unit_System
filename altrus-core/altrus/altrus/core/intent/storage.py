import json
from pathlib import Path
from typing import List
from altrus.core.intent.intent import Intent, IntentPriority, IntentState


class IntentStorage:
    def __init__(self):
        base_dir = Path.home() / ".altrus"
        base_dir.mkdir(exist_ok=True)
        self.path = base_dir / "intents.json"

    def load(self):
        if not self.path.exists():
            return []

        try:
            if self.path.stat().st_size == 0:
                return []

            with open(self.path, "r") as f:
                raw = json.load(f)

            return [Intent.from_dict(d) for d in raw]

        except (json.JSONDecodeError, ValueError):
            # Corrupted storage → reset safely
            return []

    def save(self, intents: List[Intent]):
        with self.path.open("w") as f:
            json.dump(
                [
                    {
                        "intent_id": i.intent_id,
                        "name": i.name,
                        "capability": i.capability,
                        "priority": int(i.priority),
                        "preemptible": i.preemptible,
                        "ttl": i.ttl,
                        "state": i.state.value,
                        "created_at": i.created_at,
                    }
                    for i in intents
                ],
                f,
                indent=2,
            )

