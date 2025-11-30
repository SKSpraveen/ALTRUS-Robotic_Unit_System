from altrus.adapters.base.adapter import Adapter
from altrus.core.intent.intent import Intent, IntentPriority


class ROSAdapter(Adapter):
    """
    Translates ROS messages into middleware intents.
    """

    def translate(self, ros_msg) -> Intent:
        """
        ros_msg is abstracted for now.
        """
        return Intent(
            intent_id=None,
            capability=ros_msg.command,
            priority=IntentPriority.NORMAL
        )
