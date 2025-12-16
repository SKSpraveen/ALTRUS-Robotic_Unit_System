import threading
import time


def start_health_monitor(registry):
    def loop():
        while True:
            registry.evaluate_health()
            time.sleep(2)

    threading.Thread(target=loop, daemon=True).start()
