import threading
import time


class HealthMonitor:
    def __init__(self, registry,fault_engine, interval_seconds=5):
        self.registry = registry
        self.interval = interval_seconds
        self._stop = threading.Event()
        self.fault_engine = fault_engine
        self._thread = threading.Thread(
            target=self._run,
            daemon=True
        )

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()

    def _run(self):
        while not self._stop.is_set():
            self.registry.evaluate_health()
            self.fault_engine.evaluate()
            time.sleep(self.interval)
