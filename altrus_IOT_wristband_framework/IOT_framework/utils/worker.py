from __future__ import annotations

from typing import Callable, Iterable

from PySide6 import QtCore


class WorkerSignals(QtCore.QObject):
    finished = QtCore.Signal()
    error = QtCore.Signal(str)
    log = QtCore.Signal(str)


class WorkerThread(QtCore.QThread):
    def __init__(self, func: Callable, *args) -> None:
        super().__init__()
        self.func = func
        self.args = args
        self.signals = WorkerSignals()

    def run(self) -> None:
        try:
            self.func(*self.args, self.signals.log)
        except Exception as exc:  # noqa: BLE001 - background boundary
            self.signals.error.emit(str(exc))
        else:
            self.signals.finished.emit()
