"""Background workers for CPU-heavy optical-pose presentation commands."""

from __future__ import annotations

from PyQt5 import QtCore

from .optical_pose_controller import OpticalPoseAutoSelectJob


class OpticalPoseAutoSelectWorker(QtCore.QObject):
    """Run one frozen-frame auto-selection without blocking the Qt GUI thread."""

    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(self, job: OpticalPoseAutoSelectJob, parent: QtCore.QObject | None = None) -> None:
        super().__init__(parent)
        self.job = job
        self._cancel_requested = False

    @QtCore.pyqtSlot()
    def run(self) -> None:
        try:
            if self._cancel_requested:
                return
            self.completed.emit(self.job.run())
        except Exception as exc:
            self.failed.emit(str(exc))
        finally:
            self.finished.emit()

    @QtCore.pyqtSlot()
    def cancel(self) -> None:
        """Prevent a queued job from starting; running OpenCV work is not interrupted."""

        self._cancel_requested = True


__all__ = ["OpticalPoseAutoSelectWorker"]
