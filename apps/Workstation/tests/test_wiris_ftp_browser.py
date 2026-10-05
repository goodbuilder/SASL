from __future__ import annotations

import time

from PyQt5 import QtCore, QtWidgets

from Workstation.wiris_ftp_browser import WirisFtpBrowserDialog


def _app() -> QtWidgets.QApplication:
    return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


class _SlowWorker(QtCore.QObject):
    finished = QtCore.pyqtSignal()

    @QtCore.pyqtSlot()
    def run(self) -> None:
        QtCore.QThread.msleep(300)
        self.finished.emit()


def test_closing_dialog_while_ftp_thread_is_running_does_not_abort() -> None:
    """Close is deferred without freezing the GUI or destroying a live QThread."""
    app = _app()
    dialog = WirisFtpBrowserDialog(hosts={})

    thread = QtCore.QThread(dialog)
    worker = _SlowWorker()
    worker.moveToThread(thread)
    thread.started.connect(worker.run)
    worker.finished.connect(thread.quit)
    thread.finished.connect(dialog._clear_thread)
    thread.start()
    dialog._thread = thread
    dialog._worker = worker

    assert thread.isRunning()

    started_s = time.monotonic()
    dialog.done(QtWidgets.QDialog.Rejected)

    assert time.monotonic() - started_s < 0.05
    assert thread.isRunning()
    deadline_s = time.monotonic() + 1.0
    while thread.isRunning() and time.monotonic() < deadline_s:
        app.processEvents()
        time.sleep(0.01)
    app.processEvents()
    assert not thread.isRunning()
    assert dialog._pending_done_result is None

    dialog.deleteLater()
    app.processEvents()


def test_close_button_remains_available_while_busy() -> None:
    app = _app()
    dialog = WirisFtpBrowserDialog(hosts={})

    assert dialog.close_button.isEnabled()
    dialog._busy(True)
    assert dialog.close_button.isEnabled()
    dialog._busy(False)
    assert dialog.close_button.isEnabled()

    dialog.deleteLater()
    app.processEvents()
