from __future__ import annotations

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
    """Destroying a live QThread aborts the whole process (SIGABRT); the
    dialog must block in done() until any in-flight worker thread actually
    finishes instead of letting Qt tear it down mid-request."""
    app = _app()
    dialog = WirisFtpBrowserDialog(hosts={})

    thread = QtCore.QThread(dialog)
    worker = _SlowWorker()
    worker.moveToThread(thread)
    thread.started.connect(worker.run)
    worker.finished.connect(thread.quit)
    thread.start()
    dialog._thread = thread
    dialog._worker = worker

    assert thread.isRunning()

    dialog.done(QtWidgets.QDialog.Rejected)

    assert not thread.isRunning()

    dialog.deleteLater()
    app.processEvents()


def test_close_button_disabled_while_busy() -> None:
    app = _app()
    dialog = WirisFtpBrowserDialog(hosts={})

    assert dialog.close_button.isEnabled()
    dialog._busy(True)
    assert not dialog.close_button.isEnabled()
    dialog._busy(False)
    assert dialog.close_button.isEnabled()

    dialog.deleteLater()
    app.processEvents()
