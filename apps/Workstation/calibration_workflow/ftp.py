"""WIRIS calibration worker resources and ordered OZ save admission.

The shared CalibrationJobs slot owns checkerboard persistence. This owner only
advances a downloaded pair after admission, and resumes it after ordered job
retirement. Neither shutdown nor retry consumes an unadmitted queue head.
"""
from __future__ import annotations
from dataclasses import dataclass, field
import threading
from PyQt5 import QtCore

FTP_RESOURCE_FIELDS = ('calibration_wiris_ftp_thread', 'calibration_wiris_ftp_worker', 'calibration_wiris_ftp_oz_thread', 'calibration_wiris_ftp_oz_worker', 'calibration_wiris_oz_capture_thread', 'calibration_wiris_oz_capture_worker', 'calibration_wiris_oz_sync_thread', 'calibration_wiris_oz_sync_worker', 'calibration_wiris_oz_delete_thread', 'calibration_wiris_oz_delete_worker', 'calibration_wiris_oz_format_thread', 'calibration_wiris_oz_format_worker', 'calibration_wiris_oz_gilc_thread', 'calibration_wiris_oz_gilc_worker')
FTP_PENDING_FIELDS = ('_pending_oz_capture_target_infos', '_pending_ftp_oz_sync_target_infos', '_pending_ftp_oz_sync_rounds', '_pending_ftp_oz_sync_round_ids', '_pending_ftp_oz_sync_frames', '_ftp_oz_sync_saved_count')


@dataclass(frozen=True)
class FtpQueueActions:
    capture: object
    jobs: object
    legacy_thread: object
    shutting_down: object
    completed: object


@dataclass
class FtpTransaction:
    kind: str
    token: int
    thread: object
    worker: object
    completed: object
    failed: object
    retired: object
    root: object = None
    relay: object = None
    terminal: bool = False
    started: threading.Event = field(default_factory=threading.Event)
    callback_depth: int = 0
    retire_requested: bool = False


class _FtpRelay(QtCore.QObject):
    def __init__(self, owner, transaction):
        super().__init__(owner)
        self.owner = owner
        self.transaction = transaction

    @QtCore.pyqtSlot(object)
    def completed(self, result):
        self.owner.deliver(self.transaction, "completed", result)

    @QtCore.pyqtSlot(str)
    def failed(self, error):
        self.owner.deliver(self.transaction, "failed", error)

    @QtCore.pyqtSlot()
    def finished(self):
        self.owner.retire(self.transaction)


class CalibrationFtpTransactions(QtCore.QObject):
    def __init__(self, parent=None, *, publish_legacy=None):
        super().__init__(parent)
        self._publish_legacy = publish_legacy
        self.actions = None
        self.closing = False
        self.operation_roots = {}
        self.ledger_root = None
        self.queue_root = None
        self._transactions = {}
        self._legacy_resources = {name: None for name in FTP_RESOURCE_FIELDS}
        self._next_token = 0
        self.callback_error = ""
        self._pending_oz_capture_target_infos = ()
        self._pending_ftp_oz_sync_target_infos = ()
        self._pending_ftp_oz_sync_rounds = []
        self._pending_ftp_oz_sync_round_ids = []
        self._pending_ftp_oz_sync_frames = []
        self._ftp_oz_sync_saved_count = 0

    @property
    def busy(self):
        return bool(self._transactions) or any(value is not None for value in self._legacy_resources.values())

    def _publish(self):
        if self._publish_legacy is not None:
            self._publish_legacy(self)

    def read_resource(self, name):
        kind, attribute = name.removeprefix("calibration_wiris_").rsplit("_", 1)
        transaction = self._transactions.get(kind)
        return getattr(transaction, attribute) if transaction is not None else self._legacy_resources[name]

    def write_resource(self, name, value):
        kind, attribute = name.removeprefix("calibration_wiris_").rsplit("_", 1)
        transaction = self._transactions.get(kind)
        if transaction is not None:
            if value is not getattr(transaction, attribute):
                raise RuntimeError("Admitted calibration FTP resources retire only through their owner")
            return
        self._legacy_resources[name] = value

    def prepare(self, kind, worker, *, completed, failed, retired, root=None):
        field_name = f"calibration_wiris_{kind}_thread"
        try:
            if QtCore.QThread.currentThread() is not self.thread():
                raise RuntimeError("Calibration FTP must be admitted on its Qt owner thread")
            if self.closing:
                raise RuntimeError("Calibration FTP is shutting down")
            if field_name not in FTP_RESOURCE_FIELDS:
                raise ValueError(f"Unknown calibration FTP operation: {kind}")
            if getattr(self, field_name) is not None:
                raise RuntimeError(f"Calibration FTP operation already admitted: {kind}")
            if not isinstance(worker, QtCore.QObject) or not callable(getattr(worker, "run", None)):
                raise TypeError("Calibration FTP worker requires QObject/run")
            if not all(callable(callback) for callback in (completed, failed, retired)):
                raise TypeError("Calibration FTP requires completed, failed and retired callbacks")
        except Exception:
            delete = getattr(worker, "deleteLater", None)
            if callable(delete):
                delete()
            raise
        thread = QtCore.QThread(self)
        self._next_token += 1
        transaction = FtpTransaction(kind, self._next_token, thread, worker, completed, failed, retired, root)
        relay = _FtpRelay(self, transaction)
        transaction.relay = relay
        try:
            thread.started.connect(transaction.started.set, QtCore.Qt.DirectConnection)
            thread.started.connect(worker.run)
            worker.completed.connect(relay.completed, QtCore.Qt.QueuedConnection)
            worker.failed.connect(relay.failed, QtCore.Qt.QueuedConnection)
            worker.finished.connect(thread.quit)
            worker.finished.connect(worker.deleteLater)
            thread.finished.connect(relay.finished, QtCore.Qt.QueuedConnection)
            worker.moveToThread(thread)
        except Exception:
            worker.deleteLater()
            relay.deleteLater()
            thread.deleteLater()
            raise
        self._transactions[kind] = transaction
        if root is not None:
            self.operation_roots[kind] = root
        self._publish()
        return thread, worker

    def start(self, kind, *, expected_thread=None):
        transaction = self._transactions.get(kind)
        if transaction is None:
            if self.closing:
                return False
            raise RuntimeError(f"Calibration FTP operation was not prepared: {kind}")
        if expected_thread is not None and transaction.thread is not expected_thread:
            return False
        try:
            if self.closing:
                raise RuntimeError("Calibration FTP is shutting down")
            transaction.thread.start()
            if not transaction.thread.isRunning():
                raise RuntimeError("Calibration FTP thread did not start")
            return True
        except Exception as exc:
            return self.fail_start(transaction, exc)

    def fail_start(self, transaction, error):
        if self._transactions.get(transaction.kind) is not transaction:
            return False
        if transaction.started.is_set() or transaction.thread.isRunning():
            self.callback_error = f"start reported an error after launch: {error}"
            # A command or download may already have happened. Consume that
            # admitted outcome before releasing its resource or retrying it.
            return True
        self.deliver(transaction, "failed", str(error))
        transaction.thread.quit()
        if not transaction.thread.isRunning():
            self.retire(transaction)
            transaction.worker.deleteLater()
        return False

    def deliver(self, transaction, event, value):
        if self._transactions.get(transaction.kind) is not transaction or transaction.terminal:
            return
        transaction.terminal = True
        transaction.callback_depth += 1
        try:
            getattr(transaction, event)(value)
        except Exception as exc:
            self.callback_error = f"{event}: {exc}"
            if event == "completed":
                try:
                    transaction.failed(str(exc))
                except Exception as failure:
                    self.callback_error += f"; failure presentation: {failure}"
        finally:
            transaction.callback_depth -= 1
            if transaction.callback_depth == 0 and transaction.retire_requested:
                self.retire(transaction)

    def retire(self, transaction):
        if self._transactions.get(transaction.kind) is not transaction:
            return False
        if transaction.callback_depth:
            transaction.retire_requested = True
            return False
        try:
            if transaction.thread.isRunning():
                return False
        except RuntimeError:
            pass
        del self._transactions[transaction.kind]
        transaction.retire_requested = False
        transaction.terminal = True
        self._publish()
        try:
            transaction.retired()
        except Exception as exc:
            self.callback_error = f"retired: {exc}"
        finally:
            transaction.relay.deleteLater()
            transaction.thread.deleteLater()
        return True

    def root_for(self, kind, fallback=None):
        transaction = self._transactions.get(kind)
        if transaction is not None and transaction.root is not None:
            return transaction.root
        return self.operation_roots.get(kind, fallback)

    def clear(self, kind):
        # Untagged legacy callbacks cannot release a replacement transaction.
        if kind in self._transactions:
            return False
        thread_name = f"calibration_wiris_{kind}_thread"
        thread = self._legacy_resources[thread_name]
        if thread is not None:
            try:
                is_running = getattr(thread, "isRunning", None)
                if callable(is_running) and is_running():
                    return False
            except RuntimeError:
                pass
        self._legacy_resources[thread_name] = None
        self._legacy_resources[f"calibration_wiris_{kind}_worker"] = None
        self._publish()
        return True

    def resource_inventory(self):
        return {name: getattr(self, name) for name in FTP_RESOURCE_FIELDS}

    def request_shutdown(self):
        self.closing = True
        for transaction in tuple(self._transactions.values()):
            if not transaction.started.is_set() and not transaction.thread.isRunning():
                self.fail_start(transaction, "Calibration FTP shutdown before launch")
            else:
                transaction.thread.quit()
        for kind in {name.removeprefix("calibration_wiris_").rsplit("_", 1)[0] for name in FTP_RESOURCE_FIELDS}:
            if kind in self._transactions:
                continue
            thread = self._legacy_resources[f"calibration_wiris_{kind}_thread"]
            if thread is not None:
                try:
                    quit_thread = getattr(thread, "quit", None)
                    if callable(quit_thread):
                        quit_thread()
                    is_running = getattr(thread, "isRunning", None)
                    if not callable(is_running) or not is_running():
                        self.clear(kind)
                except RuntimeError:
                    self.clear(kind)
        # A stopped admitted QThread may still have an undelivered result.
        return not self.busy

    def _process_next_ftp_oz_sync_pair(self) -> None:
        if self.actions is None:
            raise RuntimeError("Calibration FTP queue operations are not composed")
        owner = self.actions.jobs()
        if self.closing or self.actions.shutting_down() or (owner is not None and owner.closing):
            return
        queue = getattr(self, "_pending_ftp_oz_sync_frames", None)
        if not queue:
            saved = getattr(self, "_ftp_oz_sync_saved_count", 0)
            self.actions.completed(saved)
            return
        entry = queue[0]
        capture_id, frames, metadata_by_camera = entry
        request = dict(frames=frames, metadata_by_camera=metadata_by_camera, capture_origin="ftp_oz_sync")
        if self.queue_root is not None:
            request["capture_root"] = self.queue_root
        started = self.actions.capture(**request)
        if not started:
            QtCore.QTimer.singleShot(200, lambda: self._resume_ftp_oz_sync_queue(queue))
            return
        if queue and queue[0] is entry:
            queue.pop(0)
        if getattr(self, "_pending_ftp_oz_sync_frames", None) is queue:
            self._ftp_oz_sync_saved_count = getattr(self, "_ftp_oz_sync_saved_count", 0) + 1
        owner = self.actions.jobs()
        if owner is not None and owner.has_admitted:
            if not owner.closing and not owner.after_current_retired(lambda: self._resume_ftp_oz_sync_queue(queue)):
                QtCore.QTimer.singleShot(200, lambda: self._resume_ftp_oz_sync_queue(queue))
            return
        thread = self.actions.legacy_thread()
        if thread is not None:
            thread.finished.connect(lambda: self._resume_ftp_oz_sync_queue(queue))
        else:
            QtCore.QTimer.singleShot(200, lambda: self._resume_ftp_oz_sync_queue(queue))


    def _resume_ftp_oz_sync_queue(self, queue) -> None:
        if getattr(self, "_pending_ftp_oz_sync_frames", None) is queue:
            self._process_next_ftp_oz_sync_pair()




def _ftp_resource(name):
    return property(lambda owner: owner.read_resource(name), lambda owner, value: owner.write_resource(name, value))


for _name in FTP_RESOURCE_FIELDS:
    setattr(CalibrationFtpTransactions, _name, _ftp_resource(_name))
del _name
