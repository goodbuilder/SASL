"""Bounded read-only session queries with GUI-thread delivery and retirement.

Requests own only immutable root/selection identifiers. Workers construct the
existing SessionEvidenceQuery from that root; no panel, backend or widget is
passed across the thread boundary. Stored NED/FRD/RDF fields, origins, units,
source clocks and integer timestamps are serialized without reinterpretation.
"""
from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import json
import os
from pathlib import Path

from PyQt5 import QtCore

from .catalog import SessionEvidenceQuery


@dataclass(frozen=True)
class EvidenceQueryRequest:
    token: int
    kind: str
    data_root: str
    session_id: str = ""
    artifact_name: str = ""
    measurement_id: str = ""
    limit: int = 100


@dataclass(frozen=True)
class EvidenceQueryResult:
    request: EvidenceQueryRequest
    payload_json: bytes = b"null"
    error: str = ""

    def data(self):
        """Give the GUI a detached tree; the admitted result remains immutable.

        Legacy JSON previews may contain non-finite floats. The private JSON
        snapshot retains their existing Python JSON semantics and exact ints.
        This is not an authoritative artifact encoder or a schema migration.
        """
        return json.loads(self.payload_json)


class _QueryThread(QtCore.QThread):
    completed = QtCore.pyqtSignal(object)

    def __init__(self, parent, request, query_factory):
        super().__init__(parent)
        self.request = request
        self._query_factory = query_factory

    def run(self):
        request = self.request
        try:
            query = self._query_factory(Path(request.data_root))
            if request.kind == "catalog":
                payload = query.session_catalog()
            else:
                payload = query.session_artifact_rows(
                    request.session_id,
                    request.artifact_name,
                    measurement_id=request.measurement_id or None,
                    limit=request.limit,
                )
            result = EvidenceQueryResult(
                request,
                json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8"),
            )
        except Exception as exc:
            result = EvidenceQueryResult(request, error=f"{type(exc).__name__}: {exc}")
        self.completed.emit(result)


@dataclass
class _QueryTask:
    request: EvidenceQueryRequest
    thread: _QueryThread
    relay: object = None
    delivered: bool = False


class _QueryRelay(QtCore.QObject):
    def __init__(self, owner, task):
        super().__init__(owner)
        self.owner = owner
        self.task = task
        self.retirement_timer = QtCore.QTimer(self)
        self.retirement_timer.setSingleShot(True)
        self.retirement_timer.setInterval(1)
        self.retirement_timer.timeout.connect(self.finished)

    @QtCore.pyqtSlot(object)
    def completed(self, result):
        self.owner._deliver(self.task, result)

    @QtCore.pyqtSlot()
    def finished(self):
        self.owner._retire(self.task)


class SessionEvidenceQueryRuntime(QtCore.QObject):
    """One running query, plus at most one latest pending request per kind.

    Catalog refresh and artifact selection have independent latest identities;
    neither can starve the other by erasing its pending request. The two kinds
    bound pending admission at two. Shutdown invalidates delivery immediately,
    but stays busy until the real thread-finished event retires on the GUI.
    """

    completed = QtCore.pyqtSignal(object)
    idle = QtCore.pyqtSignal()
    _KINDS = frozenset({"catalog", "artifact"})

    def __init__(self, parent: QtCore.QObject, *, query_factory: Callable = SessionEvidenceQuery):
        if not isinstance(parent, QtCore.QObject):
            raise TypeError("Session query runtime requires an application QObject parent")
        if not callable(query_factory):
            raise TypeError("Session query factory must be callable")
        super().__init__(parent)
        self._query_factory = query_factory
        self._active: _QueryTask | None = None
        self._pending: dict[str, EvidenceQueryRequest] = {}
        self._latest: dict[str, EvidenceQueryRequest] = {}
        self._next_token = 0
        self._closing = False

    def _require_owner_thread(self):
        if QtCore.QThread.currentThread() is not self.thread():
            raise RuntimeError("Session query admission and retirement require the GUI owner thread")

    @property
    def busy(self) -> bool:
        return self._active is not None

    @property
    def pending_count(self) -> int:
        return len(self._pending)

    @property
    def closing(self) -> bool:
        return self._closing

    def is_current(self, request: EvidenceQueryRequest) -> bool:
        return not self._closing and self._latest.get(request.kind) == request

    def has_work(self, kind: str) -> bool:
        return kind in self._pending or (
            self._active is not None and self._active.request.kind == kind
        )

    def submit_catalog(self, data_root: Path | str) -> EvidenceQueryRequest:
        return self._submit("catalog", data_root)

    def submit_artifact(
        self, data_root: Path | str, session_id: str, artifact_name: str,
        *, measurement_id: str | None = None, limit: int = 100,
    ) -> EvidenceQueryRequest:
        return self._submit(
            "artifact", data_root, session_id=str(session_id),
            artifact_name=str(artifact_name), measurement_id=str(measurement_id or ""),
            limit=max(0, int(limit)),
        )

    def _submit(self, kind, data_root, **selection):
        self._require_owner_thread()
        if self._closing:
            raise RuntimeError("Session queries are shutting down")
        self._next_token += 1
        # abspath captures CWD lexically without filesystem stat/resolve work
        # on the GUI thread. Query safety checks remain with SessionEvidenceQuery.
        request = EvidenceQueryRequest(
            self._next_token, kind, os.path.abspath(os.fspath(data_root)), **selection,
        )
        self._latest[kind] = request
        if self._active is None:
            self._launch(request)
        else:
            self._pending.pop(kind, None)
            self._pending[kind] = request
        return request

    def invalidate(self, kind: str | None = None) -> None:
        self._require_owner_thread()
        kinds = self._KINDS if kind is None else (kind,)
        for name in kinds:
            if name not in self._KINDS:
                raise ValueError(f"Unknown session query kind: {name}")
            self._latest.pop(name, None)
            self._pending.pop(name, None)

    def _launch(self, request):
        thread = _QueryThread(self, request, self._query_factory)
        task = _QueryTask(request, thread)
        relay = _QueryRelay(self, task)
        task.relay = relay
        thread.completed.connect(relay.completed, QtCore.Qt.QueuedConnection)
        thread.finished.connect(relay.finished, QtCore.Qt.QueuedConnection)
        self._active = task
        try:
            thread.start()
            if not thread.isRunning() and not thread.isFinished():
                raise RuntimeError("Session query thread did not start")
        except Exception as exc:
            # QThread is also the worker, so a failed start has no QObject
            # stranded in the affinity of a thread which never ran.
            self._deliver(task, EvidenceQueryResult(request, error=f"{type(exc).__name__}: {exc}"))
            if thread.isRunning():
                thread.requestInterruption()
            else:
                self._retire(task)

    def _deliver(self, task, result):
        self._require_owner_thread()
        if self._active is not task or task.delivered:
            return
        task.delivered = True
        if self.is_current(task.request) and result.request == task.request:
            self.completed.emit(result)

    def _retire(self, task):
        self._require_owner_thread()
        if self._active is not task:
            return
        if task.thread.isRunning():
            # QThread.finished can be queued while native thread teardown is
            # still completing. Retry without dropping its only owner/relay.
            task.relay.retirement_timer.start()
            return
        task.relay.retirement_timer.stop()
        self._active = None
        task.relay.deleteLater()
        task.thread.deleteLater()
        while self._pending and not self._closing:
            kind = next(iter(self._pending))
            request = self._pending.pop(kind)
            if self.is_current(request):
                self._launch(request)
                return
        self.idle.emit()

    @QtCore.pyqtSlot()
    def request_shutdown(self) -> bool:
        self._require_owner_thread()
        self._closing = True
        self.invalidate()
        if self._active is not None:
            # Read-only filesystem queries are not forcibly killed midway.
            # Keep the parent-owned thread discoverable through actual retire.
            self._active.thread.requestInterruption()
        return not self.busy

    def resource_inventory(self):
        prefix = f"session_evidence_query_{id(self)}"
        return {
            f"{prefix}_worker_thread": None if self._active is None else self._active.thread,
            f"{prefix}_retirement_timer": None if self._active is None else self._active.relay.retirement_timer,
        }
