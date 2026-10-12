"""Read-only file queries keep selection identity and application-owned threads."""
from __future__ import annotations

from dataclasses import FrozenInstanceError
import math
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import pytest
from PyQt5 import QtCore, QtGui, QtWidgets, sip

from Workstation.evidence_views.catalog import SessionEvidenceQuery
from Workstation.evidence_views.query_runtime import SessionEvidenceQueryRuntime


def _drain(runtime, timeout=3.0):
    app = QtWidgets.QApplication.instance()
    deadline = time.monotonic() + timeout
    idle_rounds = 0
    while idle_rounds < 2 and time.monotonic() < deadline:
        app.processEvents()
        idle_rounds = idle_rounds + 1 if not runtime.busy else 0
        QtCore.QThread.msleep(1)
    assert not runtime.busy and idle_rounds == 2


def _close(runtime, parent, *release_events):
    for event in release_events:
        event.set()
    runtime.request_shutdown()
    _drain(runtime)
    runtime.deleteLater()
    parent.deleteLater()
    QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)


def test_query_runs_off_gui_and_delivers_detached_immutable_result_on_gui(tmp_path):
    parent = QtCore.QObject()
    called_threads = []
    payload = {"rows": [{"point_left_rdf_m": [2, 7, 19], "world_ned_m": [41, -13, 23],
                         "baseline_center_frd_m": [17, -3, 5], "clock_ns": 9007199254740993,
                         "legacy_nan": float("nan")}], "columns": ["clock_ns"], "error": ""}

    class Query:
        def __init__(self, root):
            assert root == tmp_path

        def session_artifact_rows(self, session, artifact, **options):
            called_threads.append(QtCore.QThread.currentThread())
            assert (session, artifact, options) == ("s", "raw.jsonl", {"measurement_id": "m1", "limit": 7})
            return payload

    runtime = SessionEvidenceQueryRuntime(parent, query_factory=Query)
    events = []
    runtime.completed.connect(lambda result: events.append((result, QtCore.QThread.currentThread())))
    try:
        request = runtime.submit_artifact(tmp_path, "s", "raw.jsonl", measurement_id="m1", limit=7)
        _drain(runtime)
        [(result, receiver_thread)] = events
        assert called_threads[0] is not runtime.thread()
        assert receiver_thread is runtime.thread()
        assert result.request == request
        with pytest.raises(FrozenInstanceError):
            request.session_id = "changed"
        payload["rows"][0]["point_left_rdf_m"][0] = 999
        first = result.data()
        assert first["rows"][0]["point_left_rdf_m"] == [2, 7, 19]
        assert first["rows"][0]["clock_ns"] == 9007199254740993
        assert math.isnan(first["rows"][0]["legacy_nan"])
        first["rows"][0]["world_ned_m"][0] = 0
        assert result.data()["rows"][0]["world_ned_m"] == [41, -13, 23]
    finally:
        _close(runtime, parent)


def test_latest_pending_is_bounded_per_kind_and_stale_completion_is_rejected(tmp_path):
    parent = QtCore.QObject()
    started, release = threading.Event(), threading.Event()
    invoked = []

    class Query:
        def __init__(self, root):
            self.root = root

        def session_artifact_rows(self, session, artifact, **options):
            invoked.append(("artifact", self.root, session))
            if session == "first":
                started.set()
                assert release.wait(3.0)
            return {"rows": [{"session": session}], "error": ""}

        def session_catalog(self):
            invoked.append(("catalog", self.root, ""))
            return [{"session_id": "latest-catalog"}]

    runtime = SessionEvidenceQueryRuntime(parent, query_factory=Query)
    events = []
    runtime.completed.connect(events.append)
    try:
        first = runtime.submit_artifact(tmp_path, "first", "file")
        assert started.wait(1.0)
        runtime.submit_artifact(tmp_path / "discarded-root", "middle", "file")
        latest = runtime.submit_artifact(tmp_path / "latest-root", "last", "file")
        catalog = runtime.submit_catalog(tmp_path / "catalog-root")
        assert runtime.pending_count == 2
        assert runtime.has_work("catalog") and runtime.has_work("artifact")
        assert not runtime.is_current(first)
        release.set()
        _drain(runtime)
        assert [result.request for result in events] == [latest, catalog]
        assert invoked == [("artifact", tmp_path, "first"),
                           ("artifact", tmp_path / "latest-root", "last"),
                           ("catalog", tmp_path / "catalog-root", "")]
        assert runtime.pending_count == 0
    finally:
        _close(runtime, parent, release)


def test_shutdown_retains_worker_until_queued_thread_retirement(tmp_path, monkeypatch):
    parent = QtCore.QObject()
    started, release = threading.Event(), threading.Event()

    class Query:
        def __init__(self, root):
            pass

        def session_catalog(self):
            started.set()
            assert release.wait(3.0)
            return []

    runtime = SessionEvidenceQueryRuntime(parent, query_factory=Query)
    events = []
    runtime.completed.connect(events.append)
    try:
        runtime.submit_catalog(tmp_path)
        assert started.wait(1.0)
        task = runtime._active
        runtime.submit_artifact(tmp_path, "s", "file")
        assert runtime.request_shutdown() is False
        assert runtime.pending_count == 0
        release.set()
        assert task.thread.wait(2000)
        # Actual thread exit alone does not drop its ownership/inventory before
        # the owner-thread finished relay has retired the task.
        assert runtime.busy
        assert task.thread in runtime.resource_inventory().values()
        with pytest.raises(RuntimeError, match="shutting down"):
            runtime.submit_catalog(tmp_path)
        # Cover an early queued finished delivery while native teardown still
        # reports running once. Retirement must retry instead of losing it.
        observations = iter((True, False))
        monkeypatch.setattr(task.thread, "isRunning", lambda: next(observations, False))
        _drain(runtime)
        assert events == []
        assert all(value is None for value in runtime.resource_inventory().values())
        assert runtime.request_shutdown() is True
    finally:
        _close(runtime, parent, release)


def test_query_failure_and_start_failure_allow_later_requests(tmp_path, monkeypatch):
    parent = QtCore.QObject()

    class Query:
        def __init__(self, root):
            self.root = root

        def session_catalog(self):
            if self.root.name == "denied":
                raise PermissionError("fixture denied")
            return []

    runtime = SessionEvidenceQueryRuntime(parent, query_factory=Query)
    events = []
    runtime.completed.connect(events.append)
    try:
        runtime.submit_catalog(tmp_path / "denied")
        _drain(runtime)
        assert "PermissionError: fixture denied" == events[-1].error
        with monkeypatch.context() as patch:
            def fail_start(thread):
                raise RuntimeError("fixture start failed")
            patch.setattr(QtCore.QThread, "start", fail_start)
            runtime.submit_catalog(tmp_path)
            assert not runtime.busy
            assert events[-1].error == "RuntimeError: fixture start failed"
        request = runtime.submit_catalog(tmp_path)
        _drain(runtime)
        assert events[-1].request == request and events[-1].error == ""
    finally:
        _close(runtime, parent)


def test_real_session_query_preserves_stored_rows(tmp_path):
    root = tmp_path / "recordings" / "s"
    root.mkdir(parents=True)
    (root / "raw.jsonl").write_text('{"north_m": 41, "east_m": -13, "down_m": 23, "time_ns": 9007199254740993}\n', encoding="utf-8")
    expected = SessionEvidenceQuery(tmp_path).session_artifact_rows("s", "raw.jsonl")
    parent = QtCore.QObject()
    runtime = SessionEvidenceQueryRuntime(parent)
    events = []
    runtime.completed.connect(events.append)
    try:
        runtime.submit_artifact(tmp_path, "s", "raw.jsonl")
        _drain(runtime)
        [result] = events
        assert result.error == ""
        actual = result.data()
        assert actual["rows"] == list(expected["rows"])
        assert actual["columns"] == list(expected["columns"])
        assert actual["rows"][0]["time_ns"] == 9007199254740993
    finally:
        _close(runtime, parent)


def _review_harness(tmp_path, monkeypatch, query_factory):
    from Workstation.evidence_views import query_runtime
    from Workstation.workspace_views.recording import RecordingWorkspace

    parent = QtCore.QObject()
    parent.app_root = tmp_path
    runtimes = []
    parent.application = SimpleNamespace(register_evidence_query_runtime=runtimes.append)
    view = RecordingWorkspace.__new__(RecordingWorkspace)
    QtWidgets.QWidget.__init__(view)
    view.backend = SimpleNamespace(panel=parent)
    view._sessionless = False
    view._query_disposed = False
    view._catalog_query_failed = False
    view._last_session_scan_monotonic_s = float("-inf")
    view._last_session_scan_root = ""
    view._session_catalog_signature = ()
    view._evidence_query_runtime = None
    view._catalog_query_request = None
    view._artifact_query_request = None
    view.session_records = {}
    view.session_selector = QtWidgets.QComboBox(view)
    view.artifact_selector = QtWidgets.QComboBox(view)
    view.review_model = QtGui.QStandardItemModel(view)
    view.review_status_label = QtWidgets.QLabel(view)
    view._refresh_package_details = lambda: None
    view.session_selector.currentIndexChanged.connect(lambda _index: view._refresh_review_artifacts())
    view.artifact_selector.currentIndexChanged.connect(lambda _index: view._preview_selected_artifact())
    original = query_runtime.SessionEvidenceQueryRuntime
    monkeypatch.setattr(query_runtime, "SessionEvidenceQueryRuntime", lambda owner: original(owner, query_factory=query_factory))
    view._initialize_session_queries(parent)
    assert runtimes == [view._evidence_query_runtime]
    return view, parent, view._evidence_query_runtime


def test_recording_view_root_change_rejects_old_result_and_periodic_scan_does_not_starve(tmp_path, monkeypatch):
    started, release = threading.Event(), threading.Event()

    class Query:
        def __init__(self, root):
            self.root = root

        def session_catalog(self):
            if self.root == tmp_path:
                started.set()
                assert release.wait(3.0)
            return [{"session_id": self.root.name, "catalog_key": self.root.name, "artifacts": []}]

    view, parent, runtime = _review_harness(tmp_path, monkeypatch, Query)
    try:
        view.refresh_sessions()
        assert started.wait(1.0)
        first = view._catalog_query_request
        view._last_session_scan_monotonic_s = float("-inf")
        view.refresh_sessions(force=False)
        assert view._catalog_query_request == first
        assert runtime.pending_count == 0
        parent.app_root = tmp_path / "new-root"
        view.refresh_sessions(force=False)
        assert view._catalog_query_request.data_root == str(tmp_path / "new-root")
        release.set()
        _drain(runtime)
        assert list(view.session_records) == ["new-root"]
        assert view.session_selector.currentData() == "new-root"
        assert view.review_status_label.text() == "Select a session and file"
    finally:
        view.dispose_view_resources()
        _close(runtime, parent, release)
        view.deleteLater()


def test_view_destruction_keeps_application_thread_alive_until_retirement(tmp_path, monkeypatch):
    started, release = threading.Event(), threading.Event()

    class Query:
        def __init__(self, root):
            pass

        def session_catalog(self):
            started.set()
            assert release.wait(3.0)
            return []

    view, parent, runtime = _review_harness(tmp_path, monkeypatch, Query)
    try:
        view.refresh_sessions()
        assert started.wait(1.0)
        task = runtime._active
        view.deleteLater()
        QtCore.QCoreApplication.sendPostedEvents(None, QtCore.QEvent.DeferredDelete)
        assert sip.isdeleted(view)
        assert not sip.isdeleted(runtime)
        assert runtime.closing and runtime.busy
        assert task.thread in runtime.resource_inventory().values()
        release.set()
        _drain(runtime)
        assert runtime.request_shutdown() is True
    finally:
        _close(runtime, parent, release)


def test_recording_view_artifact_selection_uses_worker_and_ignores_older_result(tmp_path, monkeypatch):
    from Workstation.evidence import EvidenceAdapter

    started, release = threading.Event(), threading.Event()
    calls = []

    class Query:
        def __init__(self, root):
            assert root == tmp_path

        def session_artifact_rows(self, session, artifact, **options):
            calls.append((session, QtCore.QThread.currentThread()))
            if session == "one":
                started.set()
                assert release.wait(3.0)
            return {"columns": ["session", "time_ns"], "rows": [{"session": session, "time_ns": 9007199254740993}], "error": ""}

    monkeypatch.setattr(EvidenceAdapter, "session_artifact_rows", lambda *args, **kwargs: pytest.fail("GUI synchronous file reader"))
    view, parent, runtime = _review_harness(tmp_path, monkeypatch, Query)
    try:
        view.session_records = {name: {"session_id": name, "artifacts": ["raw.jsonl"]} for name in ("one", "two")}
        view.session_selector.blockSignals(True)
        view.session_selector.addItem("one", "one")
        view.session_selector.addItem("two", "two")
        view.session_selector.blockSignals(False)
        view._refresh_review_artifacts()
        assert started.wait(1.0)
        view.session_selector.setCurrentIndex(1)
        assert view.review_model.rowCount() == 0
        assert runtime.pending_count == 1
        release.set()
        _drain(runtime)
        assert [name for name, _thread in calls] == ["one", "two"]
        assert all(thread is not runtime.thread() for _name, thread in calls)
        assert view.review_model.rowCount() == 1
        assert view.review_model.item(0, 0).text() == "two"
        assert view.review_model.item(0, 1).text() == "9007199254740993"
        assert view.review_status_label.text() == "raw.jsonl: 1 rows"
    finally:
        view.dispose_view_resources()
        _close(runtime, parent, release)
        view.deleteLater()
