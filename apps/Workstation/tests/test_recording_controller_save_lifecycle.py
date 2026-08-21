from __future__ import annotations

import time
from pathlib import Path

import pytest
from PyQt5 import QtCore

from Workstation import session_workers
from Workstation.recording_controller import RecordingController
from Workstation.workstation_core.recording_session import SpoolRows


class _WidgetState:
    def __init__(self) -> None:
        self.enabled = True
        self.text = ""

    def setEnabled(self, value: bool) -> None:  # noqa: N802 - Qt-compatible double
        self.enabled = bool(value)

    def setText(self, value: str) -> None:  # noqa: N802 - Qt-compatible double
        self.text = str(value)


class _SavePanel(QtCore.QObject):
    def __init__(self, root: Path) -> None:
        super().__init__()
        self.root = root
        self.distance_recording_enabled = False
        self.recording_session_id = "session-save-controller"
        self.distance_recording_rows = [
            {
                "sample_index": 1,
                "session_elapsed_s": 1.25,
                "distance_m": 12.5,
                "measurement_valid": True,
                "sentinel": "controller-real-worker",
            }
        ]
        self.camera_recording_rows = {"left": [], "right": []}
        self.px4_recording_rows = {"left": [], "right": []}
        self.gnss_base_recording_rows = []
        self.extrinsics_comparison_recording_rows = []
        self._session_save_thread = None
        self._session_save_worker = None
        self._session_save_finalized = False
        self._session_save_retry_required = False
        self._session_save_shutdown_wait_started_s = None
        self._session_save_shutdown_timeout_logged = False
        self._recovered_session_output_dir = None
        self._recovered_session_metadata = None
        self.record_button = _WidgetState()
        self.save_history_button = _WidgetState()
        self.recording_status_label = _WidgetState()
        self.logs: list[str] = []
        self.controller: RecordingController | None = None

    def _recording_session_dir(self, session_id: str) -> Path:
        return self.root / "recordings" / session_id

    def _recording_session_metadata(self, **_kwargs: object) -> dict[str, object]:
        return {
            "session_id": self.recording_session_id,
            "measurement_id": "m000001",
            "session_started_unix_ns": 1_700_000_000_000_000_000,
        }

    def _recording_optical_pose_session_data(self) -> dict[str, object]:
        return {"state": "NO DATA"}

    def _log(self, message: str) -> None:
        self.logs.append(str(message))

    def _refresh_operation_header(self) -> None:
        return None

    @QtCore.pyqtSlot(object)
    def _handle_session_saved(self, saved_files: object) -> None:
        assert self.controller is not None
        self.controller._handle_session_saved(saved_files)

    @QtCore.pyqtSlot(str)
    def _handle_session_save_failed(self, error: str) -> None:
        assert self.controller is not None
        self.controller._handle_session_save_failed(error)

    @QtCore.pyqtSlot()
    def _handle_session_save_thread_finished(self) -> None:
        assert self.controller is not None
        self.controller._handle_session_save_thread_finished()


def _drain_qt_events_until(predicate, *, timeout_s: float = 5.0) -> bool:
    app = QtCore.QCoreApplication.instance() or QtCore.QCoreApplication([])
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        app.processEvents(QtCore.QEventLoop.AllEvents, 25)
        if predicate():
            return True
        QtCore.QThread.msleep(5)
    return bool(predicate())


def test_save_controller_owns_real_worker_thread_with_panel_parent(tmp_path: Path) -> None:
    panel = _SavePanel(tmp_path)
    controller = RecordingController(panel)
    panel.controller = controller

    controller.save_distance_history_csv()

    thread = panel._session_save_thread
    assert isinstance(thread, QtCore.QThread)
    assert thread.parent() is panel
    assert panel._session_save_worker is not None
    assert _drain_qt_events_until(lambda: panel._session_save_thread is None)
    assert panel._session_save_finalized is True
    assert panel._session_save_retry_required is False
    assert (panel._recording_session_dir(panel.recording_session_id) / "measurement.json").is_file()


def test_real_save_worker_failure_preserves_spool_and_sets_retry(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    panel = _SavePanel(tmp_path)
    output_dir = panel._recording_session_dir(panel.recording_session_id)
    spool = SpoolRows(preview_limit=8, supports_updates=True)
    spool.bind(output_dir / ".spool" / "pair_rows.jsonl")
    spool.append(
        {
            "sample_index": 7,
            "measurement_valid": False,
            "sentinel": "save-failure-spool",
        }
    )
    spool.flush()
    before = spool.spool_path.read_bytes()
    panel.distance_recording_rows = spool
    controller = RecordingController(panel)
    panel.controller = controller

    def fail_save(*_args: object, **_kwargs: object) -> object:
        raise OSError("injected persistence failure")

    monkeypatch.setattr(session_workers, "save_session_files", fail_save)
    controller.save_distance_history_csv()

    assert _drain_qt_events_until(lambda: panel._session_save_thread is None)
    assert panel._session_save_retry_required is True
    assert panel._session_save_finalized is False
    assert panel._session_save_last_error == "injected persistence failure"
    assert spool.spool_path.read_bytes() == before
    assert "injected persistence failure" in panel.recording_status_label.text
