"""Calibration capture retirement and FTP admission through real UI adapters."""

from pathlib import Path
import threading
import time
from types import SimpleNamespace

import pytest
from PyQt5 import QtCore, QtGui, QtWidgets

import Workstation.calibration_controller as controller_module
from Workstation.app_lifecycle_controller import AppLifecycleController
from Workstation.calibration_controller import CalibrationWorkflowController
from Workstation.calibration_workflow.jobs import CALIBRATION_JOB_FIELDS
from Workstation.tests.test_panel_composition import panels


class HeldCaptureWorker(QtCore.QObject):
    completed = QtCore.pyqtSignal(object)
    failed = QtCore.pyqtSignal(str)
    progress = QtCore.pyqtSignal(object)
    finished = QtCore.pyqtSignal()

    def __init__(self, *, root, frames, stem, **inputs):
        super().__init__()
        self.root, self.frames, self.stem = Path(root), frames, stem
        self.inputs = inputs
        self.saved = threading.Event()
        self.release = threading.Event()
        self.paths = []

    @QtCore.pyqtSlot()
    def run(self):
        try:
            for side, image in self.frames.items():
                directory = self.root / side
                directory.mkdir(parents=True, exist_ok=True)
                path = directory / f"{side}_{self.stem}.png"
                if not image.save(str(path), "PNG"):
                    raise RuntimeError("fixture PNG save failed")
                self.paths.append(path)
            self.progress.emit({"side": "left", "phase": "saved", "state": "saved"})
            self.saved.set()
            if not self.release.wait(3.0):
                raise RuntimeError("fixture result was not released")
            self.completed.emit(
                {
                    "stem": self.stem,
                    "saved_sides": ("left", "right"),
                    "stereo_pair_saved": True,
                    "pair_timing_ok": True,
                    "pair_delta_s": 0.0,
                    "pair_time_basis": "app-receive-monotonic",
                    "status_parts": ("left saved", "right saved"),
                }
            )
        except Exception as exc:
            self.failed.emit(str(exc))
        finally:
            self.finished.emit()


def _until(predicate):
    deadline = time.monotonic() + 3.0
    while not predicate() and time.monotonic() < deadline:
        QtWidgets.QApplication.instance().processEvents()
        QtCore.QThread.msleep(1)
    assert predicate()


def _prepare_panel(panel, monkeypatch, tmp_path):
    panel._measurement_session_hierarchy_enabled = False
    panel.calib_dir_edit.setText(str(tmp_path / "capture"))
    monkeypatch.setattr(controller_module, "CheckerboardCaptureWorker", HeldCaptureWorker)
    frames = {}
    metadata = {}
    now = time.monotonic()
    for side, color in (("left", QtGui.QColor(11, 22, 33)), ("right", QtGui.QColor(44, 55, 66))):
        image = QtGui.QImage(7, 5, QtGui.QImage.Format_RGB888)
        image.fill(color)
        frames[side] = image
        metadata[side] = {
            "timestamp_s": now,
            "native_width": 7,
            "native_height": 5,
            "native_lossless_source": True,
            "overlay_applied": False,
            "frame_id": "fixture-" + side,
        }
    return panel._calibration_workflow_controller_owner, frames, metadata


def _finish(owner):
    if owner.current is not None:
        owner.current.worker.release.set()
    _until(lambda: not owner.busy)


def test_auto_stop_keeps_saved_result_and_admitted_capture_origin(panels, monkeypatch, tmp_path):
    panel = panels()
    controller, frames, metadata = _prepare_panel(panel, monkeypatch, tmp_path)
    controller._auto_capture_active = True
    owner = panel.application.features["calibration"].state.calibration_jobs
    try:
        assert panel.capture_calibration_images(frames=frames, metadata_by_camera=metadata, capture_origin="auto")
        job = owner.current
        assert job.worker.saved.wait(1.0)
        frames["left"].fill(QtGui.QColor(255, 0, 0))
        assert job.worker.frames["left"].pixelColor(0, 0) == QtGui.QColor(11, 22, 33)
        panel.stop_auto_calibration_capture()
        controller._capture_origin = "manual"  # Completion must use its admitted origin.
        state = panel.application.features["calibration"].state
        for name in CALIBRATION_JOB_FIELDS:
            assert name not in vars(state)
            assert getattr(panel, name) is getattr(owner, name)
        assert panel.application.resource_inventory()["checkerboard_calibration_thread"] is job.thread
        panel._handle_checkerboard_capture_failed("untagged late error")
        assert controller._capture_feedback_run["inflight_count"] == 1
        paths = tuple(job.worker.paths)
        _finish(owner)
        assert controller._capture_feedback_last["origin"] == "auto"
        assert controller._capture_feedback_run["stereo_saved_count"] == 1
        assert controller._capture_feedback_run["inflight_count"] == 0
        assert controller._auto_capture_saved_pairs == 0
        assert "중지된 자동 촬영 횟수" in panel.calibration_status_label.text()
        assert all(path.is_file() for path in paths)
        assert QtGui.QImage(str(paths[0])).pixelColor(0, 0) == QtGui.QColor(11, 22, 33)
        assert panel.capture_calibration_button.isEnabled()
    finally:
        _finish(owner)


def test_app_close_consumes_saved_capture_before_retirement_and_keeps_queued_pair(panels, monkeypatch, tmp_path):
    panel = panels()
    controller, frames, metadata = _prepare_panel(panel, monkeypatch, tmp_path)
    controller._pending_ftp_oz_sync_frames = [("first", frames, metadata), ("second", frames, metadata)]
    owner = panel.application.features["calibration"].state.calibration_jobs
    try:
        controller._process_next_ftp_oz_sync_pair()
        job = owner.current
        assert job.worker.saved.wait(1.0)
        paths = tuple(job.worker.paths)
        lifecycle = AppLifecycleController(panel)
        assert lifecycle._shutdown_stop_thread("checkerboard_calibration_thread") is False
        job.worker.release.set()
        assert job.thread.wait(1000)
        # Physical exit precedes delivery of the already-produced result.
        assert lifecycle._shutdown_stop_thread("checkerboard_calibration_thread") is False
        assert panel.checkerboard_calibration_thread is job.thread
        assert controller._capture_feedback_run["inflight_count"] == 1
        _until(lambda: not owner.busy)
        assert controller._capture_feedback_run["stereo_saved_count"] == 1
        assert controller._capture_feedback_run["inflight_count"] == 0
        assert [entry[0] for entry in controller._pending_ftp_oz_sync_frames] == ["second"]
        assert all(path.is_file() for path in paths)
        controller._process_next_ftp_oz_sync_pair()
        assert owner.current is None
        assert [entry[0] for entry in controller._pending_ftp_oz_sync_frames] == ["second"]
    finally:
        _finish(owner)


@pytest.mark.parametrize("partial_start", [False, True])
def test_two_ftp_pairs_start_in_order_after_real_thread_retirement(panels, monkeypatch, tmp_path, partial_start):
    panel = panels()
    controller, frames, metadata = _prepare_panel(panel, monkeypatch, tmp_path)
    controller._pending_ftp_oz_sync_frames = [("first", frames, metadata), ("second", frames, metadata)]
    owner = panel.application.features["calibration"].state.calibration_jobs
    try:
        if partial_start:
            original_start = QtCore.QThread.start

            def start_then_raise(thread):
                original_start(thread)
                raise RuntimeError("start reported error after launch")

            with monkeypatch.context() as patch:
                patch.setattr(QtCore.QThread, "start", start_then_raise)
                controller._process_next_ftp_oz_sync_pair()
        else:
            controller._process_next_ftp_oz_sync_pair()
        first = owner.current
        assert first.worker.saved.wait(1.0)
        first_paths = tuple(first.worker.paths)
        first.worker.release.set()
        _until(lambda: owner.current is not None and owner.current is not first)
        second = owner.current
        assert second.worker.saved.wait(1.0)
        second_paths = tuple(second.worker.paths)
        assert set(first_paths).isdisjoint(second_paths)
        assert controller._pending_ftp_oz_sync_frames == []
        assert controller._ftp_oz_sync_saved_count == 2  # Existing admission counter.
        assert controller._capture_feedback_run["inflight_count"] == 1
        _finish(owner)
        assert controller._capture_feedback_run["stereo_saved_count"] == 2
        assert all(path.is_file() for path in (*first_paths, *second_paths))
        if partial_start:
            assert "after launch" in owner.callback_error
        else:
            assert owner.callback_error == ""
    finally:
        _finish(owner)


def test_failed_thread_start_restores_capture_feedback_and_buttons(panels, monkeypatch, tmp_path):
    panel = panels()
    controller, frames, metadata = _prepare_panel(panel, monkeypatch, tmp_path)
    owner = panel.application.features["calibration"].state.calibration_jobs
    with monkeypatch.context() as patch:
        patch.setattr(QtCore.QThread, "start", lambda thread: None)
        assert panel.capture_calibration_images(frames=frames, metadata_by_camera=metadata) is False
    assert not owner.busy
    assert panel.checkerboard_calibration_thread is None
    assert controller._capture_feedback_run["failed_count"] == 1
    assert controller._capture_feedback_run["inflight_count"] == 0
    assert panel.capture_calibration_button.isEnabled()
    assert panel.run_calibration_button.isEnabled()


def test_ftp_rejected_admission_preserves_head_and_replaced_queue_ignores_old_retry(monkeypatch):
    calls, callbacks = [], []
    accepted = [False]
    panel = SimpleNamespace(
        checkerboard_calibration_thread=None,
        _log=lambda _: None,
        calibration_status_label=SimpleNamespace(setText=lambda _: None),
        capture_calibration_images=lambda **kwargs: calls.append(kwargs) or accepted[0],
    )
    controller = CalibrationWorkflowController(panel)
    first, second = ("first", {"id": 1}, {}), ("second", {"id": 2}, {})
    controller._pending_ftp_oz_sync_frames = [first, second]
    monkeypatch.setattr(QtCore.QTimer, "singleShot", lambda ms, callback: callbacks.append(callback))
    try:
        controller._process_next_ftp_oz_sync_pair()
        assert controller._pending_ftp_oz_sync_frames == [first, second]
        accepted[0] = True
        callbacks.pop(0)()
        assert calls[0]["frames"] is calls[1]["frames"] is first[1]
        assert controller._pending_ftp_oz_sync_frames == [second]
        old_retry = callbacks.pop(0)
        replacement = [("replacement", {}, {})]
        controller._pending_ftp_oz_sync_frames = replacement
        old_retry()
        assert len(calls) == 2
        assert controller._pending_ftp_oz_sync_frames is replacement
        panel._shutdown_in_progress = True
        controller._process_next_ftp_oz_sync_pair()
        assert len(calls) == 2
        assert controller._pending_ftp_oz_sync_frames is replacement
    finally:
        controller._auto_capture_timer.stop()
        controller._auto_capture_timer.deleteLater()
