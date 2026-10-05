from __future__ import annotations

from pathlib import Path

import pytest
from PyQt5 import QtGui, QtWidgets


@pytest.fixture
def readiness_panel(monkeypatch: pytest.MonkeyPatch, tmp_path: Path):
    from Workstation import standalone_gui

    monkeypatch.setattr(
        standalone_gui.WirisStereoDistancePanel,
        "_initial_status",
        lambda self: None,
    )
    monkeypatch.setattr(
        standalone_gui.WirisStereoDistancePanel,
        "_auto_load_intrinsics",
        lambda self: None,
    )
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    panel = standalone_gui.WirisStereoDistancePanel(app_root=tmp_path)
    callbacks: list[tuple[int, object]] = []
    monkeypatch.setattr(
        standalone_gui.QtCore.QTimer,
        "singleShot",
        lambda delay_ms, callback: callbacks.append((int(delay_ms), callback)),
    )
    panel.left_url_edit.setText("rtsp://left/visible")
    panel.right_url_edit.setText("rtsp://right/visible")
    panel.left_pane.stop = lambda *, wait=False: True
    panel.right_pane.stop = lambda *, wait=False: True

    def start(pane):
        def _start(_url: str, **_kwargs: object) -> bool:
            pane.stream_generation += 1
            return True

        return _start

    panel.left_pane.start_stream = start(panel.left_pane)
    panel.right_pane.start_stream = start(panel.right_pane)
    yield panel, callbacks
    cancel = getattr(panel, "_cancel_camera_start_readiness", None)
    if callable(cancel):
        cancel()
    cancel = getattr(panel, "_cancel_video_start_readiness", None)
    if callable(cancel):
        cancel(mark_idle=True)
    panel._shutdown_coordinator = type("Done", (), {"run": lambda self: True})()
    panel.close()
    panel.deleteLater()
    app.processEvents()


def _frame_metadata(panel, side: str) -> dict[str, object]:
    pane = panel.left_pane if side == "left" else panel.right_pane
    return {
        "camera_id": side,
        "stream_generation": pane.stream_generation,
        "frame_id": 1,
        "timestamp_s": 1.0,
    }


def test_qthread_start_waits_for_current_generation_frames_before_running(
    readiness_panel,
) -> None:
    panel, _callbacks = readiness_panel
    image = QtGui.QImage(16, 9, QtGui.QImage.Format_RGB888)
    assert panel.begin_measurement_run()

    assert panel._start_camera_streams(
        enable_detection=True,
        switch_to_depth=False,
    )

    assert panel.camera_connected is False
    assert panel.yolo_enabled is False
    assert panel.measurement_run_snapshot()["state"] == "RUNNING"

    panel.left_pane.frame_changed.emit(
        "left", image, _frame_metadata(panel, "left")
    )
    assert panel.camera_connected is False
    assert panel.measurement_run_snapshot()["state"] == "RUNNING"

    stale_right = _frame_metadata(panel, "right")
    stale_right["stream_generation"] = int(stale_right["stream_generation"]) - 1
    panel.right_pane.frame_changed.emit("right", image, stale_right)
    assert panel.measurement_run_snapshot()["state"] == "RUNNING"

    panel.right_pane.frame_changed.emit(
        "right", image, _frame_metadata(panel, "right")
    )
    snapshot = panel.measurement_run_snapshot()
    assert snapshot["state"] == "RUNNING"
    assert snapshot["active_sides"] == ["left", "right"]
    assert panel.camera_connected is True
    assert panel.yolo_enabled is True


def test_invalid_camera_configuration_does_not_reject_observation_run(
    readiness_panel,
) -> None:
    panel, _callbacks = readiness_panel
    panel.left_url_edit.setText("")

    assert panel.start_yolo() is None

    snapshot = panel.measurement_run_snapshot()
    assert snapshot["state"] == "RUNNING"
    assert snapshot["sources"]["camera.left"]["state"] == "UNAVAILABLE"
    assert snapshot["sources"]["camera.right"]["state"] == "UNAVAILABLE"


def test_recording_start_failure_does_not_reject_camera_observation(
    readiness_panel,
) -> None:
    panel, _callbacks = readiness_panel
    starts: list[dict[str, object]] = []
    panel.start_automatic_session_recording = lambda: False
    panel._start_camera_streams = lambda **kwargs: starts.append(dict(kwargs)) or True

    assert panel.start_yolo() is None

    snapshot = panel.measurement_run_snapshot()
    assert snapshot["state"] == "RUNNING"
    assert snapshot["sources"]["recording.left"]["state"] == "UNAVAILABLE"
    assert snapshot["sources"]["recording.right"]["state"] == "UNAVAILABLE"
    assert starts == [{"enable_detection": True, "switch_to_depth": True}]


def test_zero_camera_observation_can_stop_and_immediately_restart(
    readiness_panel,
) -> None:
    panel, _callbacks = readiness_panel
    panel.left_url_edit.setText("")
    assert panel.start_yolo() is None
    first = panel.measurement_run_snapshot()["generation"]
    assert panel.camera_connected is False

    panel.stop_yolo_detection()

    assert panel.measurement_run_snapshot()["state"] == "IDLE"
    assert panel.start_yolo() is None
    assert panel.measurement_run_snapshot()["state"] == "RUNNING"
    assert panel.measurement_run_snapshot()["generation"] != first


def test_first_frame_timeout_keeps_ready_side_as_partial_measurement(
    readiness_panel,
) -> None:
    panel, _callbacks = readiness_panel
    image = QtGui.QImage(16, 9, QtGui.QImage.Format_RGB888)
    assert panel.begin_measurement_run()
    assert panel._start_camera_streams(enable_detection=True, switch_to_depth=False)
    panel._camera_stream_frame_ready("left", image, _frame_metadata(panel, "left"))
    stopped: list[str] = []
    panel.right_pane.stop = lambda *, wait=False: stopped.append("right") or True
    readiness = panel._camera_start_readiness
    readiness["deadline_monotonic_s"] = 0.0

    panel._poll_camera_start_readiness(int(readiness["generation"]))

    snapshot = panel.measurement_run_snapshot()
    assert snapshot["state"] == "RUNNING"
    assert snapshot["active_sides"] == ["left"]
    assert panel.camera_connected is True
    assert stopped == ["right"]
    assert "partial" in panel.camera_connection_label.text().lower()


def test_fatal_source_start_status_keeps_only_the_frame_ready_side(
    readiness_panel,
) -> None:
    panel, _callbacks = readiness_panel
    image = QtGui.QImage(16, 9, QtGui.QImage.Format_RGB888)
    assert panel.begin_measurement_run()
    assert panel._start_camera_streams(enable_detection=True, switch_to_depth=False)
    panel._camera_stream_frame_ready("left", image, _frame_metadata(panel, "left"))

    panel._camera_stream_start_status(
        "right",
        "GStreamer 오류: failed to enter PLAYING state",
    )

    snapshot = panel.measurement_run_snapshot()
    assert snapshot["state"] == "RUNNING"
    assert snapshot["active_sides"] == ["left"]
    assert panel.camera_connected is True
    assert "right unavailable" in panel.camera_connection_label.text()


def test_worker_replacement_timeout_keeps_observation_running(
    readiness_panel,
) -> None:
    panel, callbacks = readiness_panel
    assert panel.begin_measurement_run()
    panel._automatic_session_recording = True
    panel.distance_recording_enabled = True
    panel.video_recording_enabled = False
    panel.left_pane.stop = lambda *, wait=False: False
    panel.right_pane.stop = lambda *, wait=False: False

    assert panel._start_camera_streams(enable_detection=True, switch_to_depth=False)
    assert panel.measurement_run_snapshot()["state"] == "RUNNING"
    panel._camera_start_retry_deadline_s = 0.0

    _delay_ms, retry = callbacks.pop(0)
    retry()

    assert panel.measurement_run_snapshot()["state"] == "RUNNING"
    assert panel.measurement_run_snapshot()["sources"]["camera.left"]["state"] == "UNAVAILABLE"
    assert panel._automatic_session_recording is False
    assert panel.camera_connected is False


def test_no_first_frame_stops_recording_but_keeps_observation_running(
    readiness_panel,
) -> None:
    panel, _callbacks = readiness_panel
    assert panel.begin_measurement_run()
    panel._automatic_session_recording = True
    panel._recording_mode = "automatic_detection"
    panel.distance_recording_enabled = True
    panel.video_recording_enabled = True
    panel.video_recording_paths = {
        "left": panel.app_root / "left.mp4",
        "right": panel.app_root / "right.mp4",
    }
    panel._video_recording_allocation_sizes = {"left": 0, "right": 0}
    assert panel._start_camera_streams(enable_detection=True, switch_to_depth=False)
    readiness = panel._camera_start_readiness
    readiness["deadline_monotonic_s"] = 0.0

    panel._poll_camera_start_readiness(int(readiness["generation"]))

    snapshot = panel.measurement_run_snapshot()
    assert snapshot["state"] == "RUNNING"
    assert snapshot["sources"]["camera.left"]["state"] == "UNAVAILABLE"
    assert panel.camera_connected is False
    assert panel._automatic_session_recording is False
    assert panel.distance_recording_enabled is False
    assert panel.video_recording_enabled is False
    assert panel.begin_measurement_run() is False


def test_dual_decoder_start_rejection_stops_automatic_recording(
    readiness_panel,
) -> None:
    panel, _callbacks = readiness_panel
    assert panel.begin_measurement_run()
    panel._automatic_session_recording = True
    panel._recording_mode = "automatic_detection"
    panel.distance_recording_enabled = True
    panel.video_recording_enabled = True
    panel.video_recording_paths = {
        "left": panel.app_root / "left.mp4",
        "right": panel.app_root / "right.mp4",
    }
    panel._video_recording_allocation_sizes = {"left": 0, "right": 0}
    assert panel._start_camera_streams(enable_detection=True, switch_to_depth=False)

    panel._handle_runtime_decoder_fallback(
        "left",
        "DECODER_FALLBACK_REQUIRED: nvh264dec first-frame timeout",
    )
    assert panel.measurement_run_snapshot()["state"] == "RUNNING"
    panel._handle_runtime_decoder_fallback(
        "right",
        "DECODER_FALLBACK_REQUIRED: nvh264dec first-frame timeout",
    )

    assert panel.measurement_run_snapshot()["state"] == "RUNNING"
    assert panel._automatic_session_recording is False
    assert panel.distance_recording_enabled is False
    assert panel.video_recording_enabled is False
    assert panel._runtime_decoder_override is None
    assert panel._runtime_decoder_overrides == {
        "left": "software",
        "right": "software",
    }


def test_video_growth_is_independent_from_measurement_running(
    readiness_panel,
) -> None:
    panel, _callbacks = readiness_panel
    image = QtGui.QImage(16, 9, QtGui.QImage.Format_RGB888)
    left_path = panel.app_root / "left.mp4"
    right_path = panel.app_root / "right.mp4"
    panel.distance_recording_enabled = True
    panel.video_recording_enabled = True
    panel.video_recording_paths = {"left": left_path, "right": right_path}
    panel._video_recording_allocation_sizes = {"left": 0, "right": 0}
    assert panel.begin_measurement_run()
    assert panel._start_camera_streams(enable_detection=True, switch_to_depth=False)

    panel._camera_stream_frame_ready("left", image, _frame_metadata(panel, "left"))
    panel._camera_stream_frame_ready("right", image, _frame_metadata(panel, "right"))

    assert panel.measurement_run_snapshot()["state"] == "RUNNING"
    assert panel.video_recording_start_snapshot()["state"] == "STARTING"

    left_path.write_bytes(b"left-video")
    right_path.write_bytes(b"right-video")
    video_readiness = panel._video_start_readiness
    panel._poll_video_start_readiness(int(video_readiness["generation"]))

    video = panel.video_recording_start_snapshot()
    assert video["state"] == "RUNNING"
    assert video["sides"]["left"]["size_bytes"] > 0
    assert video["sides"]["right"]["size_bytes"] > 0


def test_video_timeout_degrades_recording_without_reverting_measurement(
    readiness_panel,
) -> None:
    panel, _callbacks = readiness_panel
    image = QtGui.QImage(16, 9, QtGui.QImage.Format_RGB888)
    panel.distance_recording_enabled = True
    panel.video_recording_enabled = True
    panel.video_recording_paths = {
        "left": panel.app_root / "left.mp4",
        "right": panel.app_root / "right.mp4",
    }
    panel._video_recording_allocation_sizes = {"left": 0, "right": 0}
    assert panel.begin_measurement_run()
    assert panel._start_camera_streams(enable_detection=True, switch_to_depth=False)
    panel._camera_stream_frame_ready("left", image, _frame_metadata(panel, "left"))
    panel._camera_stream_frame_ready("right", image, _frame_metadata(panel, "right"))
    assert panel.measurement_run_snapshot()["state"] == "RUNNING"
    video_readiness = panel._video_start_readiness
    video_readiness["deadline_monotonic_s"] = 0.0

    panel._poll_video_start_readiness(int(video_readiness["generation"]))

    assert panel.measurement_run_snapshot()["state"] == "RUNNING"
    assert panel.video_recording_start_snapshot()["state"] == "FAILED"
    assert panel.video_recording_enabled is False
    assert "video=FAILED" in panel.recording_status_label.text()


def test_one_video_writer_failure_is_partial_and_keeps_measurement_running(
    readiness_panel,
) -> None:
    panel, _callbacks = readiness_panel
    image = QtGui.QImage(16, 9, QtGui.QImage.Format_RGB888)
    left_path = panel.app_root / "left.mp4"
    panel.distance_recording_enabled = True
    panel.video_recording_enabled = True
    panel.video_recording_paths = {
        "left": left_path,
        "right": panel.app_root / "right.mp4",
    }
    panel._video_recording_allocation_sizes = {"left": 0, "right": 0}
    assert panel.begin_measurement_run()
    assert panel._start_camera_streams(enable_detection=True, switch_to_depth=False)
    panel._camera_stream_frame_ready("left", image, _frame_metadata(panel, "left"))
    panel._camera_stream_frame_ready("right", image, _frame_metadata(panel, "right"))
    left_path.write_bytes(b"left-video")

    panel._camera_stream_start_status(
        "right",
        "영상 녹화 파일 열기 실패: permission denied",
    )

    assert panel.measurement_run_snapshot()["state"] == "RUNNING"
    video = panel.video_recording_start_snapshot()
    assert video["state"] == "PARTIAL"
    assert video["sides"]["left"]["state"] == "RUNNING"
    assert video["sides"]["right"]["state"] == "FAILED"
    assert panel.video_recording_enabled is True


def test_explicit_start_rejection_keeps_measurement_and_stops_auto_recording() -> None:
    from types import SimpleNamespace

    from Workstation.camera_stream_controller import CameraStreamLifecycleController
    from Workstation.measurement_run_controller import MeasurementRunController

    calls: list[object] = []
    panel = SimpleNamespace(
        _manual_recording_active=False,
        _automatic_session_recording=False,
        _pending_camera_start=None,
        camera_connected=False,
        yolo_enabled=False,
        _measurement_view_controller_owner=None,
        _measurement_view_controller=lambda: None,
        retry_data_storage=lambda: None,
        append_workstation_data=lambda *_args, **_kwargs: True,
        flush_workstation_data=lambda: True,
        data_storage_snapshot=lambda: {"state": "OK", "streams": {}},
        _refresh_operation_header=lambda: None,
        _session_recording_active=lambda: False,
        start_automatic_session_recording=lambda: (
            setattr(panel, "_automatic_session_recording", True) or True
        ),
        stop_automatic_session_recording=lambda **_kwargs: (
            calls.append("recording.stop")
            or setattr(panel, "_automatic_session_recording", False)
            or True
        ),
        _start_camera_streams=lambda **_kwargs: False,
        stop_yolo_streams=lambda **_kwargs: calls.append("streams.stop") or True,
        _log=lambda message: calls.append(("log", message)),
    )
    run = MeasurementRunController(panel)
    panel.begin_measurement = run.begin_measurement
    panel.measurement_run_snapshot = run.measurement_run_snapshot
    panel.cancel_measurement_detection_start = run.cancel_measurement_detection_start
    panel.fail_measurement_start = run.fail_measurement_start

    result = CameraStreamLifecycleController(panel).start_yolo()

    assert result is None
    assert run.state == "RUNNING"
    assert panel._automatic_session_recording is False
    assert calls[0] == "recording.stop"
    assert "streams.stop" not in calls
