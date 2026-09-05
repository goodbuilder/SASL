from __future__ import annotations

import time
from pathlib import Path
from types import SimpleNamespace

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


def test_recording_stop_waits_for_method_bundle_barrier() -> None:
    owner = SimpleNamespace(method_evaluation_pending_count=lambda: 2)
    panel = SimpleNamespace(
        _camera_photo_capture_active=lambda: False,
        _measurement_view_controller_owner=owner,
        _method_evaluation_accepting=True,
        _method_evaluation_stop_retry_scheduled=False,
        recording_status_label=_WidgetState(),
        distance_recording_enabled=True,
    )

    RecordingController(panel)._stop_session_recording(restart_streams=False)

    assert panel.distance_recording_enabled is True
    assert panel._method_evaluation_accepting is False
    assert panel._method_evaluation_stop_retry_scheduled is True
    assert "barrier" in panel.recording_status_label.text
    assert panel._recording_stop_retry_timer.isSingleShot()
    assert panel._recording_stop_retry_timer.interval() == 100
    assert panel._recording_stop_retry_request["barrier"] == "method_evaluation"
    panel._recording_stop_retry_timer.stop()


def test_recording_stop_rejects_target_artifact_failure_result() -> None:
    logs: list[str] = []
    saves: list[str] = []
    target_owner = SimpleNamespace(
        target_drone_stream_snapshot=lambda: {
            "phase": "SEALED",
            "worker_alive": False,
            "artifact": {
                "absolute_path": "/session/raw/target.jsonl",
                "size_bytes": 42,
                "sha256": "a" * 64,
            },
        }
    )
    panel = SimpleNamespace(
        pnp_photo_capture_thread=None,
        oz_photo_capture_thread=None,
        _measurement_view_controller_owner=None,
        _native_frame_async_stores={},
        _session_recording_active=lambda: False,
        _workstation_ulog_controller=None,
        flush_workstation_run_evidence=lambda: True,
        _target_drone_controller_owner=target_owner,
        register_workstation_session_artifact=lambda *_args, **_kwargs: {
            "schema": "workstation-persistence-failure-v1",
            "status": "failed",
            "kind": "artifact_registration",
            "error": "target-disk-sentinel",
        },
        recording_status_label=_WidgetState(),
        distance_recording_enabled=True,
        video_recording_enabled=True,
        save_distance_history_csv=lambda **_kwargs: saves.append("saved"),
        _log=logs.append,
    )

    RecordingController(panel)._stop_session_recording(restart_streams=False)

    assert panel.distance_recording_enabled is True
    assert panel.video_recording_enabled is True
    assert saves == []
    assert "target-disk-sentinel" in panel.recording_status_label.text
    assert any("target-disk-sentinel" in message for message in logs)


def test_sessionless_manual_stop_releases_owner_and_never_waits_for_save() -> None:
    calls: list[str] = []
    events: list[tuple[str, dict[str, object]]] = []
    video_path = Path("/tmp/unvalidated-sessionless.mp4")
    panel = SimpleNamespace(
        _measurement_session_hierarchy_enabled=False,
        _method_evaluation_accepting=True,
        distance_recording_enabled=True,
        video_recording_enabled=True,
        distance_recording_started_monotonic_s=1.0,
        _manual_recording_active=True,
        _recording_mode="manual",
        recording_session_id="manual-generation",
        distance_recording_rows=[],
        camera_recording_rows={"left": [], "right": []},
        px4_recording_rows={"left": [], "right": []},
        gnss_base_recording_rows=[],
        extrinsics_comparison_recording_rows=[],
        method_evaluation_bundle_rows=[],
        video_recording_paths={"left": video_path},
        stream_backend_combo=SimpleNamespace(currentData=lambda: "gstreamer"),
        left_pane=SimpleNamespace(
            recording_finalizations={
                str(video_path): {
                    "success": True,
                    "detail": "EOS received; mp4mux finalized",
                    "recording_encoding_mode": "h264_copy_integrity_gated",
                }
            }
        ),
        record_button=_WidgetState(),
        camera_connected=False,
        append_workstation_data=lambda stream, payload, **_kwargs: events.append(
            (stream, dict(payload))
        ),
        flush_workstation_data=lambda: calls.append("flush"),
        _sync_rtcm_evidence_context=lambda: calls.append("rtcm-context"),
        _resume_queued_measurement_start=lambda: calls.append("resume"),
        _refresh_recording_status_label=lambda: calls.append("status"),
        _refresh_operation_header=lambda: calls.append("header"),
        _log=lambda _message: None,
    )

    RecordingController(panel)._stop_session_recording(restart_streams=False)

    assert panel.distance_recording_enabled is False
    assert panel.video_recording_enabled is False
    assert panel._manual_recording_active is False
    assert calls[:3] == ["rtcm-context", "flush", "resume"]
    assert any(
        stream == "events" and payload["event_type"] == "recording_stopped"
        for stream, payload in events
    )
    video_closed = next(
        payload
        for stream, payload in events
        if stream == "artifacts" and payload["event_type"] == "video_closed"
    )
    assert video_closed["container_finalization_success"] is True
    assert video_closed["finalization_state"] == "failed"
    assert video_closed["integrity_validation_status"] == "FAIL"
    assert video_closed["recording_integrity_verified"] is False
    assert video_closed["recording_usable"] is False
    assert video_closed["integrity_failure_reasons"] == [
        "integrity_validation_schema_invalid"
    ]


def test_sessionless_stop_infers_decoded_nvenc_finalization_when_signal_is_lost(
    tmp_path: Path,
) -> None:
    calls: list[str] = []
    events: list[tuple[str, dict[str, object]]] = []
    video_path = tmp_path / "decoded-sessionless.mp4"
    video_path.write_bytes(b"finalized-decoded-mp4")
    panel = SimpleNamespace(
        _measurement_session_hierarchy_enabled=False,
        _method_evaluation_accepting=True,
        distance_recording_enabled=True,
        video_recording_enabled=True,
        distance_recording_started_monotonic_s=1.0,
        _manual_recording_active=True,
        _recording_mode="manual",
        recording_session_id="manual-generation",
        distance_recording_rows=[],
        camera_recording_rows={"left": [], "right": []},
        px4_recording_rows={"left": [], "right": []},
        gnss_base_recording_rows=[],
        extrinsics_comparison_recording_rows=[],
        method_evaluation_bundle_rows=[],
        video_recording_paths={"left": video_path},
        stream_backend_combo=SimpleNamespace(currentData=lambda: "gstreamer"),
        left_pane=SimpleNamespace(recording_finalizations={}),
        record_button=_WidgetState(),
        camera_connected=False,
        append_workstation_data=lambda stream, payload, **_kwargs: events.append(
            (stream, dict(payload))
        ),
        flush_workstation_data=lambda: calls.append("flush"),
        _sync_rtcm_evidence_context=lambda: calls.append("rtcm-context"),
        _resume_queued_measurement_start=lambda: calls.append("resume"),
        _refresh_recording_status_label=lambda: calls.append("status"),
        _refresh_operation_header=lambda: calls.append("header"),
        _log=lambda _message: None,
    )

    RecordingController(panel)._stop_session_recording(restart_streams=False)

    video_closed = next(
        payload
        for stream, payload in events
        if stream == "artifacts" and payload["event_type"] == "video_closed"
    )
    assert video_closed["recording_encoding_mode"] == (
        "decoded_nvenc_lossless_reencode"
    )
    assert video_closed["container_finalization_success"] is True
    assert video_closed["container_finalization_inferred"] is True
    assert video_closed["finalization_state"] == "success"
    assert video_closed["integrity_validation_status"] == "PENDING"
    assert video_closed["recording_integrity_verified"] is False
    assert video_closed["recording_usable"] is True
    assert video_closed["recording_file_integrity_verified"] is False
    assert video_closed["source_media_integrity_verified"] is False
    assert video_closed["integrity_failure_reasons"] == [
        "finalized_mp4_output_validation_pending"
    ]


def test_measurement_camera_rejoin_allocates_new_non_overwriting_segment(
    tmp_path: Path,
) -> None:
    artifacts: list[dict[str, object]] = []
    transitions: list[tuple[str, str, str]] = []
    panel = SimpleNamespace(
        recording_session_id="run-observation",
        _measurement_run_generation="run-observation",
        allocate_workstation_data_artifact=lambda category, **kwargs: (
            tmp_path / category / f"{kwargs['label']}{kwargs['suffix']}"
        ),
        append_workstation_data=lambda _stream, payload, **_kwargs: artifacts.append(
            dict(payload)
        ),
        measurement_source_transition=lambda source, state, detail: transitions.append(
            (source, state, detail)
        ),
        _log=lambda _message: None,
    )
    controller = RecordingController(panel)

    first = controller.allocate_measurement_video_segment("left", utc_ns=1)
    second = controller.allocate_measurement_video_segment("left", utc_ns=2)

    assert first is not None and second is not None and first != second
    assert "segment-01" in first.name
    assert "segment-02" in second.name
    first.parent.mkdir(parents=True, exist_ok=True)
    first.touch()
    row = controller.record_measurement_video_segment_presence(
        "left", first, runtime_generation="run-observation"
    )
    assert row["file_present"] is True
    assert controller.measurement_video_segments_snapshot()["state"] == "PARTIAL"
    assert [item["segment_index"] for item in panel._measurement_video_segments["left"]] == [1, 2]
    assert artifacts[0]["runtime_generation"] == "run-observation"
    assert transitions[0][:2] == ("recording.left", "WAITING_RESOURCE")


def test_sessionless_stop_finishes_when_optional_persistence_boundaries_raise(
    tmp_path: Path,
) -> None:
    class BrokenRows:
        def close(self) -> None:
            raise OSError("injected row close failure")

    panel = SimpleNamespace(
        _measurement_session_hierarchy_enabled=False,
        _method_evaluation_accepting=True,
        distance_recording_enabled=True,
        video_recording_enabled=False,
        distance_recording_started_monotonic_s=1.0,
        _manual_recording_active=True,
        _recording_mode="manual",
        recording_session_id="run-best-effort",
        distance_recording_rows=BrokenRows(),
        camera_recording_rows={},
        px4_recording_rows={},
        gnss_base_recording_rows=BrokenRows(),
        extrinsics_comparison_recording_rows=BrokenRows(),
        method_evaluation_bundle_rows=BrokenRows(),
        video_recording_paths={},
        record_button=_WidgetState(),
        camera_connected=False,
        append_workstation_data=lambda *_args, **_kwargs: (_ for _ in ()).throw(
            OSError("injected append failure")
        ),
        flush_workstation_data=lambda: (_ for _ in ()).throw(
            OSError("injected flush failure")
        ),
        _sync_rtcm_evidence_context=lambda: (_ for _ in ()).throw(
            OSError("injected sync failure")
        ),
        _resume_queued_measurement_start=lambda: None,
        _refresh_recording_status_label=lambda: None,
        _refresh_operation_header=lambda: None,
        _log=lambda _message: None,
    )

    RecordingController(panel)._stop_session_recording(restart_streams=False)

    assert panel.distance_recording_enabled is False
    assert panel.video_recording_enabled is False
    assert panel._manual_recording_active is False
    assert panel.recording_session_id == ""
    assert panel._recording_mode == "idle"


def test_sessionless_row_setup_falls_back_to_memory_when_writer_setup_fails() -> None:
    class BrokenStorage:
        @property
        def store(self):
            raise OSError("unwritable data root")

    logs: list[str] = []
    panel = SimpleNamespace(
        _data_storage_controller=lambda: BrokenStorage(),
        _log=logs.append,
    )

    rows = RecordingController(panel)._new_live_rows("measurements", priority=True)
    rows.append({"result": "VALID"})

    assert list(rows) == [{"result": "VALID"}]
    assert any("Measurement continues" not in value and "측정 계속" in value for value in logs)
