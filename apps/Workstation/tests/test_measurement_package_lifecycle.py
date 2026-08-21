from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

from Workstation.camera_stream_controller import CameraStreamLifecycleController
from Workstation.measurement_package_controller import MeasurementPackageController
from Workstation.workstation_core.recording_session import save_session_files


class _Label:
    def __init__(self) -> None:
        self.value = ""

    def setText(self, value: str) -> None:  # noqa: N802 - Qt-compatible double
        self.value = str(value)


def _panel(tmp_path: Path) -> SimpleNamespace:
    logs: list[str] = []
    panel = SimpleNamespace(
        recording_session_id="",
        _measurement_package_explicit_lifecycle=True,
        _measurement_package_collect_provenance=False,
        _measurement_package_verify_media=False,
        _session_save_thread=None,
        _session_save_finalized=False,
        _session_save_retry_required=False,
        _automatic_session_recording=False,
        _recording_mode="manual",
        _pnp_session_prepared=False,
        pnp_photo_capture_records=[],
        oz_photo_capture_records=[],
        distance_recording_rows=[],
        camera_connected=True,
        _pending_camera_start=None,
        pnp_photo_capture_thread=None,
        oz_photo_capture_thread=None,
        _session_recording_active=lambda: False,
        _recording_session_dir=lambda session_id: tmp_path / "recordings" / session_id,
        _log=logs.append,
    )
    panel.logs = logs
    return panel


def _write_required_recording_outputs(panel: SimpleNamespace) -> Path:
    root = panel._recording_session_dir(panel.recording_session_id)
    (root / "video").mkdir(parents=True, exist_ok=True)
    (root / "video" / "raw_left.mp4").write_bytes(b"left-mp4")
    (root / "video" / "raw_right.mp4").write_bytes(b"right-mp4")
    save_session_files(
        root,
        metadata={"session_id": panel.recording_session_id},
        pair_rows=[],
        camera_rows={"left": [], "right": []},
        px4_rows={"left": [], "right": []},
    )
    panel._session_save_finalized = True
    return root


def _begin_and_stop_detection(controller: MeasurementPackageController) -> None:
    assert controller.begin_measurement_session() is True
    assert controller.begin_measurement_detection() is True
    assert controller.complete_measurement_detection() is True


def test_explicit_begin_allocates_one_session_and_one_detection_interval(tmp_path: Path) -> None:
    panel = _panel(tmp_path)
    applied_identity: list[dict[str, object]] = []
    panel.apply_workstation_run_setup = lambda values: applied_identity.append(dict(values))
    controller = MeasurementPackageController(panel)

    assert controller.begin_measurement_session() is True
    first = controller.measurement_package_snapshot()
    assert first["state"] == "COLLECTING"
    assert first["session_id"]
    assert first["revision"] == "r0001"
    assert panel._pnp_session_prepared is True
    assert applied_identity == [
        {
            "experiment_id": "unassigned",
            "run_id": first["session_id"],
            "condition_id": "unspecified",
            "mode": "drone",
            "condition": {
                "identity_source": "automatic_session_identity",
                "operator_metadata": "not_collected",
            },
        }
    ]

    assert controller.begin_measurement_detection() is True
    assert controller.begin_measurement_detection() is False
    assert controller.complete_measurement_detection() is True
    assert controller.measurement_package_snapshot()["state"] == "POST_CAPTURE"
    assert not (Path(first["session_dir"]) / "revisions" / "r0001" / "package_manifest.json").exists()


def test_finalize_waits_for_session_save_then_seals_complete(tmp_path: Path) -> None:
    panel = _panel(tmp_path)
    controller = MeasurementPackageController(panel)
    _begin_and_stop_detection(controller)
    _write_required_recording_outputs(panel)
    panel._session_save_thread = object()
    controller._schedule_finalize_retry = lambda: None  # type: ignore[method-assign]

    assert controller.finalize_measurement_package() is False
    assert controller.measurement_package_snapshot()["state"] == "VERIFYING"
    root = panel._recording_session_dir(panel.recording_session_id)
    assert not (root / "revisions" / "r0001" / "package_manifest.json").exists()

    panel._session_save_thread = None
    assert controller._continue_measurement_package_finalization() is True
    snapshot = controller.measurement_package_snapshot()
    assert snapshot["state"] == "SEALED_COMPLETE"
    assert (root / "measurement.json").is_symlink()
    assert any(
        row.get("role") == "session/finalization_manifest"
        and row.get("status") == "present"
        for row in snapshot["artifacts"]
    )
    assert controller.verify_measurement_package()["valid"] is True


def test_active_px4_and_base_gnss_accept_partitioned_parquet_evidence(
    tmp_path: Path,
) -> None:
    panel = _panel(tmp_path)
    controller = MeasurementPackageController(panel)
    _begin_and_stop_detection(controller)
    root = panel._recording_session_dir(panel.recording_session_id)
    (root / "video").mkdir(parents=True)
    (root / "video" / "raw_left.mp4").write_bytes(b"left")
    (root / "video" / "raw_right.mp4").write_bytes(b"right")
    controller.activate_measurement_package_source(
        "px4.left", required_roles=("telemetry/px4/left/parquet",)
    )
    controller.activate_measurement_package_source(
        "gnss.base", required_roles=("telemetry/gnss/base/parquet",)
    )
    save_session_files(
        root,
        metadata={"session_id": panel.recording_session_id},
        pair_rows=[],
        camera_rows={"left": [], "right": []},
        px4_rows={
            "left": [{"sequence": 1, "sample_type": "attitude", "roll_deg": 1.0}],
            "right": [],
        },
        gnss_base_rows=[{"sequence": 1, "latitude_deg": 37.0, "fixed": True}],
    )
    panel._session_save_finalized = True

    assert controller.finalize_measurement_package() is True
    snapshot = controller.measurement_package_snapshot()
    assert snapshot["state"] == "SEALED_COMPLETE"
    roles = {
        row["role"]
        for row in snapshot["artifacts"]
        if row.get("status") == "present"
    }
    assert "telemetry/px4/left/parquet" in roles
    assert "telemetry/gnss/base/parquet" in roles


def test_requested_hr_pair_without_right_original_seals_incomplete_with_reason(
    tmp_path: Path,
) -> None:
    panel = _panel(tmp_path)
    controller = MeasurementPackageController(panel)
    _begin_and_stop_detection(controller)
    root = _write_required_recording_outputs(panel)
    left = root / "frames" / "hr_capture" / "left" / "hr-001-left.jpg"
    left.parent.mkdir(parents=True, exist_ok=True)
    left.write_bytes(b"exact-left")
    panel.pnp_photo_capture_records = [
        {
            "capture_id": "hr-001",
            "state": "partial_or_failed",
            "cameras": {
                "left": {
                    "state": "completed",
                    "ftp_downloaded": True,
                    "ftp_local_path": str(left),
                },
                "right": {
                    "state": "timeout",
                    "ftp_downloaded": False,
                    "ftp_error": "right FTP timed out",
                },
            },
        }
    ]

    assert controller.finalize_measurement_package() is True
    snapshot = controller.measurement_package_snapshot()
    assert snapshot["state"] == "SEALED_INCOMPLETE"
    manifest = json.loads(
        (root / "revisions" / "r0001" / "package_manifest.json").read_text(encoding="utf-8")
    )
    missing = manifest["evidence_completeness"]["missing_required"]
    assert any(
        row["source"] == "wiris.hr.hr-001"
        and row["role"] == "captures/hr/hr-001/right"
        and row["status"] == "missing"
        for row in missing
    )
    right = next(
        row
        for row in manifest["artifacts"]
        if row["role"] == "captures/hr/hr-001/right"
    )
    assert right["metadata"]["reason"] == "right FTP timed out"


def test_seal_failure_is_recoverable_and_keeps_spool(tmp_path: Path) -> None:
    panel = _panel(tmp_path)
    controller = MeasurementPackageController(panel)
    _begin_and_stop_detection(controller)
    root = _write_required_recording_outputs(panel)
    spool = root / ".spool" / "pair_rows.jsonl"
    spool.parent.mkdir(parents=True, exist_ok=True)
    spool.write_text('{"pair_id":"p1"}\n', encoding="utf-8")

    def fail_seal(*_args: object, **_kwargs: object) -> None:
        raise OSError("manifest write failed")

    panel._measurement_package.seal = fail_seal
    assert controller.finalize_measurement_package() is False
    snapshot = controller.measurement_package_snapshot()
    assert snapshot["state"] == "RECOVERABLE"
    assert "manifest write failed" in snapshot["error"]
    assert spool.read_text(encoding="utf-8") == '{"pair_id":"p1"}\n'


def test_session_save_failure_never_seals_measurement_revision(tmp_path: Path) -> None:
    panel = _panel(tmp_path)
    controller = MeasurementPackageController(panel)
    _begin_and_stop_detection(controller)
    root = panel._recording_session_dir(panel.recording_session_id)
    spool = root / ".spool" / "pair_rows.jsonl"
    spool.parent.mkdir(parents=True, exist_ok=True)
    spool.write_text('{"pair_id":"save-retry-sentinel"}\n', encoding="utf-8")
    panel._session_save_retry_required = True
    panel._session_save_finalized = False

    assert controller.finalize_measurement_package() is False

    snapshot = controller.measurement_package_snapshot()
    assert snapshot["state"] == "RECOVERABLE"
    assert "session persistence" in snapshot["detail"].lower()
    assert not (root / "revisions" / "r0001" / "package_manifest.json").exists()
    assert spool.read_text(encoding="utf-8") == '{"pair_id":"save-retry-sentinel"}\n'


def test_explicit_session_save_retry_resumes_pending_finalize_once(tmp_path: Path) -> None:
    panel = _panel(tmp_path)
    controller = MeasurementPackageController(panel)
    _begin_and_stop_detection(controller)
    root = panel._recording_session_dir(panel.recording_session_id)
    save_calls: list[str] = []

    def fail_first_save() -> None:
        save_calls.append("failed")
        panel._session_save_retry_required = True
        panel._session_save_finalized = False
        panel._session_save_last_error = "injected persistence failure"

    panel.save_distance_history_csv = fail_first_save
    assert controller.finalize_measurement_package() is False
    assert controller.measurement_package_snapshot()["state"] == "RECOVERABLE"
    assert save_calls == ["failed"]
    assert not (root / "revisions" / "r0001" / "package_manifest.json").exists()

    _write_required_recording_outputs(panel)
    panel._session_save_retry_required = False
    panel._session_save_last_error = ""
    assert controller._continue_measurement_package_finalization() is True
    assert controller.measurement_package_snapshot()["state"] == "SEALED_COMPLETE"
    assert controller.verify_measurement_package()["valid"] is True


def test_finalize_waits_for_valid_rtcm_evidence_barrier(tmp_path: Path) -> None:
    panel = _panel(tmp_path)
    controller = MeasurementPackageController(panel)
    _begin_and_stop_detection(controller)
    root = _write_required_recording_outputs(panel)
    requested: list[str] = []
    panel.rtcm_pipeline_worker = object()
    panel.rtcm_pipeline_thread = SimpleNamespace(isRunning=lambda: True)
    panel._close_rtcm_evidence_context_and_drain = (
        lambda token: requested.append(str(token)) or True
    )
    controller._schedule_finalize_retry = lambda: None  # type: ignore[method-assign]

    assert controller.finalize_measurement_package() is False
    assert requested == [f"{panel.recording_session_id}:session:r0001"]
    assert controller.measurement_package_snapshot()["state"] == "VERIFYING"
    assert not (root / "revisions" / "r0001" / "package_manifest.json").exists()

    assert controller._handle_measurement_rtcm_evidence_drained(
        {
            "schema": "workstation-rtcm-evidence-drain-v1",
            "context_token": requested[0],
            "session_id": panel.recording_session_id,
            "session_root": str(root),
            "valid": True,
            "error": "",
            "high_water": {"last_sequence": 7, "source_sequences": {"ntrip": 7}},
            "unconfirmed_injection_sequences": [],
            "integrity": {"evidence_failed": 0},
        }
    ) is True
    assert controller._continue_measurement_package_finalization() is True
    assert controller.verify_measurement_package()["valid"] is True


def test_invalid_rtcm_evidence_barrier_keeps_revision_recoverable(tmp_path: Path) -> None:
    panel = _panel(tmp_path)
    controller = MeasurementPackageController(panel)
    _begin_and_stop_detection(controller)
    root = _write_required_recording_outputs(panel)
    panel.rtcm_pipeline_worker = object()
    panel.rtcm_pipeline_thread = SimpleNamespace(isRunning=lambda: True)
    panel._close_rtcm_evidence_context_and_drain = lambda _token: True
    controller._schedule_finalize_retry = lambda: None  # type: ignore[method-assign]

    assert controller.finalize_measurement_package() is False
    token = panel._measurement_package_rtcm_drain_requested_token
    assert controller._handle_measurement_rtcm_evidence_drained(
        {
            "schema": "workstation-rtcm-evidence-drain-v1",
            "context_token": token,
            "session_id": panel.recording_session_id,
            "session_root": str(root),
            "valid": False,
            "error": "unindexed RTCM tail",
            "high_water": {"last_sequence": 0, "source_sequences": {}},
            "unconfirmed_injection_sequences": [],
            "integrity": {"evidence_failed": 1},
        }
    ) is False

    snapshot = controller.measurement_package_snapshot()
    assert snapshot["state"] == "RECOVERABLE"
    assert "unindexed RTCM tail" in snapshot["error"]
    assert not (root / "revisions" / "r0001" / "package_manifest.json").exists()


def test_revision_does_not_mutate_previous_manifest(tmp_path: Path) -> None:
    panel = _panel(tmp_path)
    controller = MeasurementPackageController(panel)
    _begin_and_stop_detection(controller)
    root = _write_required_recording_outputs(panel)
    assert controller.finalize_measurement_package() is True
    first_manifest = root / "revisions" / "r0001" / "package_manifest.json"
    before = first_manifest.read_bytes()

    assert controller.begin_measurement_revision() is True
    (root / "post_note.txt").write_text("post evidence", encoding="utf-8")
    assert controller.finalize_measurement_package() is True

    assert first_manifest.read_bytes() == before
    assert (root / "revisions" / "r0002" / "package_manifest.json").is_file()
    assert controller.verify_measurement_package()["valid"] is True


def test_camera_controller_blocks_explicit_detection_before_begin() -> None:
    calls: list[object] = []
    panel = SimpleNamespace(
        camera_connected=False,
        yolo_enabled=False,
        _pending_camera_start=None,
        _automatic_session_recording=False,
        _measurement_package_explicit_lifecycle=True,
        _session_recording_active=lambda: False,
        measurement_package_snapshot=lambda: {"state": "IDLE"},
        begin_measurement_detection=lambda: calls.append("detection.begin") or False,
        start_automatic_session_recording=lambda: calls.append("recording.begin") or True,
        _start_camera_streams=lambda **kwargs: calls.append(kwargs),
        _log=lambda message: calls.append(message),
    )

    assert CameraStreamLifecycleController(panel).start_yolo() is False
    assert "detection.begin" in calls
    assert "recording.begin" not in calls


def test_controller_reopens_a_recoverable_working_package_after_restart(tmp_path: Path) -> None:
    first_panel = _panel(tmp_path)
    first = MeasurementPackageController(first_panel)
    assert first.begin_measurement_session(session_id="restart-session") is True
    assert first.begin_measurement_detection() is True

    restarted_panel = _panel(tmp_path)
    restarted_panel.recording_session_id = "restart-session"
    restarted_panel._session_save_retry_required = True
    restarted = MeasurementPackageController(restarted_panel)
    snapshot = restarted.measurement_package_snapshot()

    assert snapshot["session_id"] == "restart-session"
    assert snapshot["state"] == "RECOVERABLE"
    assert snapshot["detection_started"] is True
    assert snapshot["detection_completed"] is False


def test_finalize_waits_for_stable_videos_then_registers_probe_evidence(tmp_path: Path) -> None:
    panel = _panel(tmp_path)
    panel._measurement_package_verify_media = True
    panel._measurement_package_video_probe_runner = lambda *_args, **_kwargs: SimpleNamespace(
        returncode=0,
        stderr="",
        stdout=json.dumps(
            {
                "streams": [
                    {
                        "index": 0,
                        "codec_type": "video",
                        "codec_name": "h264",
                        "width": 640,
                        "height": 480,
                        "avg_frame_rate": "30/1",
                        "r_frame_rate": "30/1",
                        "nb_frames": "3",
                        "duration": "0.1",
                    }
                ],
                "format": {"format_name": "mov,mp4", "duration": "0.1"},
            }
        ),
    )
    controller = MeasurementPackageController(panel)
    _begin_and_stop_detection(controller)
    _write_required_recording_outputs(panel)
    controller._schedule_finalize_retry = lambda: None  # type: ignore[method-assign]

    assert controller.finalize_measurement_package() is False
    assert "stable" in controller.measurement_package_snapshot()["detail"]
    assert controller._continue_measurement_package_finalization() is True

    manifest = controller.measurement_package_snapshot()
    videos = [
        row
        for row in manifest["artifacts"]
        if str(row.get("role", "")).startswith("video/")
    ]
    assert {row["source"] for row in videos} == {"camera.left", "camera.right"}
    assert all(row["metadata"]["codec"] == "h264" for row in videos)
