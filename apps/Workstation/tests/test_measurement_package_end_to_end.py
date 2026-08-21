from __future__ import annotations

import hashlib
import json
import shutil
from pathlib import Path
from types import SimpleNamespace

from PyQt5 import QtGui

from Workstation.measurement_package_controller import MeasurementPackageController
from Workstation.recording_controller import RecordingController
from Workstation.workstation_core.measurement_attempt import (
    BlockedMeasurementPair,
    DetectionSnapshot,
    GeometryConfigSnapshot,
    MeasurementAttempt,
)
from Workstation.workstation_core.measurement_package import (
    MeasurementPackage,
    verify_package,
)
from Workstation.workstation_core.package_provenance import (
    REDACTED,
    write_atomic_json_snapshot,
)
from Workstation.workstation_core.recording_session import SpoolRows, save_session_files


def _package_panel(tmp_path: Path) -> SimpleNamespace:
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
        camera_connected=False,
        _pending_camera_start=None,
        pnp_photo_capture_thread=None,
        oz_photo_capture_thread=None,
        left_pane=None,
        right_pane=None,
        _session_recording_active=lambda: False,
        _recording_session_dir=lambda session_id: tmp_path / "recordings" / session_id,
        _log=logs.append,
    )
    panel.logs = logs
    return panel


def _write_finalized_session_outputs(panel: SimpleNamespace) -> Path:
    root = panel._recording_session_dir(panel.recording_session_id)
    video = root / "video"
    video.mkdir(parents=True, exist_ok=True)
    # This test exercises the package inventory boundary, not codec probing;
    # media_evidence has independent ffprobe/decoder tests with injected runners.
    (video / "raw_left.mp4").write_bytes(b"left-video-after-writer-drain")
    (video / "raw_right.mp4").write_bytes(b"right-video-after-writer-drain")

    save_session_files(
        root,
        metadata={"session_id": panel.recording_session_id},
        pair_rows=[],
        camera_rows={"left": [], "right": []},
        px4_rows={"left": [], "right": []},
    )
    panel._session_save_finalized = True
    return root


def _add_post_capture_pair(panel: SimpleNamespace, root: Path) -> None:
    stored_files: dict[str, dict[str, object]] = {}
    for side in ("left", "right"):
        path = root / "frames" / "hr_capture" / side / f"post-001-{side}.png"
        path.parent.mkdir(parents=True, exist_ok=True)
        image = QtGui.QImage(640, 480, QtGui.QImage.Format_RGB888)
        image.fill(QtGui.QColor("red" if side == "left" else "blue"))
        assert image.save(str(path), "PNG")
        payload = path.read_bytes()
        stored_files[side] = {
            "session_relative_path": path.relative_to(root).as_posix(),
            "sha256": hashlib.sha256(payload).hexdigest(),
            "byte_size": len(payload),
            "camera_serial": f"WIRIS-{side.upper()}-001",
            "width_px": 640,
            "height_px": 480,
        }
    panel.pnp_photo_capture_records = [
        {
            "capture_id": "post-001",
            "purpose": "post",
            "state": "imported",
            "storage": "session_owned_exact_file",
            "stored_files": stored_files,
        }
    ]


def test_recovered_session_relative_capture_path_cannot_escape_package_root(
    tmp_path: Path,
) -> None:
    panel = _package_panel(tmp_path)
    controller = MeasurementPackageController(panel)
    assert controller.begin_measurement_session(session_id="capture-path-escape") is True
    root = panel._recording_session_dir(panel.recording_session_id)
    outside = tmp_path / "secret.png"
    image = QtGui.QImage(8, 6, QtGui.QImage.Format_RGB888)
    image.fill(QtGui.QColor("magenta"))
    assert image.save(str(outside), "PNG")
    secret_bytes = outside.read_bytes()
    panel.pnp_photo_capture_records = [
        {
            "capture_id": "hr-escape",
            "state": "recovered",
            "stored_files": {
                "left": {"session_relative_path": "../../secret.png"},
            },
        }
    ]

    controller._register_capture_records("hr", panel.pnp_photo_capture_records)

    left = next(
        row
        for row in controller.measurement_package_snapshot()["artifacts"]
        if row.get("role") == "captures/hr/hr-escape/left"
    )
    assert left["status"] == "failed"
    assert "escapes the session root" in left["metadata"]["reason"]
    assert not any(
        path.read_bytes() == secret_bytes
        for path in root.rglob("*")
        if path.is_file()
    )


def test_explicit_lifecycle_is_portable_and_revisions_leave_r1_valid(
    tmp_path: Path,
) -> None:
    panel = _package_panel(tmp_path)
    controller = MeasurementPackageController(panel)

    assert controller.begin_measurement_session(session_id="field-measurement-001") is True
    assert controller.begin_measurement_detection() is True
    assert controller.begin_measurement_detection() is False
    assert controller.complete_measurement_detection() is True
    assert controller.measurement_package_snapshot()["state"] == "POST_CAPTURE"

    original_root = _write_finalized_session_outputs(panel)
    _add_post_capture_pair(panel, original_root)
    (original_root / "post" / "operator-note.json").parent.mkdir(parents=True)
    (original_root / "post" / "operator-note.json").write_text(
        '{"note":"post evidence retained"}\n', encoding="utf-8"
    )

    assert controller.finalize_measurement_package() is True
    first_manifest = original_root / "revisions" / "r0001" / "package_manifest.json"
    first_bytes = first_manifest.read_bytes()
    assert controller.measurement_package_snapshot()["state"] == "SEALED_COMPLETE"
    assert verify_package(original_root, revision=1).valid is True

    assert controller.begin_measurement_revision() is True
    late = original_root / "post" / "late-review.json"
    late.write_text('{"review":"accepted"}\n', encoding="utf-8")
    assert controller.finalize_measurement_package() is True
    assert first_manifest.read_bytes() == first_bytes
    assert verify_package(original_root, revision=1).valid is True
    latest_verification = verify_package(original_root)
    assert latest_verification.valid is True
    assert latest_verification.revision == "r0002"
    assert latest_verification.verified_manifests == 2

    latest_manifest = json.loads(
        (original_root / "revisions" / "r0002" / "package_manifest.json").read_text(
            encoding="utf-8"
        )
    )
    for artifact in latest_manifest["artifacts"]:
        relative = artifact.get("relative_path")
        if relative:
            assert not Path(relative).is_absolute()
            assert ".." not in Path(relative).parts

    relocated = tmp_path / "portable-copy"
    shutil.copytree(original_root, relocated, symlinks=True)
    shutil.rmtree(original_root)
    assert not original_root.exists()
    assert verify_package(relocated).valid is True
    assert verify_package(relocated, revision="r0001").valid is True
    assert (relocated / "post" / "late-review.json").read_text(encoding="utf-8")


def test_active_source_with_zero_output_seals_incomplete(tmp_path: Path) -> None:
    root = tmp_path / "zero-output"
    package = MeasurementPackage.create(root, session_id="zero-output-session")
    package.set_source(
        "gnss.late",
        active=True,
        required_roles=("gnss/raw_rtcm",),
        metadata={"activated_monotonic_ns": 55, "deactivated_monotonic_ns": 89},
    )

    manifest = package.seal({"result": "NO_VALID_RESULT"})

    assert manifest["state"] == "SEALED_INCOMPLETE"
    assert manifest["evidence_completeness"]["missing_required"] == [
        {"source": "gnss.late", "role": "gnss/raw_rtcm", "status": "missing"}
    ]
    assert manifest["measurement_outcome"]["result"] == "NO_VALID_RESULT"
    assert verify_package(root).valid is True


def _detection(side: str, timestamp_s: float) -> DetectionSnapshot:
    return DetectionSnapshot(
        camera=side,
        app_receive_monotonic_s=timestamp_s,
        width_px=640,
        height_px=480,
        bbox_xyxy_px=(10.0, 20.0, 50.0, 80.0),
        center_px=(30.0, 50.0),
        confidence=0.95,
        class_id=1,
        class_name="target",
        pairing_timestamp_basis="app_receive_monotonic",
        pairing_timestamp_s=timestamp_s,
    )


def _measurement_attempt() -> MeasurementAttempt:
    return MeasurementAttempt(
        attempt_id=1,
        attempt_session_id="attempt-session-001",
        detection_pair_key=("accepted", 1.0, 1.01),
        created_monotonic_s=1.02,
        created_wall_s=2.0,
        detection_midpoint_monotonic_s=1.005,
        timestamp_basis="app_receive_monotonic",
        triangulation_source="pixhawk",
        left_detection=_detection("left", 1.0),
        right_detection=_detection("right", 1.01),
        left_attitude=None,
        right_attitude=None,
        left_gps=None,
        right_gps=None,
        intrinsics=None,
        baseline=None,
        geometry_config=GeometryConfigSnapshot(
            focal_length_px=900.0,
            max_time_delta_s=0.1,
            max_vertical_delta_px=20.0,
            require_same_class=True,
            timestamp_basis="app_receive_monotonic",
            manual_focal_override=False,
        ),
        left_attitude_age_s=None,
        right_attitude_age_s=None,
        attitude_pair_delta_s=None,
        relative_roll_deg=None,
        relative_pitch_deg=None,
        relative_yaw_deg=None,
        source_quality="GOOD",
        warnings=(),
    )


def _recording_panel(tmp_path: Path, rows: SpoolRows) -> SimpleNamespace:
    def format_float(value: object, digits: int) -> str:
        return "" if value is None else f"{float(value):.{digits}f}"

    no_fields = lambda *_args, **_kwargs: {}
    panel = SimpleNamespace(
        recording_session_id="canonical-row-session",
        recording_session_started_monotonic_ns=0,
        recording_session_started_unix_ns=2_000_000_000,
        recording_session_started_wall_s=2.0,
        video_recording_paths={"left": "video/raw_left.mp4", "right": "video/raw_right.mp4"},
        left_pane=SimpleNamespace(native_measurement_frame=lambda _timestamp: None),
        right_pane=SimpleNamespace(native_measurement_frame=lambda _timestamp: None),
        _recording_session_dir=lambda _session_id: tmp_path / "canonical-row-session",
        _fmt_record_float=format_float,
        _record_bbox_parameters=no_fields,
        _record_measurement_sync_timestamp_parameters=no_fields,
        _record_intrinsic_parameters=no_fields,
        _record_frame_timestamp_parameters=no_fields,
        _record_live_telemetry_snapshot_parameters=no_fields,
        _measurement_gnss_record_metadata=no_fields,
        _record_external_parameters=no_fields,
        _record_reference_delta_parameters=no_fields,
        recording_nominal_intrinsics_snapshot=None,
        distance_recording_enabled=True,
        distance_recording_started_monotonic_s=0.0,
        distance_recording_rows=rows,
        distance_recording_keys=[],
        _recording_pair_row_index={},
        _last_distance_history_key=None,
        distance_history=[],
        distance_plot=SimpleNamespace(add_point=lambda _point: None),
        _distance_history_class_name=lambda _estimate: "target",
        _append_recording_table_row=lambda _row: None,
        _refresh_recording_table=lambda: None,
        _refresh_recording_status_label=lambda: None,
        _refresh_operation_header=lambda: None,
        _append_extrinsics_comparison_snapshot=lambda *_args: None,
        geometry_comparison_rows=[],
        extrinsics_comparison_recording_rows=[],
    )
    return panel


def _assert_canonical_hash(row: dict[str, object], json_field: str, hash_field: str) -> None:
    encoded = str(row[json_field]).encode("utf-8")
    assert hashlib.sha256(encoded).hexdigest() == row[hash_field]


def test_accepted_and_blocked_rows_persist_exact_canonical_payloads_and_hashes(
    tmp_path: Path,
) -> None:
    spool = SpoolRows(supports_updates=True)
    spool_path = tmp_path / "canonical-row-session" / ".spool" / "pair_rows.jsonl"
    spool.bind(spool_path)
    panel = _recording_panel(tmp_path, spool)
    controller = RecordingController(panel)
    panel._pair_recording_base_row = controller._pair_recording_base_row
    panel._upsert_pair_recording_row = controller._upsert_pair_recording_row

    attempt = _measurement_attempt()
    estimate = SimpleNamespace(
        left=SimpleNamespace(timestamp_s=1.0),
        right=SimpleNamespace(timestamp_s=1.01),
        sync_quality="GOOD",
        disparity_px=12.5,
        time_delta_s=0.01,
        vertical_delta_px=1.5,
        warnings=(),
    )
    assert controller._append_distance_history(
        estimate=estimate,
        distance_m=3.75,
        method="two_ray_closest_midpoint",
        baseline_source="ignored-by-controller",
        baseline_m=0.0,
        point_m=(1.0, 2.0, 3.0),
        raw_point_m=(1.0, 2.0, 3.0),
        raw_point_left_m=(2.0, 3.0, 1.0),
        measurement_attempt=attempt,
    ) is True

    readiness = {
        "schema": "measurement-readiness-block-v1",
        "methods": [{"method_id": "pixhawk_rtk_live", "ready": False}],
    }
    readiness_json = json.dumps(
        readiness, ensure_ascii=False, allow_nan=False, sort_keys=True, separators=(",", ":")
    )
    blocked = BlockedMeasurementPair(
        detection_pair_key=("blocked", 2.0, 2.01),
        created_monotonic_s=2.02,
        created_wall_s=3.0,
        timestamp_basis="app_receive_monotonic",
        left_detection=_detection("left", 2.0),
        right_detection=_detection("right", 2.01),
        status="matched_no_ready_measurement_method",
        reasons=("geometry missing",),
        readiness_sha256=hashlib.sha256(readiness_json.encode("utf-8")).hexdigest(),
        readiness_payload_json=readiness_json,
    )
    controller._record_blocked_measurement_pair(blocked)
    spool.close()

    recovered = SpoolRows(supports_updates=True)
    recovered.open_existing(spool_path)
    accepted_row, blocked_row = list(recovered)
    recovered.close()

    _assert_canonical_hash(
        accepted_row, "measurement_attempt_canonical_json", "measurement_attempt_sha256"
    )
    recovered_attempt = MeasurementAttempt.from_canonical_payload(
        json.loads(str(accepted_row["measurement_attempt_canonical_json"]))
    )
    assert recovered_attempt.canonical_sha256() == attempt.canonical_sha256()
    _assert_canonical_hash(
        accepted_row, "measurement_result_canonical_json", "measurement_result_sha256"
    )
    accepted_result = json.loads(str(accepted_row["measurement_result_canonical_json"]))
    assert accepted_result["valid"] is True
    assert accepted_result["measurement_attempt_sha256"] == attempt.canonical_sha256()
    assert accepted_result["vector_frd_range_m"] == [1.0, 2.0, 3.0, 3.75]

    _assert_canonical_hash(blocked_row, "blocked_pair_canonical_json", "blocked_pair_sha256")
    _assert_canonical_hash(
        blocked_row,
        "measurement_readiness_canonical_json",
        "measurement_readiness_sha256",
    )
    _assert_canonical_hash(
        blocked_row, "measurement_result_canonical_json", "measurement_result_sha256"
    )
    blocked_result = json.loads(str(blocked_row["measurement_result_canonical_json"]))
    assert blocked_result["valid"] is False
    assert blocked_result["measurement_attempt_sha256"] == blocked.canonical_sha256()


def test_provenance_redaction_survives_registration_and_sealing(tmp_path: Path) -> None:
    root = tmp_path / "redacted-package"
    package = MeasurementPackage.create(root, session_id="redaction-session")
    secrets = (
        "ftp-secret-value",
        "ntrip-secret-value",
        "url-secret-value",
        "token-secret-value",
    )
    descriptor = write_atomic_json_snapshot(
        root,
        "inputs/config/effective_settings.json",
        {
            "ftp": {"username": "operator", "password": secrets[0]},
            "environment": {"NTRIP_PASSWORD": secrets[1], "DISPLAY_MODE": "survey"},
            "endpoint": f"rtsp://camera:{secrets[2]}@192.0.2.10/live",
            "access_token": secrets[3],
            "calibration_path": tmp_path / "private-machine" / "intrinsics.json",
        },
        role="effective_settings",
        source="runtime_settings",
        data_schema="workstation-effective-settings-v1",
    )
    package.register_descriptor(descriptor)
    manifest = package.seal({"result": "NO_VALID_RESULT"})

    saved = json.loads(
        (root / "inputs" / "config" / "effective_settings.json").read_text(
            encoding="utf-8"
        )
    )
    assert saved["ftp"]["password"] == REDACTED
    assert saved["environment"]["NTRIP_PASSWORD"] == REDACTED
    assert saved["access_token"] == REDACTED
    assert manifest["state"] == "SEALED_COMPLETE"
    assert verify_package(root).valid is True

    text = "\n".join(
        path.read_text(encoding="utf-8", errors="replace")
        for path in root.rglob("*")
        if path.is_file()
        and path.suffix.lower() in {".json", ".md"}
    )
    for secret in secrets:
        assert secret not in text
    assert str(tmp_path) not in text
