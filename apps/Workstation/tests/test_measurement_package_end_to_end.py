from __future__ import annotations

import hashlib
import json
import shutil
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest
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
from Workstation.workstation_core.measurement_data_store import LiveDataRows
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
    assert accepted_row["terminal_record_id"] == attempt.terminal_record_id
    assert accepted_row["measurement_package_id"] == accepted_row["measurement_id"]

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
    assert blocked_row["terminal_record_id"] == blocked.terminal_record_id
    assert blocked_row["measurement_package_id"] == blocked_row["measurement_id"]
    assert accepted_row["terminal_record_id"] != blocked_row["terminal_record_id"]


def test_primary_distance_commit_persists_method_outcome_provenance(
    tmp_path: Path,
) -> None:
    spool = SpoolRows(supports_updates=True)
    spool_path = tmp_path / "primary-row-session" / ".spool" / "pair_rows.jsonl"
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
        disparity_px=40.0,
        time_delta_s=0.01,
        vertical_delta_px=0.0,
        warnings=(),
    )
    metadata = {
        "runtime_generation": "run-primary",
        "method_evaluation_pair_id": "run-primary:00000000000000000001",
        "method_evaluation_pair_sequence": 1,
        "method_evaluation_request_generation": 1,
        "method_evaluation_snapshot_sha256": "b" * 64,
        "primary_outcome_status": "VALID",
        "primary_outcome_sha256": "c" * 64,
        "primary_committed_before_bundle": True,
        "primary_commit_source": "early_outcome",
    }

    committed = controller._commit_primary_distance_outcome(
        estimate=estimate,
        distance_m=9.0,
        method="two_ray_closest_midpoint-pixhawk_rtk_live",
        baseline_source="ignored-by-controller",
        baseline_m=0.0,
        point_m=(9.0, 0.0, 0.0),
        raw_point_m=(9.0, 0.0, 0.0),
        raw_point_left_m=(0.0, 0.0, 9.0),
        measurement_attempt=attempt,
        outcome_metadata=metadata,
        publish_history=True,
    )

    assert committed.history_added is True
    assert committed.row_commit is not None
    assert committed.row_commit.enqueue_admitted is True
    assert len(spool) == 1
    row = spool[0]
    for key, value in metadata.items():
        assert row[key] == value
    assert row["measurement_valid"] == "true"
    assert row["measurement_validity"] == "VALID"
    assert row["distance_m"] == "9.000000"


def test_primary_bundle_retry_updates_the_physically_admitted_provenance(
    tmp_path: Path,
) -> None:
    calls: list[dict[str, object]] = []
    admissions = iter((False, True))

    class Store:
        def enqueue(self, _stream, payload, *, priority=None):
            calls.append({"payload": dict(payload), "priority": priority})
            return next(admissions)

    rows = LiveDataRows(Store(), "measurements", priority=True)
    panel = _recording_panel(tmp_path, rows)
    controller = RecordingController(panel)
    native_calls: list[int] = []
    persist_native = controller._persist_native_measurement_pair

    def counted_native(measurement_attempt):
        native_calls.append(int(measurement_attempt.attempt_id))
        return persist_native(measurement_attempt)

    controller._persist_native_measurement_pair = counted_native
    panel._pair_recording_base_row = controller._pair_recording_base_row
    panel._upsert_pair_recording_row = controller._upsert_pair_recording_row
    attempt = _measurement_attempt()
    estimate = SimpleNamespace(
        left=SimpleNamespace(timestamp_s=1.0),
        right=SimpleNamespace(timestamp_s=1.01),
        sync_quality="GOOD",
        disparity_px=40.0,
        time_delta_s=0.01,
        vertical_delta_px=0.0,
        warnings=(),
    )
    base_metadata = {
        "runtime_generation": "run-primary",
        "method_evaluation_pair_id": "run-primary:00000000000000000001",
        "method_evaluation_pair_sequence": 1,
        "method_evaluation_request_generation": 1,
        "method_evaluation_snapshot_sha256": "b" * 64,
        "primary_outcome_status": "VALID",
        "primary_outcome_sha256": "c" * 64,
    }

    first = controller._commit_primary_distance_outcome(
        estimate=estimate,
        distance_m=9.0,
        method="two_ray_closest_midpoint-pixhawk_rtk_live",
        baseline_source="ignored-by-controller",
        baseline_m=0.0,
        point_m=(9.0, 0.0, 0.0),
        raw_point_m=(9.0, 0.0, 0.0),
        raw_point_left_m=(0.0, 0.0, 9.0),
        measurement_attempt=attempt,
        outcome_metadata={
            **base_metadata,
            "primary_committed_before_bundle": True,
            "primary_commit_source": "early_outcome",
        },
        publish_history=True,
    )
    second = controller._commit_primary_distance_outcome(
        estimate=estimate,
        distance_m=9.0,
        method="two_ray_closest_midpoint-pixhawk_rtk_live",
        baseline_source="ignored-by-controller",
        baseline_m=0.0,
        point_m=(9.0, 0.0, 0.0),
        raw_point_m=(9.0, 0.0, 0.0),
        raw_point_left_m=(0.0, 0.0, 9.0),
        measurement_attempt=attempt,
        outcome_metadata={
            **base_metadata,
            "primary_committed_before_bundle": False,
            "primary_commit_source": "bundle_fallback",
        },
        publish_history=False,
    )

    assert first.row_commit is not None
    assert first.row_commit.enqueue_admitted is False
    assert second.row_commit is not None
    assert second.row_commit.enqueue_admitted is True
    assert [call["payload"]["_record_operation"] for call in calls] == [
        "append",
        "append",
    ]
    assert rows[0]["primary_commit_source"] == "bundle_fallback"
    assert rows[0]["primary_committed_before_bundle"] is False
    assert native_calls == [attempt.attempt_id]


def test_primary_unavailable_spool_uses_direct_measurement_persistence(
    tmp_path: Path,
) -> None:
    for mode in ("unbound", "closed"):
        rows = SpoolRows(supports_updates=True)
        if mode == "closed":
            rows.bind(tmp_path / mode / "measurements.jsonl")
            assert rows.close() is True
        panel = _recording_panel(tmp_path, rows)
        writes: list[dict[str, object]] = []

        def append_workstation_data(stream, payload, *, priority=None):
            writes.append(
                {
                    "stream": str(stream),
                    "payload": dict(payload),
                    "priority": priority,
                }
            )
            return True

        panel.append_workstation_data = append_workstation_data
        controller = RecordingController(panel)
        panel._pair_recording_base_row = controller._pair_recording_base_row
        panel._upsert_pair_recording_row = controller._upsert_pair_recording_row
        attempt = _measurement_attempt()
        estimate = SimpleNamespace(
            left=SimpleNamespace(timestamp_s=1.0),
            right=SimpleNamespace(timestamp_s=1.01),
            sync_quality="GOOD",
            disparity_px=40.0,
            time_delta_s=0.01,
            vertical_delta_px=0.0,
            warnings=(),
        )

        committed = controller._commit_primary_distance_outcome(
            estimate=estimate,
            distance_m=9.0,
            method="two_ray_closest_midpoint-pixhawk_rtk_live",
            baseline_source="ignored-by-controller",
            baseline_m=0.0,
            point_m=(9.0, 0.0, 0.0),
            raw_point_m=(9.0, 0.0, 0.0),
            raw_point_left_m=(0.0, 0.0, 9.0),
            measurement_attempt=attempt,
            outcome_metadata={
                "runtime_generation": f"run-{mode}",
                "primary_outcome_status": "VALID",
                "primary_outcome_sha256": "d" * 64,
                "primary_committed_before_bundle": True,
                "primary_commit_source": "early_outcome",
            },
            publish_history=True,
        )

        assert committed.history_added is True
        assert committed.row_commit is not None
        assert committed.row_commit.enqueue_admitted is True
        assert committed.row_commit.direct_persistence is True
        assert committed.row_commit.row_index == -1
        assert len(rows) == 0
        assert len(writes) == 1
        assert writes[0]["stream"] == "measurements"
        assert writes[0]["priority"] is True
        assert writes[0]["payload"]["row_persistence_mode"] == (
            "direct_owner_unavailable"
        )
        assert mode in str(
            writes[0]["payload"]["row_owner_unavailable_reason"]
        ) or "spool" in str(
            writes[0]["payload"]["row_owner_unavailable_reason"]
        )


def test_primary_direct_rejection_retries_cached_full_row_without_new_native_request(
    tmp_path: Path,
) -> None:
    rows = SpoolRows(supports_updates=True)
    panel = _recording_panel(tmp_path, rows)
    writes: list[dict[str, object]] = []
    direct_admissions = iter((False, True))

    def append_workstation_data(stream, payload, *, priority=None):
        writes.append(
            {
                "stream": str(stream),
                "payload": dict(payload),
                "priority": priority,
            }
        )
        return next(direct_admissions)

    panel.append_workstation_data = append_workstation_data
    controller = RecordingController(panel)
    native_calls: list[int] = []

    def counted_native(measurement_attempt):
        native_calls.append(int(measurement_attempt.attempt_id))
        return {
            "native_pair_request_id": "native-primary-1",
            "native_pair_status": "pending",
        }

    controller._persist_native_measurement_pair = counted_native
    panel._pair_recording_base_row = controller._pair_recording_base_row
    panel._upsert_pair_recording_row = controller._upsert_pair_recording_row
    attempt = _measurement_attempt()
    estimate = SimpleNamespace(
        left=SimpleNamespace(timestamp_s=1.0),
        right=SimpleNamespace(timestamp_s=1.01),
        sync_quality="GOOD",
        disparity_px=40.0,
        time_delta_s=0.01,
        vertical_delta_px=0.0,
        warnings=(),
    )
    base_metadata = {
        "runtime_generation": "run-primary",
        "method_evaluation_pair_id": "run-primary:00000000000000000001",
        "method_evaluation_pair_sequence": 1,
        "method_evaluation_request_generation": 1,
        "method_evaluation_snapshot_sha256": "b" * 64,
        "primary_outcome_status": "VALID",
        "primary_outcome_sha256": "c" * 64,
    }

    first = controller._commit_primary_distance_outcome(
        estimate=estimate,
        distance_m=9.0,
        method="two_ray_closest_midpoint-pixhawk_rtk_live",
        baseline_source="ignored-by-controller",
        baseline_m=0.0,
        point_m=(9.0, 0.0, 0.0),
        raw_point_m=(9.0, 0.0, 0.0),
        raw_point_left_m=(0.0, 0.0, 9.0),
        measurement_attempt=attempt,
        outcome_metadata={
            **base_metadata,
            "primary_committed_before_bundle": True,
            "primary_commit_source": "early_outcome",
        },
        publish_history=True,
    )
    controller._deferred_native_frame_results["native-primary-1"] = {
        "fields": {
            "native_pair_status": "saved",
            "native_pair_sha256": "f" * 64,
        }
    }
    second = controller._commit_primary_distance_outcome(
        estimate=estimate,
        distance_m=9.0,
        method="two_ray_closest_midpoint-pixhawk_rtk_live",
        baseline_source="ignored-by-controller",
        baseline_m=0.0,
        point_m=(9.0, 0.0, 0.0),
        raw_point_m=(9.0, 0.0, 0.0),
        raw_point_left_m=(0.0, 0.0, 9.0),
        measurement_attempt=attempt,
        outcome_metadata={
            **base_metadata,
            "primary_committed_before_bundle": False,
            "primary_commit_source": "background_retry",
        },
        publish_history=False,
    )

    assert first.row_commit is not None
    assert first.row_commit.enqueue_admitted is False
    assert second.row_commit is not None
    assert second.row_commit.enqueue_admitted is True
    assert second.row_commit.direct_persistence is True
    assert native_calls == [attempt.attempt_id]
    assert len(writes) == 2
    persisted = writes[-1]["payload"]
    assert persisted["measurement_attempt_sha256"] == attempt.canonical_sha256()
    assert persisted["distance_m"] == "9.000000"
    assert persisted["primary_commit_source"] == "background_retry"
    assert persisted["primary_committed_before_bundle"] is False
    assert persisted["row_persistence_mode"] == "direct_owner_unavailable"
    assert persisted["native_pair_request_id"] == "native-primary-1"
    assert persisted["native_pair_status"] == "saved"
    assert persisted["native_pair_sha256"] == "f" * 64
    assert controller._deferred_native_frame_results == {}
    assert controller._pending_primary_record_rows == {}
    assert len(panel.distance_history) == 1


def test_primary_cache_eviction_uses_compact_direct_without_native_reschedule(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from Workstation import recording_controller as recording_module

    monkeypatch.setattr(recording_module, "RECORDING_KEY_CACHE_LIMIT", 2)
    store_calls: list[dict[str, object]] = []
    store_admissions = iter((False, False, False, True, True))

    class Store:
        def enqueue(self, _stream, payload, *, priority=None):
            store_calls.append({"payload": dict(payload), "priority": priority})
            return next(store_admissions)

    rows = LiveDataRows(Store(), "measurements", priority=True)
    panel = _recording_panel(tmp_path, rows)
    panel._recording_pending_native_request_rows = {}
    direct_writes: list[dict[str, object]] = []
    suppression_admissions = iter((False, True))

    def append_workstation_data(stream, payload, **_kwargs):
        direct_writes.append({"stream": str(stream), "payload": dict(payload)})
        if (
            stream == "native.frames"
            and payload.get("event_type")
            == "native_frame_row_projection_suppressed"
        ):
            return next(suppression_admissions)
        return True

    panel.append_workstation_data = append_workstation_data
    controller = RecordingController(panel)
    native_calls: list[int] = []

    def counted_native(measurement_attempt):
        native_calls.append(int(measurement_attempt.attempt_id))
        return {
            "native_pair_request_id": f"native-{measurement_attempt.attempt_id}",
            "native_pair_status": "pending",
        }

    controller._persist_native_measurement_pair = counted_native
    panel._pair_recording_base_row = controller._pair_recording_base_row
    panel._upsert_pair_recording_row = controller._upsert_pair_recording_row
    base_attempt = _measurement_attempt()
    attempts = tuple(
        replace(
            base_attempt,
            attempt_id=index,
            attempt_session_id=f"attempt-session-{index:03d}",
            detection_pair_key=("accepted", index),
        )
        for index in range(1, 4)
    )
    estimate = SimpleNamespace(
        left=SimpleNamespace(timestamp_s=1.0),
        right=SimpleNamespace(timestamp_s=1.01),
        sync_quality="GOOD",
        disparity_px=40.0,
        time_delta_s=0.01,
        vertical_delta_px=0.0,
        warnings=(),
    )

    def metadata(attempt: MeasurementAttempt, source: str) -> dict[str, object]:
        return {
            "runtime_generation": "run-primary",
            "method_evaluation_pair_id": f"run-primary:{attempt.attempt_id}",
            "method_evaluation_pair_sequence": attempt.attempt_id,
            "method_evaluation_request_generation": attempt.attempt_id,
            "method_evaluation_snapshot_sha256": "b" * 64,
            "primary_outcome_status": "VALID",
            "primary_outcome_sha256": f"{attempt.attempt_id:x}" * 64,
            "primary_committed_before_bundle": source == "early_outcome",
            "primary_commit_source": source,
        }

    first_results = [
        controller._commit_primary_distance_outcome(
            estimate=estimate,
            distance_m=9.0,
            method="two_ray_closest_midpoint-pixhawk_rtk_live",
            baseline_source="ignored-by-controller",
            baseline_m=0.0,
            point_m=(9.0, 0.0, 0.0),
            raw_point_m=(9.0, 0.0, 0.0),
            raw_point_left_m=(0.0, 0.0, 9.0),
            measurement_attempt=attempt,
            outcome_metadata=metadata(attempt, "early_outcome"),
            publish_history=True,
        )
        for attempt in attempts
    ]

    assert all(
        result.row_commit is not None
        and result.row_commit.enqueue_admitted is False
        for result in first_results
    )
    assert len(controller._pending_primary_record_rows) == 2
    assert tuple(attempts[0].detection_pair_key) not in (
        controller._pending_primary_record_rows
    )

    retry_results = [
        controller._commit_primary_distance_outcome(
            estimate=estimate,
            distance_m=9.0,
            method="two_ray_closest_midpoint-pixhawk_rtk_live",
            baseline_source="ignored-by-controller",
            baseline_m=0.0,
            point_m=(9.0, 0.0, 0.0),
            raw_point_m=(9.0, 0.0, 0.0),
            raw_point_left_m=(0.0, 0.0, 9.0),
            measurement_attempt=attempt,
            outcome_metadata=metadata(attempt, "background_retry"),
            publish_history=False,
        )
        for attempt in attempts
    ]

    assert all(
        result.row_commit is not None
        and result.row_commit.enqueue_admitted is True
        for result in retry_results
    )
    controller._remember_compact_primary_persisted(("extra", 1))
    controller._remember_compact_primary_persisted(("extra", 2))
    assert tuple(attempts[0].detection_pair_key) in (
        controller._compact_primary_persisted_keys
    )
    store_call_count = len(store_calls)
    controller._handle_async_native_frame_result(
        {
            "request_id": "native-1",
            "context": {
                "session_id": panel.recording_session_id,
                "record_key": list(attempts[0].detection_pair_key),
                "writer_kind": "legacy",
            },
            "fields": {
                "native_pair_status": "saved",
                "native_pair_sha256": "a" * 64,
            },
        }
    )
    assert "native-1" in controller._pending_native_completions
    assert tuple(attempts[0].detection_pair_key) in (
        controller._compact_primary_persisted_keys
    )
    controller._pending_native_completions[
        "native-1"
    ].next_retry_monotonic_s = 0.0
    controller._retry_pending_native_completions()

    assert native_calls == [1, 2, 3]
    assert len(rows) == 3
    assert len(panel.distance_history) == 3
    assert len(store_calls) == store_call_count == 5
    assert rows[0]["native_pair_status"] == "pending"
    measurement_writes = [
        write for write in direct_writes if write["stream"] == "measurements"
    ]
    assert len(measurement_writes) == 1
    assert any(
        write["stream"] == "native.frames"
        and write["payload"]["event_type"]
        == "native_frame_row_projection_suppressed"
        for write in direct_writes
    )
    assert sum(
        write["stream"] == "native.frames"
        and write["payload"]["event_type"]
        == "native_frame_row_projection_suppressed"
        for write in direct_writes
    ) == 2
    compact = measurement_writes[0]["payload"]
    assert compact["event_type"] == "primary_measurement_retry_cache_miss"
    assert compact["row_persistence_mode"] == "direct_retry_cache_miss"
    assert compact["measurement_attempt_sha256"] == attempts[0].canonical_sha256()
    assert compact["primary_commit_source"] == "background_retry"
    assert tuple(attempts[0].detection_pair_key) in (
        controller._compact_primary_persisted_keys
    )
    assert controller._pending_native_completions == {}
    assert controller._pending_primary_record_rows == {}


def test_late_compact_skips_direct_when_native_completion_admitted_full_row(
    tmp_path: Path,
) -> None:
    store_calls: list[dict[str, object]] = []
    store_admissions = iter((False, True))

    class Store:
        def enqueue(self, _stream, payload, *, priority=None):
            store_calls.append({"payload": dict(payload), "priority": priority})
            return next(store_admissions)

    rows = LiveDataRows(Store(), "measurements", priority=True)
    panel = _recording_panel(tmp_path, rows)
    panel._recording_pending_native_request_rows = {}
    writes: list[dict[str, object]] = []
    panel.append_workstation_data = lambda stream, payload, **_kwargs: (
        writes.append({"stream": str(stream), "payload": dict(payload)}) or True
    )
    controller = RecordingController(panel)
    controller._persist_native_measurement_pair = lambda _attempt: {
        "native_pair_request_id": "native-full-first",
        "native_pair_status": "pending",
    }
    panel._pair_recording_base_row = controller._pair_recording_base_row
    panel._upsert_pair_recording_row = controller._upsert_pair_recording_row
    attempt = _measurement_attempt()
    estimate = SimpleNamespace(
        left=SimpleNamespace(timestamp_s=1.0),
        right=SimpleNamespace(timestamp_s=1.01),
        sync_quality="GOOD",
        disparity_px=40.0,
        time_delta_s=0.01,
        vertical_delta_px=0.0,
        warnings=(),
    )
    first = controller._commit_primary_distance_outcome(
        estimate=estimate,
        distance_m=9.0,
        method="two_ray_closest_midpoint-pixhawk_rtk_live",
        baseline_source="ignored-by-controller",
        baseline_m=0.0,
        point_m=(9.0, 0.0, 0.0),
        raw_point_m=(9.0, 0.0, 0.0),
        raw_point_left_m=(0.0, 0.0, 9.0),
        measurement_attempt=attempt,
        outcome_metadata={
            "runtime_generation": "run-old",
            "primary_outcome_status": "VALID",
            "primary_outcome_sha256": "e" * 64,
            "primary_committed_before_bundle": True,
            "primary_commit_source": "early_outcome",
        },
        publish_history=True,
    )
    assert first.row_commit is not None
    assert first.row_commit.enqueue_admitted is False

    controller._handle_async_native_frame_result(
        {
            "request_id": "native-full-first",
            "context": {
                "session_id": panel.recording_session_id,
                "record_key": list(attempt.detection_pair_key),
                "writer_kind": "legacy",
            },
            "fields": {
                "native_pair_status": "saved",
                "native_pair_sha256": "f" * 64,
            },
        }
    )
    assert tuple(attempt.detection_pair_key) in (
        controller._native_primary_persisted_keys
    )

    admitted = controller._persist_compact_primary_measurement(
        tuple(attempt.detection_pair_key),
        {
            "event_type": "late_primary_measurement_outcome",
            "primary_persistence_mode": "late_direct_retry",
            "primary_outcome_sha256": "e" * 64,
        },
    )

    assert admitted is True
    assert len(store_calls) == 2
    assert rows[0]["native_pair_status"] == "saved"
    assert not any(write["stream"] == "measurements" for write in writes)
    assert tuple(attempt.detection_pair_key) not in (
        controller._native_primary_persisted_keys
    )


def test_native_full_tokens_protect_all_active_primary_retries_over_cache_bound(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from Workstation import recording_controller as recording_module

    monkeypatch.setattr(recording_module, "RECORDING_KEY_CACHE_LIMIT", 1)
    panel = _recording_panel(tmp_path, SpoolRows(supports_updates=True))
    writes: list[dict[str, object]] = []
    panel.append_workstation_data = lambda stream, payload, **_kwargs: (
        writes.append({"stream": str(stream), "payload": dict(payload)}) or True
    )
    controller = RecordingController(panel)
    key1 = ("active-primary", 1)
    key2 = ("active-primary", 2)
    controller._primary_retry_required_keys.update((key1, key2))

    controller._remember_native_primary_persisted(key1)
    controller._remember_native_primary_persisted(key2)

    assert controller._native_primary_persisted_keys == {key1, key2}
    assert len(controller._native_primary_persisted_order) == 2
    assert controller._persist_compact_primary_measurement(
        key1,
        {
            "event_type": "late_primary_measurement_outcome",
            "primary_persistence_mode": "late_direct_retry",
        },
    ) is True
    assert not any(write["stream"] == "measurements" for write in writes)
    assert controller._native_primary_persisted_keys == {key2}
    assert controller._primary_retry_required_keys == {key2}


@pytest.mark.parametrize("reuse_index", [False, True], ids=["cleared", "reused"])
def test_primary_retry_lease_rejects_cleared_or_reused_index_and_persists_clean_direct_row(
    tmp_path: Path,
    reuse_index: bool,
) -> None:
    admissions = iter((False, True))

    class Store:
        def enqueue(self, _stream, _payload, *, priority=None):
            return next(admissions)

    rows = LiveDataRows(Store(), "measurements", priority=True)
    panel = _recording_panel(tmp_path, rows)
    writes: list[dict[str, object]] = []
    panel.append_workstation_data = lambda stream, payload, **_kwargs: writes.append(
        {"stream": str(stream), "payload": dict(payload)}
    ) or True
    controller = RecordingController(panel)
    native_calls: list[int] = []
    persist_native = controller._persist_native_measurement_pair

    def counted_native(measurement_attempt):
        native_calls.append(int(measurement_attempt.attempt_id))
        return persist_native(measurement_attempt)

    controller._persist_native_measurement_pair = counted_native
    panel._pair_recording_base_row = controller._pair_recording_base_row
    panel._upsert_pair_recording_row = controller._upsert_pair_recording_row
    attempt = _measurement_attempt()
    estimate = SimpleNamespace(
        left=SimpleNamespace(timestamp_s=1.0),
        right=SimpleNamespace(timestamp_s=1.01),
        sync_quality="GOOD",
        disparity_px=40.0,
        time_delta_s=0.01,
        vertical_delta_px=0.0,
        warnings=(),
    )

    first = controller._commit_primary_distance_outcome(
        estimate=estimate,
        distance_m=9.0,
        method="two_ray_closest_midpoint-pixhawk_rtk_live",
        baseline_source="ignored-by-controller",
        baseline_m=0.0,
        point_m=(9.0, 0.0, 0.0),
        raw_point_m=(9.0, 0.0, 0.0),
        raw_point_left_m=(0.0, 0.0, 9.0),
        measurement_attempt=attempt,
        outcome_metadata={
            "runtime_generation": "run-old",
            "primary_outcome_status": "VALID",
            "primary_outcome_sha256": "e" * 64,
            "primary_committed_before_bundle": True,
            "primary_commit_source": "early_outcome",
        },
        publish_history=True,
    )
    assert first.row_commit is not None
    assert first.row_commit.enqueue_admitted is False
    old_lease = first.row_commit.row_lease
    assert old_lease is not None

    rows.clear()
    if reuse_index:
        replacement = rows.append_with_admission(
            {
                "sentinel": "new-run-row",
                "native_pair_request_id": "new-request",
            }
        )
        assert replacement.admitted is True
        assert replacement.lease != old_lease

    second = controller._commit_primary_distance_outcome(
        estimate=estimate,
        distance_m=9.0,
        method="two_ray_closest_midpoint-pixhawk_rtk_live",
        baseline_source="ignored-by-controller",
        baseline_m=0.0,
        point_m=(9.0, 0.0, 0.0),
        raw_point_m=(9.0, 0.0, 0.0),
        raw_point_left_m=(0.0, 0.0, 9.0),
        measurement_attempt=attempt,
        outcome_metadata={
            "runtime_generation": "run-old",
            "primary_outcome_status": "VALID",
            "primary_outcome_sha256": "e" * 64,
            "primary_committed_before_bundle": False,
            "primary_commit_source": "bundle_fallback",
        },
        publish_history=False,
    )

    assert second.row_commit is not None
    assert second.row_commit.enqueue_admitted is True
    assert second.row_commit.direct_persistence is True
    if reuse_index:
        assert rows[0] == {
            "sentinel": "new-run-row",
            "native_pair_request_id": "new-request",
        }
    else:
        assert len(rows) == 0
    assert len(writes) == 1
    persisted = writes[0]["payload"]
    assert writes[0]["stream"] == "measurements"
    assert "sentinel" not in persisted
    assert persisted["measurement_attempt_sha256"] == attempt.canonical_sha256()
    assert persisted["primary_commit_source"] == "bundle_fallback"
    assert persisted["primary_committed_before_bundle"] is False
    assert persisted["row_persistence_mode"] == "direct_owner_unavailable"
    assert native_calls == [attempt.attempt_id]


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
