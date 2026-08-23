from __future__ import annotations

import hashlib
import json
import time
from types import SimpleNamespace

import pytest
from PyQt5 import QtGui

from Workstation.recording_controller import RecordingController
from Workstation.workstation_core.measurement_attempt import (
    BlockedMeasurementPair,
    DetectionSnapshot,
    GeometryConfigSnapshot,
    MeasurementAttempt,
)


def _detection(camera: str, timestamp_s: float) -> DetectionSnapshot:
    return DetectionSnapshot(
        camera=camera,
        app_receive_monotonic_s=timestamp_s,
        width_px=640,
        height_px=480,
        bbox_xyxy_px=(10.0, 20.0, 30.0, 40.0),
        center_px=(20.0, 30.0),
        confidence=0.9,
        class_id=0,
        class_name="target",
        pairing_timestamp_basis="app_receive_monotonic",
        pairing_timestamp_s=timestamp_s,
    )


def _readiness_payload() -> dict[str, object]:
    return {
        "schema": "measurement-readiness-block-v1",
        "methods": [{"method_id": "pixhawk_rtk_live", "ready": False}],
    }


def _readiness_hash(payload: dict[str, object]) -> str:
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(encoded).hexdigest()


def _attempt() -> MeasurementAttempt:
    return MeasurementAttempt(
        attempt_id=1,
        attempt_session_id="session-1",
        detection_pair_key=("pixhawk", 1.0, 1.01),
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
            max_vertical_delta_px=10.0,
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
        source_quality="NO DATA",
        warnings=("geometry missing",),
    )


def _blocked() -> BlockedMeasurementPair:
    payload = _readiness_payload()
    return BlockedMeasurementPair(
        detection_pair_key=("blocked", 1),
        created_monotonic_s=1.0,
        created_wall_s=2.0,
        timestamp_basis="app_receive_monotonic",
        left_detection=_detection("left", 1.0),
        right_detection=_detection("right", 1.01),
        status="matched_no_ready_measurement_method",
        reasons=("geometry missing",),
        readiness_sha256=_readiness_hash(payload),
        readiness_payload_json=json.dumps(payload),
    )


def test_attempt_canonical_payload_carries_field_policy_decision() -> None:
    attempt = _attempt()
    payload = attempt.canonical_payload()

    assert payload["measurement_policy_id"] == "field_measurement_v1"
    assert payload["measurement_validity"] == "INVALID"
    assert "intrinsics" in payload["hard_block_reasons"]
    assert payload["advisory_reasons"] == ("geometry missing",)
    assert MeasurementAttempt.from_canonical_payload(payload).canonical_payload() == payload


def test_blocked_pair_retains_the_exact_readiness_payload_for_hash_verification() -> None:
    payload = _readiness_payload()
    blocked = BlockedMeasurementPair(
        detection_pair_key=("blocked", 1),
        created_monotonic_s=1.0,
        created_wall_s=2.0,
        timestamp_basis="app_receive_monotonic",
        left_detection=_detection("left", 1.0),
        right_detection=_detection("right", 1.01),
        status="matched_no_ready_measurement_method",
        reasons=("geometry missing",),
        readiness_sha256=_readiness_hash(payload),
        readiness_payload_json=json.dumps(payload, indent=2),
    )

    assert json.loads(blocked.readiness_payload_json) == payload
    assert " " not in blocked.readiness_payload_json
    assert json.loads(blocked.canonical_json())["readiness_payload_json"] == (
        blocked.readiness_payload_json
    )


def test_blocked_pair_rejects_a_readiness_payload_that_does_not_match_its_hash() -> None:
    payload = _readiness_payload()
    with pytest.raises(ValueError, match="does not match"):
        BlockedMeasurementPair(
            detection_pair_key=("blocked", 1),
            created_monotonic_s=1.0,
            created_wall_s=2.0,
            timestamp_basis="app_receive_monotonic",
            left_detection=_detection("left", 1.0),
            right_detection=_detection("right", 1.01),
            status="matched_no_ready_measurement_method",
            reasons=("geometry missing",),
            readiness_sha256="0" * 64,
            readiness_payload_json=json.dumps(payload),
        )


def test_measurement_attempt_round_trips_through_its_canonical_recalculation_boundary() -> None:
    attempt = _attempt()

    persisted = json.loads(attempt.canonical_json())
    recovered = MeasurementAttempt.from_canonical_payload(persisted)

    assert recovered.canonical_payload() == attempt.canonical_payload()
    assert recovered.canonical_sha256() == attempt.canonical_sha256()


def test_terminal_record_ids_are_stable_and_namespaces_cannot_collide() -> None:
    attempt = _attempt()
    recovered = MeasurementAttempt.from_canonical_payload(
        json.loads(attempt.canonical_json())
    )
    blocked = _blocked()

    assert attempt.terminal_record_id == recovered.terminal_record_id
    assert attempt.terminal_record_id == f"attempt:{attempt.canonical_sha256()}"
    assert blocked.terminal_record_id == f"blocked:{blocked.canonical_sha256()}"
    assert attempt.terminal_record_id != blocked.terminal_record_id


def test_pair_recording_row_persists_reconstructable_attempt_bytes() -> None:
    attempt = _attempt()
    no_fields = lambda *_args, **_kwargs: {}
    panel = SimpleNamespace(
        recording_session_id="session-1",
        recording_session_started_monotonic_ns=1_000_000_000,
        recording_session_started_unix_ns=2_000_000_000,
        video_recording_paths={},
        _fmt_record_float=lambda value, precision=6: "" if value is None else f"{value:.{precision}f}",
        _record_bbox_parameters=no_fields,
        _record_measurement_sync_timestamp_parameters=no_fields,
        _record_intrinsic_parameters=no_fields,
        _record_frame_timestamp_parameters=no_fields,
        _record_live_telemetry_snapshot_parameters=no_fields,
        _measurement_gnss_record_metadata=no_fields,
        _record_external_parameters=no_fields,
        _record_reference_delta_parameters=no_fields,
    )
    controller = RecordingController(panel)
    controller._persist_native_measurement_pair = no_fields  # type: ignore[method-assign]
    controller._record_dual_geometry_parameters = no_fields  # type: ignore[method-assign]

    row = controller._pair_recording_base_row(attempt)
    persisted = json.loads(str(row["measurement_attempt_canonical_json"]))

    assert hashlib.sha256(
        json.dumps(persisted, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest() == row["measurement_attempt_sha256"]
    assert MeasurementAttempt.from_canonical_payload(persisted).canonical_sha256() == (
        row["measurement_attempt_sha256"]
    )
    assert row["terminal_record_id"] == f"attempt:{row['measurement_attempt_sha256']}"
    assert row["measurement_package_id"] == row["measurement_id"]


def test_measurement_result_evidence_has_an_independently_checkable_digest() -> None:
    evidence = RecordingController._measurement_result_evidence(
        attempt_sha256="a" * 64,
        status="measured",
        valid=True,
        method="checkerboard_stereo",
        quality="GOOD",
        warnings=("sentinel",),
        vector_frd_range_m=(1.0, 2.0, 3.0, 4.0),
    )
    payload_bytes = evidence["measurement_result_canonical_json"].encode("utf-8")
    assert hashlib.sha256(payload_bytes).hexdigest() == evidence["measurement_result_sha256"]
    assert json.loads(payload_bytes)["vector_frd_range_m"] == [1.0, 2.0, 3.0, 4.0]


def test_native_pair_lookup_uses_app_receive_clock_when_pairing_uses_rtcp(
    tmp_path,
) -> None:
    requested: dict[str, list[float]] = {"left": [], "right": []}

    def pane(side: str, timestamp_s: float, color: str) -> SimpleNamespace:
        image = QtGui.QImage(16, 12, QtGui.QImage.Format_RGB888)
        image.fill(QtGui.QColor(color))

        def native_measurement_frame(requested_s: float):
            requested[side].append(float(requested_s))
            if abs(float(requested_s) - timestamp_s) > 1.0e-9:
                return None
            return (
                timestamp_s,
                image,
                {
                    "timestamp_s": timestamp_s,
                    "timestamp_reference": "app-receive",
                    "native_width": image.width(),
                    "native_height": image.height(),
                    "native_lossless_source": True,
                    "overlay_applied": False,
                    "source_native_frame_id": f"native-{side}",
                },
            )

        return SimpleNamespace(native_measurement_frame=native_measurement_frame)

    panel = SimpleNamespace(
        recording_session_id="rtcp-native-session",
        _recording_session_dir=lambda _session_id: tmp_path / "rtcp-native-session",
        left_pane=pane("left", 10.0, "red"),
        right_pane=pane("right", 10.01, "blue"),
    )
    left = DetectionSnapshot(
        **{
            **_detection("left", 10.0).__dict__,
            "pairing_timestamp_basis": "camera_rtc_required",
            "pairing_timestamp_s": 1_787_000_000.0,
        }
    )
    right = DetectionSnapshot(
        **{
            **_detection("right", 10.01).__dict__,
            "pairing_timestamp_basis": "camera_rtc_required",
            "pairing_timestamp_s": 1_787_000_000.01,
        }
    )

    controller = RecordingController(panel)
    result = controller._persist_native_detection_pair(
        left_detection=left,
        right_detection=right,
        pair_id="pair-rtcp",
        purpose="accepted-measurement",
    )

    assert requested == {"left": [10.0], "right": [10.01]}
    assert result["native_pair_status"] == "pending"
    assert result["native_left_source_frame_id"] == "native-left"
    assert result["native_right_source_frame_id"] == "native-right"
    deadline_s = time.monotonic() + 2.0
    while controller.native_frame_persistence_pending_count() and time.monotonic() < deadline_s:
        time.sleep(0.01)
    assert controller.native_frame_persistence_pending_count() == 0
    assert (tmp_path / "rtcp-native-session" / result["native_left_png_path"]).is_file()
    assert (tmp_path / "rtcp-native-session" / result["native_right_png_path"]).is_file()
    assert controller._shutdown_native_frame_stores() is True
