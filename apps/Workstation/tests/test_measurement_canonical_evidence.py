from __future__ import annotations

import hashlib
import json
import time
from types import SimpleNamespace

import pytest
from PyQt5 import QtGui

from Workstation.recording_controller import RecordingController
from Workstation.workstation_core.native_frame_store import AsyncNativeFrameStore
from Workstation.workstation_core.measurement_attempt import (
    BlockedMeasurementPair,
    DetectionSnapshot,
    GeometryConfigSnapshot,
    MeasurementAttempt,
    app_receive_pair_timing_evidence,
    live_pair_timing_evidence,
)
from Workstation.workstation_core.measurement_data_store import LiveDataRows
from Workstation.workstation_core.stereo import detection_timestamp_for_basis


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


def test_app_receive_pair_timing_is_authoritative_and_ntp_is_record_only() -> None:
    left = DetectionSnapshot(
        **{
            **_detection("left", 10.0).__dict__,
            "timestamp_metadata_items": (
                ("app_receive_monotonic_ns", 10_000_000_000),
                ("app_receive_timestamp_role", "authoritative_distance"),
                ("rtcp_frame_ntp_unix_s", 100.0),
                ("clock_epoch_id", "left-epoch"),
                ("clock_epoch_state", "transition"),
            ),
        }
    )
    right = DetectionSnapshot(
        **{
            **_detection("right", 10.02).__dict__,
            "timestamp_metadata_items": (
                ("app_receive_monotonic_ns", 10_020_000_000),
                ("app_receive_timestamp_role", "authoritative_distance"),
                ("rtcp_frame_ntp_unix_s", 32_500.0),
                ("clock_epoch_id", "right-epoch"),
                ("clock_epoch_state", "current"),
            ),
        }
    )

    timing = app_receive_pair_timing_evidence(left, right)

    assert timing["time_basis"] == "app_receive_monotonic"
    assert timing["time_basis_role"] == "authoritative_distance"
    assert timing["selected_pair_delta_ns"] == 20_000_000
    assert timing["selected_pair_abs_delta_ns"] == 20_000_000
    assert timing["rtcp_pair_delta_s"] == pytest.approx(32_400.0)
    assert timing["ntp_evidence_role"] == "record_only"
    assert timing["left_clock_epoch_state"] == "transition"


def test_app_receive_pair_timing_records_exact_integer_delta_over_100ms() -> None:
    left = _detection("left", 10.0)
    right = _detection("right", 10.100_000_001)

    timing = app_receive_pair_timing_evidence(left, right)

    assert timing["selected_pair_abs_delta_ns"] == 100_000_001
    assert timing["pair_timing_limit_ns"] == 100_000_000
    assert timing["pair_timing_limit_exceeded"] is True
    assert timing["pair_timing_limit_exceeded_code"] == "APP_RECEIVE_DELTA_EXCEEDED"


def test_live_pair_timing_records_selected_rtcp_and_distinct_app_delta() -> None:
    left = DetectionSnapshot(
        **{
            **_detection("left", 10.0).__dict__,
            "pairing_timestamp_basis": "camera_rtcp",
            "pairing_timestamp_s": 1_000.0,
            "timestamp_metadata_items": (
                ("app_receive_monotonic_ns", 10_000_000_000),
                ("rtcp_frame_ntp_unix_s", 1_000.0),
                ("clock_epoch_id", "epoch-1"),
                ("clock_epoch_state", "current"),
            ),
        }
    )
    right = DetectionSnapshot(
        **{
            **_detection("right", 10.09).__dict__,
            "pairing_timestamp_basis": "camera_rtcp",
            "pairing_timestamp_s": 1_000.01,
            "timestamp_metadata_items": (
                ("app_receive_monotonic_ns", 10_090_000_000),
                ("rtcp_frame_ntp_unix_s", 1_000.01),
                ("clock_epoch_id", "epoch-1"),
                ("clock_epoch_state", "current"),
            ),
        }
    )

    timing = live_pair_timing_evidence(
        left,
        right,
        requested_basis="camera_rtcp",
        effective_basis="camera_rtcp",
    )

    assert timing["requested_time_basis"] == "camera_rtcp"
    assert timing["time_basis"] == "camera_rtcp"
    assert timing["selected_pair_delta_ns"] == 10_000_000
    assert timing["app_receive_pair_delta_ns"] == 90_000_000
    assert timing["ntp_evidence_role"] == "selected_distance_pairing"
    assert timing["app_receive_evidence_role"] == "attitude_association_and_fallback"
    assert detection_timestamp_for_basis(left, "camera_rtcp") == pytest.approx(1_000.0)


def test_live_pair_timing_fallback_is_provenance_not_measurement_warning() -> None:
    left = _detection("left", 10.0)
    right = _detection("right", 10.02)

    timing = live_pair_timing_evidence(
        left,
        right,
        requested_basis="camera_rtcp",
        effective_basis="app_receive_monotonic",
        fallback_reason="CLOCK_NOT_VERIFIED",
    )

    assert timing["fallback_active"] is True
    assert timing["fallback_reason"] == "CLOCK_NOT_VERIFIED"
    assert timing["selected_pair_delta_ns"] == 20_000_000
    assert timing["ntp_evidence_role"] == "record_only"
    assert "warning" not in timing


def test_invalid_record_only_ntp_does_not_block_valid_app_pair() -> None:
    left = DetectionSnapshot(
        **{
            **_detection("left", 10.0).__dict__,
            "timestamp_metadata_items": (
                ("app_receive_monotonic_ns", 10_000_000_000),
                ("rtcp_frame_ntp_unix_s", float("nan")),
                ("rtcp_sr_ntp_raw", {"malformed": True}),
                ("clock_epoch_state", "transition"),
            ),
        }
    )
    right = DetectionSnapshot(
        **{
            **_detection("right", 10.02).__dict__,
            "timestamp_metadata_items": (
                ("app_receive_monotonic_ns", 10_020_000_000),
                ("rtcp_frame_ntp_unix_s", "not-a-time"),
                ("clock_epoch_state", "failed"),
            ),
        }
    )

    timing = app_receive_pair_timing_evidence(left, right)

    assert timing["selected_pair_delta_ns"] == 20_000_000
    assert timing["left_rtcp_frame_ntp_unix_s"] is None
    assert timing["right_rtcp_frame_ntp_unix_s"] is None
    assert timing["ntp_evidence_role"] == "record_only"
    assert left.timestamp_metadata["rtcp_sr_ntp_raw"] is None


def test_recorded_selected_sync_delta_is_app_receive_while_ntp_is_record_only() -> None:
    left = DetectionSnapshot(
        **{
            **_detection("left", 1.0).__dict__,
            "timestamp_metadata_items": (
                ("app_receive_monotonic_ns", 1_000_000_000),
                ("rtcp_frame_ntp_unix_s", 100.0),
                ("media_recovery_basis", "wiris_non_idr_intra_experimental"),
                ("media_integrity_mode", "experimental_non_idr_intra"),
                ("media_random_access_guaranteed", False),
            ),
        }
    )
    right = DetectionSnapshot(
        **{
            **_detection("right", 1.02).__dict__,
            "timestamp_metadata_items": (
                ("app_receive_monotonic_ns", 1_020_000_000),
                ("rtcp_frame_ntp_unix_s", 32_500.0),
                ("media_recovery_basis", "wiris_non_idr_intra_experimental"),
                ("media_integrity_mode", "experimental_non_idr_intra"),
                ("media_random_access_guaranteed", False),
            ),
        }
    )
    panel = SimpleNamespace(
        _detection_utc_unix_s=lambda detection: detection.timestamp_metadata.get(
            "rtcp_frame_ntp_unix_s"
        ),
        _unix_seconds_to_ns=lambda value: None
        if value is None
        else int(round(float(value) * 1_000_000_000.0)),
        _yolo_pair_max_delta_s=lambda: 0.100,
        _camera_time_sync_metrics=lambda *_args, **_kwargs: {
            "status": "pair-delta-failed",
            "basis": "rtcp-sender-report-ntp",
            "warning": "record-only NTP drift",
            "pair_delta_s": 32_400.0,
            "max_pc_offset_ms": None,
            "max_frame_transit_delay_ms": None,
            "left_frame_transit_delay_ms": None,
            "right_frame_transit_delay_ms": None,
            "left_pc_offset_ms": None,
            "right_pc_offset_ms": None,
        },
        _fmt_record_float=lambda value, _digits=6: value,
        _fmt_record_bool=lambda value: bool(value),
        camera_sync_status="failed",
        camera_sync_reason="NTP drift",
    )

    row = RecordingController(panel)._record_measurement_sync_timestamp_parameters(
        SimpleNamespace(left=left, right=right)
    )

    assert row["pairing_time_basis"] == "app_receive_monotonic"
    assert row["selected_pair_delta_ns"] == 20_000_000
    assert row["sync_delta_ns"] == 20_000_000
    assert row["selected_pair_abs_delta_ns"] == 20_000_000
    assert row["pair_timing_valid"] is True
    assert row["rtcp_pair_delta_ns"] == 32_400_000_000_000
    assert row["ntp_evidence_role"] == "record_only"
    assert row["left_bbox_timestamp_reference"] == "app-receive-monotonic"

    frame_row = RecordingController(panel)._record_frame_timestamp_parameters(
        SimpleNamespace(left_detection=left, right_detection=right)
    )
    assert frame_row["left_media_recovery_basis"] == (
        "wiris_non_idr_intra_experimental"
    )
    assert frame_row["right_media_integrity_mode"] == (
        "experimental_non_idr_intra"
    )
    assert frame_row["left_media_random_access_guaranteed"] is False


def test_recording_row_persists_requested_effective_and_fallback_truth() -> None:
    def snapshot(side: str, app_s: float, app_ns: int, ntp_s: float) -> DetectionSnapshot:
        return DetectionSnapshot(
            **{
                **_detection(side, app_s).__dict__,
                "pairing_timestamp_basis": "camera_rtcp",
                "pairing_timestamp_s": ntp_s,
                "timestamp_metadata_items": (
                    ("app_receive_monotonic_ns", app_ns),
                    ("rtcp_frame_ntp_unix_s", ntp_s),
                    ("clock_epoch_id", "epoch-1"),
                    ("clock_epoch_state", "current"),
                ),
            }
        )

    left = snapshot("left", 1.0, 1_000_000_000, 100.0)
    right = snapshot("right", 1.09, 1_090_000_000, 100.01)
    panel = SimpleNamespace(
        _detection_utc_unix_s=lambda detection: detection.timestamp_metadata.get(
            "rtcp_frame_ntp_unix_s"
        ),
        _unix_seconds_to_ns=lambda value: None
        if value is None
        else int(round(float(value) * 1_000_000_000.0)),
        _yolo_pair_max_delta_s=lambda: 0.100,
        _camera_time_sync_metrics=lambda *_args, **_kwargs: {
            "status": "within-relative-threshold",
            "basis": "rtcp-sender-report-ntp",
            "warning": "",
            "pair_delta_s": 0.01,
            "max_pc_offset_ms": None,
            "max_frame_transit_delay_ms": None,
            "left_frame_transit_delay_ms": None,
            "right_frame_transit_delay_ms": None,
            "left_pc_offset_ms": None,
            "right_pc_offset_ms": None,
        },
        _fmt_record_float=lambda value, _digits=6: value,
        _fmt_record_bool=lambda value: bool(value),
        camera_sync_status="verified",
        camera_sync_reason="current",
    )
    writer = RecordingController(panel)

    rtcp_row = writer._record_measurement_sync_timestamp_parameters(
        SimpleNamespace(
            left_detection=left,
            right_detection=right,
            timestamp_basis="camera_rtcp",
            requested_timestamp_basis="camera_rtcp",
            timing_fallback_reason="",
        )
    )

    assert rtcp_row["pairing_time_basis_requested"] == "camera_rtcp"
    assert rtcp_row["pairing_time_basis"] == "camera_rtcp"
    assert rtcp_row["selected_pair_delta_ns"] == 10_000_000
    assert rtcp_row["app_receive_pair_delta_ns"] == 90_000_000
    assert rtcp_row["ntp_evidence_role"] == "selected_distance_pairing"
    assert rtcp_row["pairing_fallback_active"] is False

    fallback_row = writer._record_measurement_sync_timestamp_parameters(
        SimpleNamespace(
            left_detection=left,
            right_detection=right,
            timestamp_basis="app_receive_monotonic",
            requested_timestamp_basis="camera_rtcp",
            timing_fallback_reason="CLOCK_NOT_VERIFIED",
        )
    )

    assert fallback_row["pairing_time_basis_requested"] == "camera_rtcp"
    assert fallback_row["pairing_time_basis"] == "app_receive_monotonic"
    assert fallback_row["selected_pair_delta_ns"] == 90_000_000
    assert fallback_row["pairing_fallback_active"] is True
    assert fallback_row["pairing_fallback_reason"] == "CLOCK_NOT_VERIFIED"
    assert fallback_row["ntp_evidence_role"] == "record_only"


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
    assert "requested_timestamp_basis" not in persisted
    assert "timing_fallback_reason" not in persisted
    recovered = MeasurementAttempt.from_canonical_payload(persisted)

    assert recovered.canonical_payload() == attempt.canonical_payload()
    assert recovered.canonical_sha256() == attempt.canonical_sha256()


def test_fallback_attempt_round_trip_retains_additive_timing_provenance() -> None:
    attempt = MeasurementAttempt.from_canonical_payload(
        {
            **_attempt().canonical_payload(),
            "requested_timestamp_basis": "camera_rtcp",
            "timing_fallback_reason": "CLOCK_NOT_VERIFIED",
        }
    )

    payload = attempt.canonical_payload()

    assert payload["requested_timestamp_basis"] == "camera_rtcp"
    assert payload["timing_fallback_reason"] == "CLOCK_NOT_VERIFIED"
    assert MeasurementAttempt.from_canonical_payload(payload).canonical_payload() == payload


def test_detection_snapshot_retains_descriptor_timestamp_for_independent_recalculation() -> None:
    metadata = {
        "camera_id": "left",
        "stream_generation": 7,
        "frame_id": 11,
        "frame_key": "left:7:11",
        "timestamp_s": 10.25,
        "source_timestamp_s": 9.75,
        "native_width": 640,
        "native_height": 480,
        "native_channels": 3,
        "native_byte_length": 640 * 480 * 3,
        "native_pixel_format": "BGR24",
        "source_native_frame_id": "left:7:11",
    }
    descriptor_identity = {
        key: metadata[key]
        for key in (
            "camera_id",
            "stream_generation",
            "frame_id",
            "frame_key",
            "timestamp_s",
            "source_timestamp_s",
            "native_width",
            "native_height",
            "native_channels",
            "native_byte_length",
            "native_pixel_format",
        )
    }
    metadata["frame_descriptor_sha256"] = hashlib.sha256(
        json.dumps(
            descriptor_identity,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
    ).hexdigest()
    observation = SimpleNamespace(
        camera="left",
        timestamp_s=10.25,
        width=640,
        height=480,
        x1=10.0,
        y1=20.0,
        x2=30.0,
        y2=40.0,
        confidence=0.9,
        class_id=0,
        class_name="target",
        timestamp_metadata=metadata,
        candidate_class_ids=(0,),
    )

    snapshot = DetectionSnapshot.from_observation(
        observation,
        pairing_timestamp_basis="app_receive_monotonic",
        pairing_timestamp_s=10.25,
    )
    persisted_metadata = dict(snapshot.timestamp_metadata_items)

    assert persisted_metadata["timestamp_s"] == 10.25
    persisted_identity = {
        key: persisted_metadata[key]
        for key in descriptor_identity
    }
    assert hashlib.sha256(
        json.dumps(
            persisted_identity,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
    ).hexdigest() == persisted_metadata["frame_descriptor_sha256"]


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


def test_async_native_frame_result_recovers_after_row_index_cache_eviction(
    tmp_path,
) -> None:
    def pane(side: str, timestamp_s: float, color: str) -> SimpleNamespace:
        image = QtGui.QImage(8, 6, QtGui.QImage.Format_RGB888)
        image.fill(QtGui.QColor(color))

        def native_measurement_frame(requested_s: float):
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

    logs: list[str] = []
    panel = SimpleNamespace(
        recording_session_id="cache-eviction-session",
        _recording_session_dir=lambda _session_id: tmp_path / "cache-eviction-session",
        left_pane=pane("left", 5.0, "red"),
        right_pane=pane("right", 5.01, "blue"),
        distance_recording_rows=[],
        _recording_pair_row_index={},
        _recording_pending_native_request_rows={},
        _refresh_recording_table=lambda: None,
        _log=logs.append,
    )
    controller = RecordingController(panel)

    key = ("cache-key", 5.0, 5.01)
    result = controller._persist_native_detection_pair(
        left_detection=_detection("left", 5.0),
        right_detection=_detection("right", 5.01),
        pair_id="pair-eviction",
        purpose="accepted-measurement",
        record_key=key,
    )
    assert result["native_pair_status"] == "pending"
    row = {"session_id": panel.recording_session_id, **result}
    panel.distance_recording_rows.append(row)
    controller._track_pending_native_row(0, row)

    deadline_s = time.monotonic() + 2.0
    while controller.native_frame_persistence_pending_count() and time.monotonic() < deadline_s:
        time.sleep(0.01)
    assert controller.native_frame_persistence_pending_count() == 0

    assert panel.distance_recording_rows[0]["native_pair_status"] == "complete"
    request_id = str(result["native_pair_request_id"])
    assert request_id not in panel._recording_pending_native_request_rows
    assert not logs
    assert controller._shutdown_native_frame_stores() is True


def test_early_native_completion_releases_barrier_when_row_binding_never_happens(
    tmp_path,
) -> None:
    def pane(side: str, timestamp_s: float, color: str) -> SimpleNamespace:
        image = QtGui.QImage(8, 6, QtGui.QImage.Format_RGB888)
        image.fill(QtGui.QColor(color))
        return SimpleNamespace(
            native_measurement_frame=lambda _requested_s: (
                timestamp_s,
                image,
                {
                    "timestamp_s": timestamp_s,
                    "timestamp_reference": "app-receive",
                    "native_width": image.width(),
                    "native_height": image.height(),
                    "overlay_applied": False,
                    "source_native_frame_id": f"native-{side}",
                },
            )
        )

    logs: list[str] = []
    events: list[tuple[str, dict[str, object]]] = []
    panel = SimpleNamespace(
        recording_session_id="early-completion-session",
        _measurement_session_hierarchy_enabled=False,
        _recording_session_dir=lambda _session_id: tmp_path / "early-completion-session",
        left_pane=pane("left", 7.0, "red"),
        right_pane=pane("right", 7.01, "blue"),
        distance_recording_rows=[],
        _recording_pair_row_index={},
        _recording_pending_native_request_rows={},
        _refresh_recording_table=lambda: None,
        append_workstation_data=lambda stream, payload, **_kwargs: events.append(
            (str(stream), dict(payload))
        )
        or True,
        _log=logs.append,
    )
    controller = RecordingController(panel)
    result = controller._persist_native_detection_pair(
        left_detection=_detection("left", 7.0),
        right_detection=_detection("right", 7.01),
        pair_id="pair-early",
        purpose="accepted-measurement",
        record_key=("row-construction-will-fail",),
    )
    request_id = str(result["native_pair_request_id"])

    deadline_s = time.monotonic() + 2.0
    while (
        request_id not in controller._deferred_native_frame_results
        and time.monotonic() < deadline_s
    ):
        time.sleep(0.01)

    assert request_id in controller._deferred_native_frame_results
    assert events == [
        (
            "native.frames",
            {
                "event_type": "native_frame_pair_persisted",
                "runtime_generation": "early-completion-session",
                "request_id": request_id,
                "native_request_context": {
                    "owner_runtime_generation": "early-completion-session",
                    "request_id": request_id,
                    "record_key": ["row-construction-will-fail"],
                    "writer_kind": "legacy",
                },
                **dict(controller._deferred_native_frame_results[request_id]["fields"]),
            },
        )
    ]
    assert controller.native_frame_persistence_pending_count() == 0
    assert controller._shutdown_native_frame_stores() is True
    assert request_id not in controller._deferred_native_frame_results
    assert any("행 등록 전" in message for message in logs)


def test_old_native_completion_after_next_run_updates_original_row_owner() -> None:
    events: list[tuple[str, dict[str, object]]] = []
    refreshes: list[str] = []
    old_row = {
        "session_id": "run-old",
        "native_pair_status": "pending",
        "native_pair_request_id": "request-old",
    }
    old_rows = [old_row]
    new_rows = [
        {
            "session_id": "run-new",
            "native_pair_status": "pending",
            "native_pair_request_id": "request-new",
        }
    ]
    panel = SimpleNamespace(
        recording_session_id="run-old",
        _measurement_session_hierarchy_enabled=False,
        distance_recording_rows=old_rows,
        _recording_pending_native_request_rows={},
        _native_frame_async_stores={},
        append_workstation_data=lambda stream, payload, **_kwargs: events.append(
            (str(stream), dict(payload))
        )
        or True,
        _refresh_recording_table=lambda: refreshes.append("current"),
        _log=lambda _message: None,
    )
    controller = RecordingController(panel)
    controller._track_pending_native_row(0, old_row)

    panel.recording_session_id = "run-new"
    panel.distance_recording_rows = new_rows
    panel._recording_pending_native_request_rows = {}
    controller._handle_async_native_frame_result(
        {
            "request_id": "request-old",
            "context": {
                "session_id": "run-old",
                "record_key": ["old-pair"],
                "writer_kind": "legacy",
            },
            "fields": {
                "native_pair_status": "complete",
                "native_left_png_sha256": "old-left",
                "native_right_png_sha256": "old-right",
            },
        }
    )

    assert old_rows[0]["native_pair_status"] == "complete"
    assert old_rows[0]["native_left_png_sha256"] == "old-left"
    assert old_rows[0]["native_right_png_sha256"] == "old-right"
    assert new_rows[0]["native_pair_status"] == "pending"
    assert "native_left_png_sha256" not in new_rows[0]
    assert refreshes == []
    assert "request-old" not in controller._native_row_owners
    assert "request-old" not in controller._deferred_native_frame_results
    assert events == [
        (
            "native.frames",
            {
                "event_type": "native_frame_pair_persisted",
                "runtime_generation": "run-old",
                "request_id": "request-old",
                "native_request_context": {
                    "owner_runtime_generation": "run-old",
                    "request_id": "request-old",
                    "record_key": ["old-pair"],
                    "writer_kind": "legacy",
                },
                "native_pair_status": "complete",
                "native_left_png_sha256": "old-left",
                "native_right_png_sha256": "old-right",
                "native_completion_sha256": (
                    controller._native_terminal_fingerprints[
                        "request-old"
                    ].canonical_sha256
                ),
            },
        )
    ]


def test_native_completion_lease_rejects_clear_and_index_reuse() -> None:
    class Store:
        def enqueue(self, _stream, _payload, *, priority=None):
            return True

        def snapshot(self):
            return {"lifecycle_state": "OPEN", "writer_alive": True, "streams": {}}

    rows = LiveDataRows(Store(), "measurements", priority=True)
    old_row = {
        "session_id": "run-old",
        "native_pair_status": "pending",
        "native_pair_request_id": "request-reused",
        "sentinel": "old-row",
    }
    old_admission = rows.append_with_admission(old_row)
    assert old_admission.admitted is True
    assert old_admission.lease is not None
    events: list[tuple[str, dict[str, object]]] = []
    refreshes: list[str] = []
    logs: list[str] = []
    panel = SimpleNamespace(
        recording_session_id="run-old",
        _measurement_session_hierarchy_enabled=False,
        distance_recording_rows=rows,
        _recording_pending_native_request_rows={},
        _native_frame_async_stores={},
        append_workstation_data=lambda stream, payload, **_kwargs: events.append(
            (str(stream), dict(payload))
        )
        or True,
        _refresh_recording_table=lambda: refreshes.append("current"),
        _log=logs.append,
    )
    controller = RecordingController(panel)
    controller._track_pending_native_row(
        int(old_admission.row_index or 0),
        old_row,
        row_lease=old_admission.lease,
    )

    rows.clear()
    new_admission = rows.append_with_admission(
        {
            "session_id": "run-new",
            "native_pair_status": "pending",
            "native_pair_request_id": "request-reused",
            "sentinel": "new-row",
        }
    )
    assert new_admission.admitted is True
    assert new_admission.row_index == old_admission.row_index
    assert new_admission.lease != old_admission.lease
    panel.recording_session_id = "run-new"
    panel._recording_pending_native_request_rows = {}

    controller._handle_async_native_frame_result(
        {
            "request_id": "request-reused",
            "context": {
                "session_id": "run-old",
                "record_key": ["old-pair"],
                "writer_kind": "legacy",
            },
            "fields": {
                "native_pair_status": "complete",
                "native_left_png_sha256": "old-left",
            },
        }
    )

    assert rows[0]["sentinel"] == "new-row"
    assert rows[0]["native_pair_status"] == "pending"
    assert "native_left_png_sha256" not in rows[0]
    assert refreshes == []
    assert "request-reused" not in controller._native_row_owners
    assert events[0][0] == "native.frames"
    assert events[0][1]["event_type"] == "native_frame_pair_persisted"
    assert events[0][1]["runtime_generation"] == "run-old"
    assert events[1][0] == "native.frames"
    assert events[1][1]["event_type"] == (
        "native_frame_row_projection_unavailable"
    )
    assert events[1][1]["native_row_projection_state"] == "UNAVAILABLE"
    assert events[1][1]["native_row_expected_lease"] == list(
        old_admission.lease
    )
    assert events[1][1]["native_row_current_lease"] == list(
        new_admission.lease
    )
    assert any("row lease" in message for message in logs)


def test_native_completion_without_atomic_owner_lease_uses_canonical_event_only() -> None:
    class Store:
        def enqueue(self, _stream, _payload, *, priority=None):
            return True

        def snapshot(self):
            return {"lifecycle_state": "OPEN", "writer_alive": True, "streams": {}}

    rows = LiveDataRows(Store(), "measurements", priority=True)
    row = {
        "session_id": "run-old",
        "native_pair_status": "pending",
        "native_pair_request_id": "request-no-lease",
    }
    admission = rows.append_with_admission(row)
    assert admission.admitted is True
    events: list[tuple[str, dict[str, object]]] = []
    refreshes: list[str] = []
    panel = SimpleNamespace(
        recording_session_id="run-old",
        _measurement_session_hierarchy_enabled=True,
        distance_recording_rows=rows,
        _recording_pending_native_request_rows={},
        _native_frame_async_stores={},
        append_workstation_data=lambda stream, payload, **_kwargs: events.append(
            (str(stream), dict(payload))
        )
        or True,
        _refresh_recording_table=lambda: refreshes.append("current"),
        _log=lambda _message: None,
    )
    controller = RecordingController(panel)
    controller._track_pending_native_row(0, row)

    controller._handle_async_native_frame_result(
        {
            "request_id": "request-no-lease",
            "context": {
                "session_id": "run-old",
                "record_key": ["old-pair"],
                "writer_kind": "legacy",
            },
            "fields": {
                "native_pair_status": "complete",
                "native_left_png_sha256": "old-left",
            },
        }
    )

    assert rows[0]["native_pair_status"] == "pending"
    assert "native_left_png_sha256" not in rows[0]
    assert refreshes == []
    assert len(events) == 1
    assert events[0][0] == "native.frames"
    assert events[0][1]["event_type"] == "native_frame_pair_persisted"
    assert events[0][1]["native_pair_status"] == "complete"
    assert events[0][1]["native_row_projection_state"] == "UNAVAILABLE"
    assert events[0][1]["native_row_projection_reason"] == (
        "native row append lease is unavailable"
    )


def test_native_completion_retryable_projection_forces_canonical_event() -> None:
    admissions = iter((True, False))

    class Store:
        def enqueue(self, _stream, _payload, *, priority=None):
            return next(admissions)

        def snapshot(self):
            return {"lifecycle_state": "OPEN", "writer_alive": True, "streams": {}}

    rows = LiveDataRows(Store(), "measurements", priority=True)
    row = {
        "session_id": "run-old",
        "native_pair_status": "pending",
        "native_pair_request_id": "request-retryable",
    }
    admission = rows.append_with_admission(row)
    assert admission.admitted is True
    assert admission.lease is not None
    events: list[tuple[str, dict[str, object]]] = []
    logs: list[str] = []
    panel = SimpleNamespace(
        recording_session_id="run-old",
        _measurement_session_hierarchy_enabled=True,
        distance_recording_rows=rows,
        _recording_pending_native_request_rows={},
        _native_frame_async_stores={},
        append_workstation_data=lambda stream, payload, **_kwargs: events.append(
            (str(stream), dict(payload))
        )
        or True,
        _refresh_recording_table=lambda: None,
        _log=logs.append,
    )
    controller = RecordingController(panel)
    controller._track_pending_native_row(0, row, row_lease=admission.lease)

    controller._handle_async_native_frame_result(
        {
            "request_id": "request-retryable",
            "context": {
                "session_id": "run-old",
                "record_key": ["old-pair"],
                "writer_kind": "legacy",
            },
            "fields": {
                "native_pair_status": "complete",
                "native_left_png_sha256": "old-left",
            },
        }
    )

    assert rows[0]["native_pair_status"] == "complete"
    assert len(events) == 1
    assert events[0][0] == "native.frames"
    assert events[0][1]["event_type"] == "native_frame_pair_persisted"
    assert events[0][1]["native_row_projection_state"] == "RETRYABLE"
    assert events[0][1]["native_row_expected_lease"] == list(admission.lease)
    assert events[0][1]["native_row_current_lease"] == list(admission.lease)
    assert any("enqueue 지연" in message for message in logs)


def test_native_completion_dual_rejection_retains_owner_and_retries_after_next_run() -> None:
    row_admissions = iter((True, False, False))

    class Store:
        def enqueue(self, _stream, _payload, *, priority=None):
            return next(row_admissions)

        def snapshot(self):
            return {"lifecycle_state": "OPEN", "writer_alive": True, "streams": {}}

    rows = LiveDataRows(Store(), "measurements", priority=True)
    row = {
        "session_id": "run-old",
        "native_pair_status": "pending",
        "native_pair_request_id": "request-dual-reject",
    }
    admission = rows.append_with_admission(row)
    assert admission.admitted is True
    assert admission.lease is not None
    event_admissions = iter((False, True))
    event_attempts: list[dict[str, object]] = []
    acknowledgements: list[str] = []

    def append_workstation_data(_stream, payload, **_kwargs):
        event_attempts.append(dict(payload))
        return next(event_admissions)

    panel = SimpleNamespace(
        recording_session_id="run-old",
        _measurement_session_hierarchy_enabled=True,
        distance_recording_rows=rows,
        _recording_pending_native_request_rows={},
        _native_frame_async_stores={},
        append_workstation_data=append_workstation_data,
        _refresh_recording_table=lambda: None,
        _log=lambda _message: None,
    )
    controller = RecordingController(panel)
    controller._native_result_store = lambda _context: SimpleNamespace(
        acknowledge=acknowledgements.append
    )
    controller._track_pending_native_row(
        0,
        row,
        row_lease=admission.lease,
    )
    payload = {
        "request_id": "request-dual-reject",
        "context": {
            "session_id": "run-old",
            "record_key": ["pair-old"],
            "writer_kind": "legacy",
        },
        "fields": {
            "native_pair_status": "complete",
            "native_left_png_sha256": "first-terminal",
        },
    }

    controller._handle_async_native_frame_result(payload)

    retained = controller._pending_native_completions["request-dual-reject"]
    assert retained.owner is not None
    assert retained.owner.rows is rows
    assert retained.owner.row_lease == admission.lease
    assert retained.payload["fields"] == payload["fields"]
    assert acknowledgements == ["request-dual-reject"]
    assert controller.native_frame_persistence_pending_count() == 1

    new_rows = [{"session_id": "run-new", "sentinel": "next-run"}]
    panel.recording_session_id = "run-new"
    panel.distance_recording_rows = new_rows
    retained.next_retry_monotonic_s = 0.0
    controller._retry_pending_native_completions()

    assert controller._pending_native_completions == {}
    assert controller.native_frame_persistence_pending_count() == 0
    assert acknowledgements == ["request-dual-reject", "request-dual-reject"]
    assert len(event_attempts) == 2
    assert event_attempts[-1]["runtime_generation"] == "run-old"
    assert event_attempts[-1]["native_pair_status"] == "complete"
    assert rows[0]["native_left_png_sha256"] == "first-terminal"
    assert new_rows == [{"session_id": "run-new", "sentinel": "next-run"}]


def test_native_completion_permanent_rejection_terminalizes_without_blocking_close(
    tmp_path,
) -> None:
    row_enqueue_count = 0

    class Store:
        def enqueue(self, _stream, _payload, *, priority=None):
            nonlocal row_enqueue_count
            row_enqueue_count += 1
            return row_enqueue_count == 1

        def snapshot(self):
            return {"lifecycle_state": "OPEN", "writer_alive": True, "streams": {}}

    rows = LiveDataRows(Store(), "measurements", priority=True)
    row = {
        "session_id": "run-drop",
        "native_pair_status": "pending",
        "native_pair_request_id": "request-drop",
    }
    admission = rows.append_with_admission(row)
    assert admission.admitted is True
    logs: list[str] = []
    native_store = AsyncNativeFrameStore(tmp_path / "native-drop")
    panel = SimpleNamespace(
        recording_session_id="run-drop",
        _measurement_session_hierarchy_enabled=True,
        distance_recording_rows=rows,
        _recording_pending_native_request_rows={},
        _native_frame_async_stores={"run-drop": native_store},
        append_workstation_data=lambda *_args, **_kwargs: False,
        _refresh_recording_table=lambda: None,
        _log=logs.append,
    )
    controller = RecordingController(panel)
    controller._track_pending_native_row(0, row, row_lease=admission.lease)
    controller._handle_async_native_frame_result(
        {
            "request_id": "request-drop",
            "context": {
                "session_id": "run-drop",
                "record_key": ["pair-drop"],
                "writer_kind": "legacy",
            },
            "fields": {
                "native_pair_status": "complete",
                "native_left_png_sha256": "terminal-drop",
            },
        }
    )
    retained = controller._pending_native_completions["request-drop"]
    assert controller._shutdown_native_frame_stores() is False
    assert panel._native_frame_async_stores == {"run-drop": native_store}
    retained.attempts = 1_000_000
    retained.next_retry_monotonic_s = 0.0

    assert controller._shutdown_native_frame_stores() is True

    assert controller._pending_native_completions == {}
    assert controller.native_frame_persistence_pending_count() == 0
    assert controller._native_completion_terminal_drop_count == 1
    assert panel._native_completion_terminal_drop_count == 1
    dropped = controller._native_completion_terminal_drops[-1]
    assert dropped["request_id"] == "request-drop"
    assert dropped["status_event_admitted"] is False
    assert dropped["payload"]["fields"]["native_left_png_sha256"] == (
        "terminal-drop"
    )
    assert dropped["owner_row_lease"] == list(admission.lease)
    assert any("terminal drop" in message for message in logs)
    assert panel._native_frame_async_stores == {}


def test_persistence_required_native_completion_is_not_consumed_by_primary_row_cache() -> None:
    panel = SimpleNamespace(
        recording_session_id="run-old",
        _measurement_session_hierarchy_enabled=True,
        distance_recording_rows=[],
        _recording_pair_row_index={},
        _recording_pending_native_request_rows={},
        _native_frame_async_stores={},
        append_workstation_data=lambda *_args, **_kwargs: False,
        _refresh_recording_table=lambda: None,
        _log=lambda _message: None,
    )
    controller = RecordingController(panel)
    key = ("pair-old",)
    controller._remember_pending_primary_row(
        key,
        {
            "native_pair_request_id": "request-deferred-required",
            "native_pair_status": "pending",
            "measurement_attempt_sha256": "a" * 64,
        },
    )
    controller._handle_async_native_frame_result(
        {
            "request_id": "request-deferred-required",
            "context": {
                "session_id": "run-old",
                "record_key": list(key),
                "writer_kind": "legacy",
            },
            "fields": {
                "native_pair_status": "complete",
                "native_left_png_sha256": "terminal",
            },
        }
    )

    assert "request-deferred-required" in controller._pending_native_completions
    retry_row = controller._pending_primary_retry_row(key)

    assert retry_row is not None
    assert retry_row["native_pair_status"] == "complete"
    assert retry_row["native_left_png_sha256"] == "terminal"
    assert retry_row["native_completion_sha256"] == (
        controller._native_terminal_fingerprints[
            "request-deferred-required"
        ].canonical_sha256
    )
    assert "request-deferred-required" not in controller._deferred_native_frame_results
    assert "request-deferred-required" in controller._pending_native_completions

    controller._finish_native_completion_from_admitted_row(retry_row)

    assert controller._pending_native_completions == {}


def test_native_completion_duplicate_is_noop_and_conflict_is_first_terminal_wins() -> None:
    row = {
        "session_id": "run-one",
        "native_pair_status": "pending",
        "native_pair_request_id": "request-fingerprint",
    }
    rows = [row]
    admissions = iter((True, False, True))
    event_attempts: list[dict[str, object]] = []
    admitted_events: list[dict[str, object]] = []

    def append_workstation_data(_stream, payload, **_kwargs):
        record = dict(payload)
        event_attempts.append(record)
        admitted = next(admissions)
        if admitted:
            admitted_events.append(record)
        return admitted

    panel = SimpleNamespace(
        recording_session_id="run-one",
        _measurement_session_hierarchy_enabled=False,
        distance_recording_rows=rows,
        _recording_pending_native_request_rows={},
        _native_frame_async_stores={},
        append_workstation_data=append_workstation_data,
        _refresh_recording_table=lambda: None,
        _log=lambda _message: None,
    )
    controller = RecordingController(panel)
    controller._track_pending_native_row(0, row)
    first = {
        "request_id": "request-fingerprint",
        "context": {
            "session_id": "run-one",
            "record_key": ["pair-one"],
            "writer_kind": "legacy",
        },
        "fields": {
            "native_pair_status": "complete",
            "native_left_png_sha256": "authoritative",
        },
    }
    conflicting = {
        **first,
        "fields": {
            "native_pair_status": "failed",
            "native_left_png_sha256": "conflicting",
        },
    }

    controller._handle_async_native_frame_result(first)
    controller._handle_async_native_frame_result(first)
    assert len(event_attempts) == 1
    assert rows[0]["native_left_png_sha256"] == "authoritative"

    controller._handle_async_native_frame_result(conflicting)
    controller._handle_async_native_frame_result(conflicting)
    assert len(event_attempts) == 2
    assert "request-fingerprint" in controller._pending_native_conflicts
    assert rows[0]["native_pair_status"] == "complete"
    assert rows[0]["native_left_png_sha256"] == "authoritative"

    conflict = controller._pending_native_conflicts["request-fingerprint"]
    conflict.next_retry_monotonic_s = 0.0
    controller._retry_pending_native_completions()

    assert controller._pending_native_conflicts == {}
    assert controller._native_conflicts_recorded == {"request-fingerprint"}
    assert len(event_attempts) == 3
    assert len(admitted_events) == 2
    diagnostic = admitted_events[-1]
    assert diagnostic["event_type"] == "native_frame_completion_conflict"
    assert diagnostic["native_completion_authority"] == "first_terminal_wins"
    assert diagnostic["native_completion_authoritative_sha256"] != (
        diagnostic["native_completion_rejected_sha256"]
    )
    assert rows[0]["native_left_png_sha256"] == "authoritative"


def test_native_completion_conflict_permanent_rejection_records_one_terminal_drop() -> None:
    row = {
        "session_id": "run-conflict-drop",
        "native_pair_status": "pending",
        "native_pair_request_id": "request-conflict-drop",
    }
    event_attempts: list[dict[str, object]] = []

    def append_workstation_data(_stream, payload, **_kwargs):
        event_attempts.append(dict(payload))
        return len(event_attempts) == 1

    panel = SimpleNamespace(
        recording_session_id="run-conflict-drop",
        _measurement_session_hierarchy_enabled=False,
        distance_recording_rows=[row],
        _recording_pending_native_request_rows={},
        _native_frame_async_stores={},
        append_workstation_data=append_workstation_data,
        _refresh_recording_table=lambda: None,
        _log=lambda _message: None,
    )
    controller = RecordingController(panel)
    controller._track_pending_native_row(0, row)
    first = {
        "request_id": "request-conflict-drop",
        "context": {
            "session_id": "run-conflict-drop",
            "record_key": ["pair-conflict-drop"],
            "writer_kind": "legacy",
        },
        "fields": {
            "native_pair_status": "complete",
            "native_left_png_sha256": "first",
        },
    }
    conflicting = {
        **first,
        "fields": {
            "native_pair_status": "failed",
            "native_left_png_sha256": "rejected",
        },
    }
    controller._handle_async_native_frame_result(first)
    controller._handle_async_native_frame_result(conflicting)
    conflict = controller._pending_native_conflicts["request-conflict-drop"]
    conflict.attempts = 1_000_000
    conflict.next_retry_monotonic_s = 0.0

    controller._retry_pending_native_completions()
    attempts_after_drop = len(event_attempts)
    controller._handle_async_native_frame_result(conflicting)

    assert controller._pending_native_conflicts == {}
    assert controller._native_conflicts_recorded == {"request-conflict-drop"}
    assert controller._native_completion_terminal_drop_count == 1
    assert len(event_attempts) == attempts_after_drop
    dropped = controller._native_completion_terminal_drops[-1]
    assert dropped["event_type"] == (
        "native_frame_completion_conflict_persistence_dropped"
    )
    assert dropped["request_id"] == "request-conflict-drop"
    assert panel.distance_recording_rows[0]["native_pair_status"] == "complete"
    assert panel.distance_recording_rows[0]["native_left_png_sha256"] == "first"


def test_async_native_frame_stale_completion_cannot_overwrite_newer_request() -> None:
    logs: list[str] = []
    panel = SimpleNamespace(
        recording_session_id="session-1",
        distance_recording_rows=[
            {
                "session_id": "session-1",
                "native_pair_status": "pending",
                "native_pair_request_id": "request-new",
            }
        ],
        _recording_pending_native_request_rows={
            "request-old": ("session-1", 0),
            "request-new": ("session-1", 0),
        },
        _native_frame_async_stores={},
        _refresh_recording_table=lambda: None,
        _log=logs.append,
    )
    controller = RecordingController(panel)
    controller._track_pending_native_row(
        0,
        {
            "session_id": "session-1",
            "native_pair_status": "pending",
            "native_pair_request_id": "request-old",
        },
    )
    controller._track_pending_native_row(0, panel.distance_recording_rows[0])
    context = {"session_id": "session-1", "record_key": ["same-pair"]}

    controller._handle_async_native_frame_result(
        {
            "request_id": "request-old",
            "context": context,
            "fields": {
                "native_pair_status": "complete",
                "native_left_png_sha256": "old",
            },
        }
    )
    assert panel.distance_recording_rows[0]["native_pair_status"] == "pending"
    assert "native_left_png_sha256" not in panel.distance_recording_rows[0]

    controller._handle_async_native_frame_result(
        {
            "request_id": "request-new",
            "context": context,
            "fields": {
                "native_pair_status": "complete",
                "native_left_png_sha256": "new",
            },
        }
    )
    assert panel.distance_recording_rows[0]["native_pair_status"] == "complete"
    assert panel.distance_recording_rows[0]["native_left_png_sha256"] == "new"
    assert controller._native_row_owners == {}
    assert not logs
