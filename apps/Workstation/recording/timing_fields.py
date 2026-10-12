"""Project frozen detection timing, pixel bounds, and telemetry timestamp evidence.

App receive, selected pairing, and record-only RTCP/NTP clocks retain provenance."""
from __future__ import annotations
from decimal import Decimal, InvalidOperation, ROUND_HALF_UP
from math import isfinite
from Workstation.workstation_core.measurement_attempt import LIVE_DISTANCE_MAX_APP_DELTA_NS, MeasurementAttempt
from Workstation.workstation_core.coordinate_contract import BODY_FRAME
from Workstation.workstation_core.runtime_camera_status import infer_runtime_camera_timestamp_reference
from .ports import TimingFieldsPort
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .formatting import Formatting


class TimingFields:
    def __init__(self, *,
        boundary: TimingFieldsPort,
    ) -> None:
        self.boundary = boundary

    def bind_collaborators(self, *,
        formatting: "Formatting",
    ) -> None:
        self.formatting = formatting

    def _record_bbox_parameters(self, estimate) -> dict[str, str]:
        row: dict[str, str] = {}
        for camera in ("left", "right"):
            detection = getattr(estimate, camera, None)
            center = getattr(detection, "center", None)
            if center is None:
                center_x = None
                center_y = None
            else:
                center_x = float(center[0])
                center_y = float(center[1])
            x1 = getattr(detection, "x1", None)
            y1 = getattr(detection, "y1", None)
            x2 = getattr(detection, "x2", None)
            y2 = getattr(detection, "y2", None)
            bbox_width = None
            bbox_height = None
            if x1 is not None and x2 is not None:
                bbox_width = float(x2) - float(x1)
            if y1 is not None and y2 is not None:
                bbox_height = float(y2) - float(y1)
            row[f"{camera}_bbox_x1_px"] = self.formatting._fmt_record_float(getattr(detection, "x1", None), 3)
            row[f"{camera}_bbox_y1_px"] = self.formatting._fmt_record_float(getattr(detection, "y1", None), 3)
            row[f"{camera}_bbox_x2_px"] = self.formatting._fmt_record_float(getattr(detection, "x2", None), 3)
            row[f"{camera}_bbox_y2_px"] = self.formatting._fmt_record_float(getattr(detection, "y2", None), 3)
            row[f"{camera}_bbox_center_x_px"] = self.formatting._fmt_record_float(center_x, 3)
            row[f"{camera}_bbox_center_y_px"] = self.formatting._fmt_record_float(center_y, 3)
            row[f"{camera}_bbox_confidence"] = self.formatting._fmt_record_float(getattr(detection, "confidence", None), 6)
            row[f"{camera}_bbox_class_id"] = "" if getattr(detection, "class_id", None) is None else str(detection.class_id)
            row[f"{camera}_bbox_class_name"] = str(getattr(detection, "class_name", "") or "")
            row[f"{camera}_x1"] = self.formatting._fmt_record_float(x1, 3)
            row[f"{camera}_y1"] = self.formatting._fmt_record_float(y1, 3)
            row[f"{camera}_x2"] = self.formatting._fmt_record_float(x2, 3)
            row[f"{camera}_y2"] = self.formatting._fmt_record_float(y2, 3)
            row[f"{camera}_cx"] = self.formatting._fmt_record_float(center_x, 3)
            row[f"{camera}_cy"] = self.formatting._fmt_record_float(center_y, 3)
            row[f"{camera}_width_px"] = self.formatting._fmt_record_float(bbox_width, 3)
            row[f"{camera}_height_px"] = self.formatting._fmt_record_float(bbox_height, 3)
            row[f"{camera}_confidence"] = self.formatting._fmt_record_float(getattr(detection, "confidence", None), 6)
        return row

    def _detection_utc_unix_s(self, detection) -> float | None:
        metadata = dict(getattr(detection, "timestamp_metadata", None) or {})
        value = metadata.get("rtcp_frame_ntp_unix_s")
        if value is None:
            return None
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            return None
        return parsed if isfinite(parsed) else None

    def _unix_seconds_to_ns(self, value: float | None) -> int | None:
        if value is None:
            return None
        try:
            return int(
                (Decimal(str(value)) * Decimal(1_000_000_000)).to_integral_value(
                    rounding=ROUND_HALF_UP
                )
            )
        except (InvalidOperation, ValueError):
            return None

    def _record_measurement_sync_timestamp_parameters(self, estimate) -> dict[str, object]:
        left_detection = getattr(
            estimate, "left", getattr(estimate, "left_detection", None)
        )
        right_detection = getattr(
            estimate, "right", getattr(estimate, "right_detection", None)
        )
        if left_detection is None or right_detection is None:
            raise ValueError("measurement timing requires Left/Right detections")
        left_timestamp_s = float(getattr(left_detection, "timestamp_s", 0.0))
        right_timestamp_s = float(getattr(right_detection, "timestamp_s", 0.0))
        def app_receive_ns(detection: object, timestamp_s: float) -> int:
            value = getattr(detection, "app_receive_monotonic_ns", None)
            if value is None:
                metadata = dict(
                    getattr(detection, "timestamp_metadata", None) or {}
                )
                value = metadata.get("app_receive_monotonic_ns")
            return (
                int(value)
                if value is not None
                else int(round(timestamp_s * 1_000_000_000.0))
            )

        left_app_ns = app_receive_ns(left_detection, left_timestamp_s)
        right_app_ns = app_receive_ns(right_detection, right_timestamp_s)
        signed_app_delta_ns = right_app_ns - left_app_ns
        bbox_mid_timestamp_s = (left_timestamp_s + right_timestamp_s) / 2.0
        left_utc_unix_s = self._detection_utc_unix_s(left_detection)
        right_utc_unix_s = self._detection_utc_unix_s(right_detection)
        bbox_mid_utc_unix_s = (
            (left_utc_unix_s + right_utc_unix_s) / 2.0
            if left_utc_unix_s is not None and right_utc_unix_s is not None
            else None
        )
        left_utc_ns = self._unix_seconds_to_ns(left_utc_unix_s)
        right_utc_ns = self._unix_seconds_to_ns(right_utc_unix_s)
        pair_mid_utc_ns = (
            None if left_utc_ns is None or right_utc_ns is None else (left_utc_ns + right_utc_ns) // 2
        )
        rtcp_pair_delta_ns = (
            None if left_utc_ns is None or right_utc_ns is None else right_utc_ns - left_utc_ns
        )
        effective_basis = str(
            getattr(
                estimate,
                "timestamp_basis",
                getattr(left_detection, "pairing_timestamp_basis", ""),
            )
            or "app_receive_monotonic"
        )
        if effective_basis == "camera_rtc_required":
            effective_basis = "camera_rtcp"
        requested_basis = str(
            getattr(estimate, "requested_timestamp_basis", "")
            or effective_basis
        )
        if requested_basis == "camera_rtc_required":
            requested_basis = "camera_rtcp"
        fallback_reason = str(
            getattr(estimate, "timing_fallback_reason", "") or ""
        )
        if effective_basis == "camera_rtcp":
            left_selected_s = (
                getattr(left_detection, "pairing_timestamp_s", None)
                if str(getattr(left_detection, "pairing_timestamp_basis", ""))
                in {"camera_rtcp", "camera_rtc_required"}
                else left_utc_unix_s
            )
            right_selected_s = (
                getattr(right_detection, "pairing_timestamp_s", None)
                if str(getattr(right_detection, "pairing_timestamp_basis", ""))
                in {"camera_rtcp", "camera_rtc_required"}
                else right_utc_unix_s
            )
            selected_left_ns = self._unix_seconds_to_ns(left_selected_s)
            selected_right_ns = self._unix_seconds_to_ns(right_selected_s)
            if selected_left_ns is None or selected_right_ns is None:
                raise ValueError("RTCP_TIMESTAMP_UNAVAILABLE")
            selected_delta_ns = selected_right_ns - selected_left_ns
            ntp_evidence_role = "selected_distance_pairing"
            app_evidence_role = "attitude_association_and_fallback"
        else:
            effective_basis = "app_receive_monotonic"
            selected_delta_ns = signed_app_delta_ns
            selected_left_ns = left_app_ns
            selected_right_ns = right_app_ns
            ntp_evidence_role = "record_only"
            app_evidence_role = "selected_distance_pairing"
        selected_abs_delta_ns = abs(selected_delta_ns)
        configured_pair_limit_s = self.boundary._yolo_pair_max_delta_s()
        configured_pair_limit_ns = (
            None
            if not isfinite(configured_pair_limit_s)
            else int(round(configured_pair_limit_s * 1_000_000_000.0))
        )
        pair_limit_ns = (
            LIVE_DISTANCE_MAX_APP_DELTA_NS
            if configured_pair_limit_ns is None
            else min(LIVE_DISTANCE_MAX_APP_DELTA_NS, configured_pair_limit_ns)
        )
        camera_time_metrics = self.boundary._camera_time_sync_metrics(
            dict(getattr(left_detection, "timestamp_metadata", None) or {}),
            dict(getattr(right_detection, "timestamp_metadata", None) or {}),
            left_receive_mono_s=left_timestamp_s,
            right_receive_mono_s=right_timestamp_s,
        )
        row = {
            "left_bbox_timestamp_s": self.formatting._fmt_record_float(left_timestamp_s, 6),
            "right_bbox_timestamp_s": self.formatting._fmt_record_float(right_timestamp_s, 6),
            "bbox_mid_timestamp_s": self.formatting._fmt_record_float(bbox_mid_timestamp_s, 6),
            "left_bbox_utc_unix_s": self.formatting._fmt_record_float(left_utc_unix_s, 6),
            "right_bbox_utc_unix_s": self.formatting._fmt_record_float(right_utc_unix_s, 6),
            "bbox_mid_utc_unix_s": self.formatting._fmt_record_float(bbox_mid_utc_unix_s, 6),
            "left_frame_utc_ns": "" if left_utc_ns is None else left_utc_ns,
            "right_frame_utc_ns": "" if right_utc_ns is None else right_utc_ns,
            "pair_mid_utc_ns": "" if pair_mid_utc_ns is None else pair_mid_utc_ns,
            "pairing_time_basis_requested": requested_basis,
            "pairing_time_basis": effective_basis,
            "pairing_time_basis_role": "authoritative_distance",
            "pairing_fallback_active": self.formatting._fmt_record_bool(
                bool(fallback_reason and requested_basis != effective_basis)
            ),
            "pairing_fallback_reason": fallback_reason,
            "left_app_receive_monotonic_ns": left_app_ns,
            "right_app_receive_monotonic_ns": right_app_ns,
            "app_receive_pair_delta_ns": signed_app_delta_ns,
            "app_receive_evidence_role": app_evidence_role,
            "selected_pair_delta_ns": selected_delta_ns,
            "selected_pair_abs_delta_ns": selected_abs_delta_ns,
            "selected_pair_midpoint_ns": (selected_left_ns + selected_right_ns) // 2,
            "selected_pair_mid_monotonic_ns": (left_app_ns + right_app_ns) // 2,
            "sync_delta_ns": selected_delta_ns,
            "rtcp_pair_delta_ns": "" if rtcp_pair_delta_ns is None else rtcp_pair_delta_ns,
            "ntp_evidence_role": ntp_evidence_role,
            "pair_timing_limit_ns": "" if pair_limit_ns is None else pair_limit_ns,
            "pair_timing_valid": self.formatting._fmt_record_bool(
                selected_abs_delta_ns <= pair_limit_ns
            ),
            "left_bbox_timestamp_reference": "app-receive-monotonic",
            "right_bbox_timestamp_reference": "app-receive-monotonic",
            "camera_time_sync_status": str(camera_time_metrics["status"]),
            "camera_time_sync_basis": str(camera_time_metrics["basis"]),
            "camera_time_sync_warning": str(camera_time_metrics["warning"]),
            "stream_sync_state": self.boundary.camera_sync_status,
            "stream_sync_reason": self.boundary.camera_sync_reason,
            "rtcp_pair_delta_s": self.formatting._fmt_record_float(camera_time_metrics["pair_delta_s"], 6),
            "rtcp_pc_offset_max_ms": self.formatting._fmt_record_float(camera_time_metrics["max_pc_offset_ms"], 3),
            "rtcp_frame_transit_max_ms": self.formatting._fmt_record_float(camera_time_metrics["max_frame_transit_delay_ms"], 3),
            "left_rtcp_frame_transit_ms": self.formatting._fmt_record_float(camera_time_metrics["left_frame_transit_delay_ms"], 3),
            "right_rtcp_frame_transit_ms": self.formatting._fmt_record_float(camera_time_metrics["right_frame_transit_delay_ms"], 3),
            "left_rtcp_pc_offset_ms": self.formatting._fmt_record_float(camera_time_metrics["left_pc_offset_ms"], 3),
            "right_rtcp_pc_offset_ms": self.formatting._fmt_record_float(camera_time_metrics["right_pc_offset_ms"], 3),
            "left_clock_epoch_id": str(
                (getattr(left_detection, "timestamp_metadata", None) or {}).get(
                    "clock_epoch_id"
                )
                or ""
            ),
            "right_clock_epoch_id": str(
                (getattr(right_detection, "timestamp_metadata", None) or {}).get(
                    "clock_epoch_id"
                )
                or ""
            ),
            "left_clock_epoch_state": str(
                (getattr(left_detection, "timestamp_metadata", None) or {}).get(
                    "clock_epoch_state"
                )
                or ""
            ),
            "right_clock_epoch_state": str(
                (getattr(right_detection, "timestamp_metadata", None) or {}).get(
                    "clock_epoch_state"
                )
                or ""
            ),
        }
        return row

    def _record_frame_timestamp_parameters(
        self, measurement_attempt: MeasurementAttempt
    ) -> dict[str, object]:
        row: dict[str, object] = {}
        rtcp_selected = str(
            getattr(measurement_attempt, "timestamp_basis", "app_receive_monotonic")
        ) in {
            "camera_rtcp",
            "camera_rtc_required",
        }
        detections = {
            "left": measurement_attempt.left_detection,
            "right": measurement_attempt.right_detection,
        }
        for camera, detection in detections.items():
            metadata = detection.timestamp_metadata
            timestamp_s = detection.app_receive_monotonic_s
            source_timestamp = metadata.get("source_timestamp_s")
            ntp_timestamp_reference = infer_runtime_camera_timestamp_reference(metadata)
            row[f"{camera}_display_frame_timestamp_s"] = self.formatting._fmt_record_float(timestamp_s, 6)
            row[f"{camera}_source_frame_timestamp_s"] = self.formatting._fmt_record_float(
                None if source_timestamp is None else float(source_timestamp),
                6,
            )
            row[f"{camera}_frame_timestamp_source"] = str(metadata.get("timestamp_source") or "")
            row[f"{camera}_frame_timestamp_reference"] = "app-receive-monotonic"
            row[f"{camera}_ntp_timestamp_reference"] = ntp_timestamp_reference
            row[f"{camera}_ntp_evidence_role"] = (
                "selected_distance_pairing" if rtcp_selected else "record_only"
            )
            row[f"{camera}_app_receive_evidence_role"] = (
                "attitude_association_and_fallback"
                if rtcp_selected
                else "selected_distance_pairing"
            )
            row[f"{camera}_media_recovery_basis"] = str(
                metadata.get("media_recovery_basis") or ""
            )
            row[f"{camera}_media_integrity_mode"] = str(
                metadata.get("media_integrity_mode") or ""
            )
            row[f"{camera}_media_integrity_unverified"] = (
                self.formatting._fmt_record_bool(
                    bool(metadata.get("media_integrity_unverified", False))
                )
            )
            row[f"{camera}_source_media_integrity_verified"] = (
                self.formatting._fmt_record_bool(
                    bool(metadata.get("source_media_integrity_verified", False))
                )
            )
            row[f"{camera}_media_random_access_guaranteed"] = (
                self.formatting._fmt_record_bool(
                    bool(metadata.get("media_random_access_guaranteed", False))
                )
            )
            row[f"{camera}_rtp_timestamp"] = "" if metadata.get("rtp_timestamp") is None else str(int(metadata["rtp_timestamp"]))
            row[f"{camera}_rtp_ssrc"] = "" if metadata.get("rtp_ssrc") is None else str(int(metadata["rtp_ssrc"]))
            row[f"{camera}_stream_generation"] = (
                ""
                if metadata.get("stream_generation") is None
                else int(metadata["stream_generation"])
            )
            row[f"{camera}_rtcp_frame_ntp_unix_s"] = self.formatting._fmt_record_float(
                None if metadata.get("rtcp_frame_ntp_unix_s") is None else float(metadata["rtcp_frame_ntp_unix_s"]),
                6,
            )
            rtcp_frame_ntp_unix_s = metadata.get("rtcp_frame_ntp_unix_s")
            row[f"{camera}_rtcp_frame_ntp_unix_ns"] = (
                ""
                if rtcp_frame_ntp_unix_s is None
                else self._unix_seconds_to_ns(float(rtcp_frame_ntp_unix_s))
            )
            row[f"{camera}_app_receive_monotonic_ns"] = (
                detection.app_receive_monotonic_ns
            )
            row[f"{camera}_camera_rtc_frame_unix_s"] = row[f"{camera}_rtcp_frame_ntp_unix_s"]
            row[f"{camera}_rtcp_sr_ntp_unix_s"] = self.formatting._fmt_record_float(
                None if metadata.get("rtcp_sr_ntp_unix_s") is None else float(metadata["rtcp_sr_ntp_unix_s"]),
                6,
            )
            row[f"{camera}_camera_rtc_sr_unix_s"] = row[f"{camera}_rtcp_sr_ntp_unix_s"]
            row[f"{camera}_rtcp_sr_rtp_timestamp"] = (
                "" if metadata.get("rtcp_sr_rtp_timestamp") is None else str(int(metadata["rtcp_sr_rtp_timestamp"]))
            )
            row[f"{camera}_rtsp_transport"] = str(metadata.get("rtsp_transport") or "")
            row[f"{camera}_camera_rtc_basis"] = (
                "rtcp-sender-report"
                if metadata.get("rtcp_frame_ntp_unix_s") is not None
                else ntp_timestamp_reference
            )
        display_delta_s = (
            measurement_attempt.right_detection.app_receive_monotonic_s
            - measurement_attempt.left_detection.app_receive_monotonic_s
        )
        left_metadata = measurement_attempt.left_detection.timestamp_metadata
        right_metadata = measurement_attempt.right_detection.timestamp_metadata
        left_source = left_metadata.get("source_timestamp_s")
        right_source = right_metadata.get("source_timestamp_s")
        source_delta_s = (
            float(right_source) - float(left_source)
            if left_source is not None and right_source is not None
            else None
        )
        left_rtcp_ntp = left_metadata.get("rtcp_frame_ntp_unix_s")
        right_rtcp_ntp = right_metadata.get("rtcp_frame_ntp_unix_s")
        rtcp_ntp_delta_s = (
            float(right_rtcp_ntp) - float(left_rtcp_ntp)
            if left_rtcp_ntp is not None and right_rtcp_ntp is not None
            else None
        )
        row["display_frame_delta_s"] = self.formatting._fmt_record_float(display_delta_s, 6)
        row["source_frame_delta_s"] = self.formatting._fmt_record_float(source_delta_s, 6)
        row["rtcp_frame_ntp_delta_s"] = self.formatting._fmt_record_float(rtcp_ntp_delta_s, 6)
        row["camera_rtc_frame_delta_s"] = row["rtcp_frame_ntp_delta_s"]
        return row

    def _record_live_telemetry_snapshot_parameters(
        self, measurement_attempt: MeasurementAttempt
    ) -> dict[str, object]:
        row: dict[str, object] = {
            "attitude_frame": BODY_FRAME,
            "attitude_units": "deg",
            "gps_quality_frame": "gps_telemetry_metadata",
            "gps_quality_units": "satellites_count,eph_m,epv_m",
        }
        for camera in ("left", "right"):
            attitude = (
                measurement_attempt.left_attitude
                if camera == "left"
                else measurement_attempt.right_attitude
            )
            attitude_age_s = (
                measurement_attempt.left_attitude_age_s
                if camera == "left"
                else measurement_attempt.right_attitude_age_s
            )
            row[f"{camera}_roll_deg"] = self.formatting._fmt_record_float(None if attitude is None else attitude.roll_deg, 6)
            row[f"{camera}_pitch_deg"] = self.formatting._fmt_record_float(None if attitude is None else attitude.pitch_deg, 6)
            row[f"{camera}_yaw_deg"] = self.formatting._fmt_record_float(None if attitude is None else attitude.yaw_deg, 6)
            row[f"{camera}_attitude_sample_age_s"] = self.formatting._fmt_record_float(
                attitude_age_s,
                6,
            )
            row[f"{camera}_attitude_alignment"] = (
                "missing" if attitude is None else "nearest_app_receive_monotonic"
            )
            row[f"{camera}_attitude_topic"] = "" if attitude is None else str(attitude.topic)
            gps = measurement_attempt.left_gps if camera == "left" else measurement_attempt.right_gps
            gps_age_s = (
                None
                if gps is None
                else abs(
                    (
                        measurement_attempt.left_detection.app_receive_monotonic_s
                        if camera == "left"
                        else measurement_attempt.right_detection.app_receive_monotonic_s
                    )
                    - gps.app_receive_monotonic_s
                )
            )
            row[f"{camera}_gps_satellites_used"] = (
                "" if gps is None or getattr(gps, "satellites_used", None) is None else str(gps.satellites_used)
            )
            row[f"{camera}_gps_eph_m"] = self.formatting._fmt_record_float(
                None if gps is None else getattr(gps, "eph_m", None),
                6,
            )
            row[f"{camera}_gps_epv_m"] = self.formatting._fmt_record_float(
                None if gps is None else getattr(gps, "epv_m", None),
                6,
            )
            row[f"{camera}_gps_fix_type"] = "" if gps is None or getattr(gps, "fix_type", None) is None else str(gps.fix_type)
            row[f"{camera}_gps_sample_age_s"] = self.formatting._fmt_record_float(
                gps_age_s
                if measurement_attempt.baseline is None
                else (
                    measurement_attempt.baseline.left_age_at_capture_s
                    if camera == "left"
                    else measurement_attempt.baseline.right_age_at_capture_s
                ),
                6,
            )
            row[f"{camera}_gps_alignment"] = (
                "missing" if gps is None else "latest_selected_app_receive_monotonic"
            )
            row[f"{camera}_gps_topic"] = "" if gps is None else str(gps.topic)
        return row
