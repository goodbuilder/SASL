"""Record raw camera/PX4 callbacks and the separate GNSS-center reference stream.

Host receive, source UTC, and session monotonic time retain independent columns;
GNSS local-level FRD never aliases optical baseline-center FRD."""
from __future__ import annotations
import math
import time
from collections.abc import Mapping
from PyQt5 import QtGui
from Workstation.workstation_core.gnss_baseline_center import DEFAULT_MAXIMUM_GNSS_AGE_S, DEFAULT_MAXIMUM_GNSS_TIMESTAMP_SPREAD_S, build_gnss_baseline_center_record, estimate_gnss_baseline_center_frd
from Workstation.workstation_core.rtk_geometry import GpsPositionState
from .contracts import (_normalize_external_timeline_values)
from .state import (RecordingRowsState, RecordingSessionState)
from .ports import TimelinePort
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .formatting import Formatting
    from .presentation import Presentation
    from .row_collections import RowCollections
    from .timing_fields import TimingFields


class Timeline:
    def __init__(self, *,
        rows: RecordingRowsState,
        session: RecordingSessionState,
        boundary: TimelinePort,
    ) -> None:
        self.rows = rows
        self.session = session
        self.boundary = boundary

    def bind_collaborators(self, *,
        formatting: "Formatting",
        presentation: "Presentation",
        row_collections: "RowCollections",
        timing_fields: "TimingFields",
    ) -> None:
        self.formatting = formatting
        self.presentation = presentation
        self.row_collections = row_collections
        self.timing_fields = timing_fields

    def _raw_timeline_row(
        self,
        *,
        side: str,
        sample_type: str,
        row_contract: str,
        sample_monotonic_s: float,
        host_receive_monotonic_ns: int,
        host_receive_utc_ns: int,
        source_utc_ns: int | None,
        source_utc_method: str,
        sequence: int,
    ) -> dict[str, object] | None:
        sample_monotonic_ns = int(round(float(sample_monotonic_s) * 1_000_000_000.0))
        started_monotonic_ns = self.session.recording_session_started_monotonic_ns
        if started_monotonic_ns is not None and sample_monotonic_ns < started_monotonic_ns:
            return None
        session_elapsed_s = (
            None
            if started_monotonic_ns is None
            else (sample_monotonic_ns - started_monotonic_ns) * 1e-9
        )
        session_utc_ns = self.formatting._session_utc_ns_for_monotonic_ns(sample_monotonic_ns)
        return {
            "sequence": sequence,
            "session_id": self.session.recording_session_id,
            "measurement_id": str(getattr(self.boundary, "_measurement_id", "") or ""),
            "side": side,
            "sample_type": sample_type,
            "row_contract": row_contract,
            "sample_monotonic_ns": sample_monotonic_ns,
            "sample_monotonic_s": self.formatting._fmt_record_float(sample_monotonic_s, 9),
            "session_elapsed_s": self.formatting._fmt_record_float(session_elapsed_s, 9),
            "session_utc_ns": "" if session_utc_ns is None else session_utc_ns,
            "session_utc_s": self.formatting._fmt_record_float(
                None if session_utc_ns is None else session_utc_ns * 1e-9,
                9,
            ),
            "session_utc_method": (
                "unavailable" if session_utc_ns is None else "host_unix_monotonic_session_anchor"
            ),
            "session_utc_anchor_uncertainty_ns": (
                ""
                if self.session.recording_session_anchor_uncertainty_ns is None
                else self.session.recording_session_anchor_uncertainty_ns
            ),
            "source_utc_ns": "" if source_utc_ns is None else source_utc_ns,
            "source_utc_s": self.formatting._fmt_record_float(
                None if source_utc_ns is None else source_utc_ns * 1e-9,
                9,
            ),
            "source_utc_valid": self.formatting._fmt_record_bool(source_utc_ns is not None),
            "source_utc_method": source_utc_method if source_utc_ns is not None else "unavailable",
            "host_receive_monotonic_ns": host_receive_monotonic_ns,
            "host_receive_utc_ns": host_receive_utc_ns,
        }

    def _record_gui_camera_frame(
        self,
        camera: str,
        image: QtGui.QImage,
        metadata: dict[str, object],
    ) -> None:
        if not self.session.distance_recording_enabled or camera not in self.rows.camera_recording_rows:
            return
        host_receive_monotonic_ns = time.monotonic_ns()
        host_receive_utc_ns = time.time_ns()
        sample_monotonic_s = float(
            metadata.get("timestamp_s") or host_receive_monotonic_ns * 1e-9
        )
        rtcp_ntp_s = metadata.get("rtcp_frame_ntp_unix_s")
        source_utc_ns = (
            None
            if rtcp_ntp_s is None
            else self.timing_fields._unix_seconds_to_ns(float(rtcp_ntp_s))
        )
        rows = self.rows.camera_recording_rows[camera]
        row = self._raw_timeline_row(
            side=camera,
            sample_type="gui_camera_frame",
            row_contract="one row per frame delivered to the GUI frame_changed callback",
            sample_monotonic_s=sample_monotonic_s,
            host_receive_monotonic_ns=host_receive_monotonic_ns,
            host_receive_utc_ns=host_receive_utc_ns,
            source_utc_ns=source_utc_ns,
            source_utc_method="rtcp_ntp",
            sequence=len(rows),
        )
        if row is None:
            return
        row.update(
            {
                "frame_stage": "gui_delivered_preview",
                "mp4_frame_index_contract": "not_available",
                "app_receive_monotonic_ns": (
                    ""
                    if metadata.get("app_receive_monotonic_ns") is None
                    else int(metadata["app_receive_monotonic_ns"])
                ),
                "app_receive_timestamp_role": str(
                    metadata.get("app_receive_timestamp_role")
                    or "legacy_float_derived"
                ),
                "ntp_evidence_role": "record_only",
                "media_recovery_basis": str(
                    metadata.get("media_recovery_basis") or ""
                ),
                "media_integrity_mode": str(
                    metadata.get("media_integrity_mode") or ""
                ),
                "media_integrity_unverified": self.formatting._fmt_record_bool(
                    bool(metadata.get("media_integrity_unverified", False))
                ),
                "source_media_integrity_verified": self.formatting._fmt_record_bool(
                    bool(metadata.get("source_media_integrity_verified", False))
                ),
                "media_random_access_guaranteed": self.formatting._fmt_record_bool(
                    bool(metadata.get("media_random_access_guaranteed", False))
                ),
                "image_width_px": image.width(),
                "image_height_px": image.height(),
                "image_axis_contract": "opencv_pixels_origin_top_left_u_right_v_down",
                "timestamp_source": str(metadata.get("timestamp_source") or ""),
                "timestamp_reference": str(metadata.get("timestamp_reference") or ""),
                "source_timestamp_s": self.formatting._fmt_record_float(
                    None
                    if metadata.get("source_timestamp_s") is None
                    else float(metadata["source_timestamp_s"]),
                    9,
                ),
                "rtp_timestamp": (
                    "" if metadata.get("rtp_timestamp") is None else str(int(metadata["rtp_timestamp"]))
                ),
                "rtp_ssrc": (
                    "" if metadata.get("rtp_ssrc") is None else str(int(metadata["rtp_ssrc"]))
                ),
                "rtp_marker_seq": (
                    "" if metadata.get("rtp_marker_seq") is None else str(int(metadata["rtp_marker_seq"]))
                ),
                "stream_generation": (
                    ""
                    if metadata.get("stream_generation") is None
                    else int(metadata["stream_generation"])
                ),
                "rtcp_sr_ntp_raw": (
                    ""
                    if metadata.get("rtcp_sr_ntp_raw") is None
                    else int(metadata["rtcp_sr_ntp_raw"])
                ),
                "rtcp_sr_ntp_unix_s": self.formatting._fmt_record_float(
                    None
                    if metadata.get("rtcp_sr_ntp_unix_s") is None
                    else float(metadata["rtcp_sr_ntp_unix_s"]),
                    9,
                ),
                "rtcp_sr_received_count": (
                    ""
                    if metadata.get("rtcp_sr_received_count") is None
                    else str(int(metadata["rtcp_sr_received_count"]))
                ),
                "rtcp_sr_rtp_timestamp": (
                    ""
                    if metadata.get("rtcp_sr_rtp_timestamp") is None
                    else str(int(metadata["rtcp_sr_rtp_timestamp"]))
                ),
                "rtcp_clock_rate_hz": self.formatting._fmt_record_float(
                    None
                    if metadata.get("rtcp_clock_rate_hz") is None
                    else float(metadata["rtcp_clock_rate_hz"]),
                    3,
                ),
                "rtcp_sr_age_s": self.formatting._fmt_record_float(
                    None if metadata.get("rtcp_sr_age_s") is None else float(metadata["rtcp_sr_age_s"]),
                    9,
                ),
                "rtcp_sr_pc_offset_s": self.formatting._fmt_record_float(
                    None
                    if metadata.get("rtcp_sr_pc_offset_s") is None
                    else float(metadata["rtcp_sr_pc_offset_s"]),
                    9,
                ),
                "rtcp_frame_ntp_unix_s": self.formatting._fmt_record_float(
                    None if rtcp_ntp_s is None else float(rtcp_ntp_s),
                    9,
                ),
                "rtcp_frame_received_wall_s": self.formatting._fmt_record_float(
                    None
                    if metadata.get("rtcp_frame_received_wall_s") is None
                    else float(metadata["rtcp_frame_received_wall_s"]),
                    9,
                ),
                "rtcp_frame_receive_delay_s": self.formatting._fmt_record_float(
                    None
                    if metadata.get("rtcp_frame_receive_delay_s") is None
                    else float(metadata["rtcp_frame_receive_delay_s"]),
                    9,
                ),
                "rtcp_frame_transport_delay_s": self.formatting._fmt_record_float(
                    None
                    if metadata.get("rtcp_frame_transport_delay_s") is None
                    else float(metadata["rtcp_frame_transport_delay_s"]),
                    9,
                ),
                "rtcp_camera_ntp_rate": self.formatting._fmt_record_float(
                    None
                    if metadata.get("rtcp_camera_ntp_rate") is None
                    else float(metadata["rtcp_camera_ntp_rate"]),
                    9,
                ),
                "rtcp_camera_media_rate": self.formatting._fmt_record_float(
                    None
                    if metadata.get("rtcp_camera_media_rate") is None
                    else float(metadata["rtcp_camera_media_rate"]),
                    9,
                ),
                "rtsp_transport": str(metadata.get("rtsp_transport") or ""),
                "rtcp_timing_invalid_reason": str(
                    metadata.get("rtcp_timing_invalid_reason") or ""
                ),
            }
        )
        rows.append(row)

    def _record_external_timeline_row(self, camera: str, sample_type: str, values: dict[str, object]) -> None:
        if not self.session.distance_recording_enabled:
            return
        side = self.formatting._recording_side(camera)
        if side is None:
            return
        normalized_values, nonfinite_reasons = _normalize_external_timeline_values(
            values
        )
        if side not in self.rows.px4_recording_rows:
            rows = self.row_collections._new_live_rows(f"px4.{side}")
            self.rows.px4_recording_rows[side] = rows
        host_receive_monotonic_ns = time.monotonic_ns()
        host_receive_utc_ns = time.time_ns()
        raw_sample_monotonic_s = normalized_values.get("sample_app_timestamp_s")
        sample_timestamp_source = "sample_app_timestamp_s"
        sample_timestamp_fallback_reason = ""
        if raw_sample_monotonic_s in (None, ""):
            sample_monotonic_s = host_receive_monotonic_ns * 1e-9
            sample_timestamp_source = "host_receive_monotonic_ns"
            reason = nonfinite_reasons.get("sample_app_timestamp_s")
            sample_timestamp_fallback_reason = (
                f"sample_app_timestamp_s is non-finite: {reason}"
                if reason is not None
                else "sample_app_timestamp_s unavailable"
            )
        else:
            try:
                sample_monotonic_s = float(raw_sample_monotonic_s)
            except (TypeError, ValueError, OverflowError):
                sample_monotonic_s = host_receive_monotonic_ns * 1e-9
                sample_timestamp_source = "host_receive_monotonic_ns"
                sample_timestamp_fallback_reason = (
                    "sample_app_timestamp_s is not numeric"
                )
            if not math.isfinite(sample_monotonic_s):
                # The recursive normalizer owns floats, but keep this guard for
                # numeric wrappers that expose float(...) without subclassing float.
                sample_monotonic_s = host_receive_monotonic_ns * 1e-9
                sample_timestamp_source = "host_receive_monotonic_ns"
                sample_timestamp_fallback_reason = (
                    "sample_app_timestamp_s converted to a non-finite number"
                )
        raw_gps_utc_us = normalized_values.get("gps_utc_us")
        if raw_gps_utc_us in (None, "", 0, "0"):
            source_utc_ns = None
        else:
            try:
                source_utc_ns = int(raw_gps_utc_us) * 1000
            except (TypeError, ValueError, OverflowError):
                source_utc_ns = None
        rows = self.rows.px4_recording_rows[side]
        row = self._raw_timeline_row(
            side=side,
            sample_type=sample_type,
            row_contract="one row per PX4 telemetry callback received by the GUI",
            sample_monotonic_s=sample_monotonic_s,
            host_receive_monotonic_ns=host_receive_monotonic_ns,
            host_receive_utc_ns=host_receive_utc_ns,
            source_utc_ns=source_utc_ns,
            source_utc_method=str(
                normalized_values.get("source_utc_method") or "gps_utc"
            ),
            sequence=len(rows),
        )
        if row is None:
            return
        row.update(
            {
                "camera": camera,
                "triangulation_extrinsics_source": self.boundary._triangulation_extrinsics_source(),
                "distance_mode": self.boundary._distance_mode_label(),
            }
        )
        row.update(self.boundary.mavlink_link_provenance(side))
        row.update(normalized_values)
        row["sample_timestamp_source"] = sample_timestamp_source
        row["sample_timestamp_fallback_reason"] = sample_timestamp_fallback_reason
        if nonfinite_reasons:
            row["nonfinite_source_fields"] = sorted(nonfinite_reasons)
            row["nonfinite_source_reasons"] = {
                name: nonfinite_reasons[name] for name in sorted(nonfinite_reasons)
            }
        rows.append(row)
        self.rows.external_parameter_recording_rows.append(row)
        self.presentation._refresh_recording_status_label()

    def _record_gnss_baseline_center(
        self,
        camera: str,
        drone: GpsPositionState,
    ) -> bool:
        """Attempt one derived row for each primary Drone GNSS callback."""

        if str(camera) != "drone" or not bool(self.session.distance_recording_enabled):
            return False
        selected_reader = getattr(self.boundary, "_selected_rover_gps_state", None)
        latest = getattr(self.boundary, "latest_gps", {}) or {}
        p1 = (
            selected_reader("left")
            if callable(selected_reader)
            else latest.get("left")
            if isinstance(latest, Mapping)
            else None
        )
        p2 = (
            selected_reader("right")
            if callable(selected_reader)
            else latest.get("right")
            if isinstance(latest, Mapping)
            else None
        )
        if not isinstance(p1, GpsPositionState):
            p1 = None
        if not isinstance(p2, GpsPositionState):
            p2 = None
        now_s = time.monotonic()
        estimate = estimate_gnss_baseline_center_frd(
            p1,
            p2,
            drone,
            now_s=now_s,
            maximum_age_s=DEFAULT_MAXIMUM_GNSS_AGE_S,
            maximum_timestamp_spread_s=(
                DEFAULT_MAXIMUM_GNSS_TIMESTAMP_SPREAD_S
            ),
        )
        selection_reader = getattr(self.boundary, "_pixhawk_gnss_source", None)
        p1_selection = (
            str(selection_reader("left") or "GPS1")
            if callable(selection_reader)
            else "GPS1"
        )
        p2_selection = (
            str(selection_reader("right") or "GPS1")
            if callable(selection_reader)
            else "GPS1"
        )
        sequence = max(
            0,
            int(getattr(self.rows, "_gnss_baseline_center_recording_sequence", 0)),
        )
        host_receive_monotonic_ns = time.monotonic_ns()
        host_receive_utc_ns = time.time_ns()
        source_utc_ns = (
            None if drone.gps_utc_us is None else int(drone.gps_utc_us) * 1000
        )
        row = self._raw_timeline_row(
            side="drone",
            sample_type="gnss_baseline_center_frd",
            row_contract="one row per primary Drone GNSS callback while recording",
            sample_monotonic_s=float(drone.timestamp_s),
            host_receive_monotonic_ns=host_receive_monotonic_ns,
            host_receive_utc_ns=host_receive_utc_ns,
            source_utc_ns=source_utc_ns,
            source_utc_method=str(drone.time_source or "drone_gps_utc"),
            sequence=sequence,
        )
        if row is None:
            return False
        row.update(
            build_gnss_baseline_center_record(
                estimate,
                p1=p1,
                p2=p2,
                drone=drone,
                p1_selection=p1_selection,
                p2_selection=p2_selection,
                sequence=sequence,
            )
        )
        self.rows._gnss_baseline_center_recording_sequence = sequence + 1
        writer = getattr(self.boundary, "append_workstation_data", None)
        if not callable(writer):
            return False
        try:
            return bool(writer("gnss.baseline.center", row, priority=False))
        except (OSError, RuntimeError, TypeError, ValueError):
            return False
