"""Project session, optical-pose, camera-clock, and native-admission diagnostics.

Persisted field names and schema identities retain their producer contracts."""
from __future__ import annotations
import copy
import json
import time
from collections.abc import Mapping
from math import isfinite
from statistics import median
from Workstation.workstation_core.measurement_attempt import LIVE_DISTANCE_MAX_APP_DELTA_NS, StereoIntrinsicsSnapshot
from Workstation.workstation_core.measurement_methods import CHECKERBOARD_STEREO
from Workstation.workstation_core.coordinate_contract import LEGACY_XYZ_ALIAS_CONTRACT, MEASUREMENT_AXIS_CONTRACT, MEASUREMENT_FRAME, PUBLIC_MEASUREMENT_VECTOR_CONTRACT
from Workstation.workstation_core.runtime_camera_status import is_sr_pc_diagnostic_reason
from Workstation.workstation_core.media_stream.configuration import recording_integrity_contract
from Workstation.workstation_core.sync import ATTITUDE_STALE_AGE_S, ATTITUDE_STALE_PAIR_DELTA_S, ATTITUDE_TIMING_POLICY, ATTITUDE_VERY_STALE_AGE_S, ATTITUDE_VERY_STALE_PAIR_DELTA_S
from .state import (PhotoCaptureState, NativeEvidenceState, RecordingReferenceState, RecordingRowsState, RecordingSessionState, VideoSegmentState)
from .ports import MetadataPort
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .calibration_fields import CalibrationFields
    from .formatting import Formatting
    from .timeline import Timeline


class Metadata:
    def __init__(self, *,
        capture_state: PhotoCaptureState,
        native_state: NativeEvidenceState,
        references: RecordingReferenceState,
        rows: RecordingRowsState,
        session: RecordingSessionState,
        video_state: VideoSegmentState,
        boundary: MetadataPort,
    ) -> None:
        self.capture_state = capture_state
        self.native_state = native_state
        self.references = references
        self.rows = rows
        self.session = session
        self.video_state = video_state
        self.boundary = boundary

    def bind_collaborators(self, *,
        calibration_fields: "CalibrationFields",
        formatting: "Formatting",
        timeline: "Timeline",
    ) -> None:
        self.calibration_fields = calibration_fields
        self.formatting = formatting
        self.timeline = timeline

    def _record_px4_mag_status_rows(self) -> list[dict[str, object]]:
        if not hasattr(self.boundary, "px4_mag_managers"):
            return []
        now_s = time.monotonic()
        rows: list[dict[str, object]] = []
        for camera, label in (("left", "Pixhawk 1"), ("right", "Pixhawk 2")):
            manager = self.boundary.px4_mag_managers.get(camera)
            if manager is None:
                continue
            if not self.boundary._px4_mag_manager_has_data_source(camera):
                continue
            if not self.boundary._sync_px4_mag_manager_store(camera):
                # A raw MAVLink sample does not expose CAL_MAG priority
                # readback; retain live telemetry but do not persist guessed
                # slot state from the local fallback defaults.
                continue
            status = self.boundary._px4_mag_status_cache.get(camera)
            if status is None:
                # PARAM readback is a blocking MAVLink operation and is owned
                # by PixhawkParameterWorker.  Recording must never trigger a
                # hidden readback from the GUI thread.
                continue
            for slot in status.slots:
                rows.append(
                    {
                        "session_id": self.session.recording_session_id,
                        "measurement_id": str(getattr(self.boundary, "_measurement_id", "") or ""),
                        "session_started_unix_s": self.formatting._fmt_record_float(self.session.recording_session_started_wall_s, 6),
                        "recording_elapsed_s": self.formatting._fmt_record_float(self.formatting._recording_elapsed_s(now_s), 6),
                        "recorded_app_timestamp_s": self.formatting._fmt_record_float(now_s, 6),
                        "pixhawk_side": camera,
                        "pixhawk_label": label,
                        "cal_mag_slot": f"CAL_MAG{slot.slot}",
                        "mag_slot": slot.slot,
                        "mag_device_id": "" if slot.device_id is None else str(slot.device_id),
                        "mag_device_type": slot.device_type,
                        "mag_interface": slot.interface,
                        "mag_is_rm3100": self.formatting._fmt_record_bool(slot.is_rm3100),
                        "mag_priority": "" if slot.priority is None else str(slot.priority),
                        "mag_rotation": "" if slot.rotation is None else str(slot.rotation),
                        "mag_enabled": self.formatting._fmt_record_bool(slot.enabled),
                        "mag_expected_selected": self.formatting._fmt_record_bool(slot.expected_selected),
                        "mag_actual_selected": self.formatting._fmt_record_bool(slot.actual_selected),
                        "mag_expected_selected_device_id": (
                            "" if status.expected_selected_device_id is None else str(status.expected_selected_device_id)
                        ),
                        "mag_actual_selected_device_id": (
                            "" if status.actual_selected_device_id is None else str(status.actual_selected_device_id)
                        ),
                        "mag_is_actual_rm3100": self.formatting._fmt_record_bool(status.is_actual_rm3100),
                        "mag_armed": self.formatting._fmt_record_bool(status.armed),
                        "mag_status": slot.status,
                        "mag_warning_text": status.warning_text,
                        "mag_rm3100_primary_id": status.rm3100_primary_id,
                        "mag_rm3100_backup_id": status.rm3100_backup_id,
                        "mag_param_uavcan_enable": status.params.get("UAVCAN_ENABLE"),
                        "mag_param_uavcan_sub_mag": status.params.get("UAVCAN_SUB_MAG"),
                        "mag_param_ekf2_mag_type": status.params.get("EKF2_MAG_TYPE"),
                        "mag_param_snapshot_json": json.dumps(status.params, sort_keys=True),
                    }
                )
        return rows

    def _record_px4_mag_status_snapshot(self, camera: str | None = None) -> None:
        """Persist the current MAG slot/priority selection state in the PX4 timeline."""
        for values in self._record_px4_mag_status_rows():
            if camera is not None and values.get("pixhawk_side") != camera:
                continue
            self.timeline._record_external_timeline_row(
                str(values["pixhawk_side"]),
                "mag_slot_status",
                {
                    "namespace": "",
                    "topic": "px4_mag_manager",
                    "sample_app_timestamp_s": values["recorded_app_timestamp_s"],
                    "sensor_slot": values["cal_mag_slot"],
                    **values,
                },
            )

    def _recording_camera_timing_diagnostics(self) -> dict[str, object]:
        """Summarize the raw camera timing evidence retained in the session spool."""

        def numeric_values(rows: list[dict[str, object]], field: str) -> list[float]:
            values: list[float] = []
            for row in rows:
                value = row.get(field)
                try:
                    parsed = float(value)
                except (TypeError, ValueError):
                    continue
                if isfinite(parsed):
                    values.append(parsed)
            return values

        def summary(values: list[float], *, scale: float = 1.0) -> dict[str, float | None]:
            if not values:
                return {"min": None, "median": None, "max": None}
            scaled = [value * scale for value in values]
            return {
                "min": min(scaled),
                "median": median(scaled),
                "max": max(scaled),
            }

        diagnostics: dict[str, object] = {}
        for camera in ("left", "right"):
            rows = [dict(row) for row in self.rows.camera_recording_rows[camera]]
            valid_rtcp_count = sum(
                1 for row in rows
                if row.get("rtcp_frame_ntp_unix_s") not in (None, "")
            )
            invalid_rtcp_count = sum(
                1 for row in rows
                if row.get("rtcp_timing_invalid_reason") not in (None, "")
                and not is_sr_pc_diagnostic_reason(row.get("rtcp_timing_invalid_reason"))
            )
            sr_pc_diagnostic_count = sum(
                1 for row in rows
                if is_sr_pc_diagnostic_reason(row.get("rtcp_timing_invalid_reason"))
            )
            clock_rate_values = numeric_values(rows, "rtcp_clock_rate_hz")
            media_rate_values = numeric_values(rows, "rtcp_camera_media_rate")
            estimated_media_clock_values: list[float] = []
            for row in rows:
                try:
                    clock_rate = float(row.get("rtcp_clock_rate_hz"))
                    media_rate = float(row.get("rtcp_camera_media_rate"))
                except (TypeError, ValueError):
                    continue
                if isfinite(clock_rate) and isfinite(media_rate):
                    estimated_media_clock_values.append(clock_rate * media_rate)
            rtp_timestamps = numeric_values(rows, "rtp_timestamp")
            diagnostics[camera] = {
                "frame_count": len(rows),
                "rtcp_valid_frame_count": valid_rtcp_count,
                "rtcp_missing_frame_count": max(0, len(rows) - valid_rtcp_count),
                "rtcp_invalid_frame_count": invalid_rtcp_count,
                "rtcp_sr_pc_diagnostic_frame_count": sr_pc_diagnostic_count,
                "rtcp_valid_ratio": (
                    None if not rows else valid_rtcp_count / float(len(rows))
                ),
                "rtcp_receive_delay_ms": summary(
                    numeric_values(rows, "rtcp_frame_receive_delay_s"),
                    scale=1000.0,
                ),
                "rtcp_transport_delay_ms": summary(
                    numeric_values(rows, "rtcp_frame_transport_delay_s"),
                    scale=1000.0,
                ),
                "rtcp_clock_rate_hz": summary(clock_rate_values),
                "rtcp_camera_ntp_rate": summary(numeric_values(rows, "rtcp_camera_ntp_rate")),
                "rtcp_camera_media_rate": summary(media_rate_values),
                "estimated_rtp_clock_rate_hz": summary(estimated_media_clock_values),
                "rtp_timestamp_first": None if not rtp_timestamps else rtp_timestamps[0],
                "rtp_timestamp_last": None if not rtp_timestamps else rtp_timestamps[-1],
            }
        return diagnostics

    def _recording_optical_pose_session_data(self) -> dict[str, object]:
        """Return JSON-safe optical-pose state for session_static.json.

        The live snapshot can contain QImages for the annotation UI.  Session
        metadata stores only frame identities, artifact paths, candidates,
        automatic proposals, and annotation provenance; the lossless pixels
        remain in the session-owned native-frame artifacts.
        """

        evidence_files = {
            "run_events": "run_events.jsonl",
            "native_frames": "native_frames.jsonl",
            "native_frame_root": "frames/optical-pose-burst",
        }
        reader = getattr(self.boundary, "workstation_optical_pose_snapshot", None)
        if not callable(reader):
            return {
                "schema": "workstation-optical-pose-session-v1",
                "state": "UNAVAILABLE",
                "snapshot": {},
                "evidence_files": evidence_files,
                "reason": "optical pose owner snapshot is unavailable",
            }
        try:
            try:
                snapshot = reader(include_frames=False)
            except TypeError:
                # Keep compatibility with small test/legacy owners that do
                # not yet expose the include_frames keyword.
                snapshot = reader()
        except Exception as exc:
            return {
                "schema": "workstation-optical-pose-session-v1",
                "state": "ERROR",
                "snapshot": {},
                "evidence_files": evidence_files,
                "reason": str(exc),
            }
        if not isinstance(snapshot, Mapping):
            return {
                "schema": "workstation-optical-pose-session-v1",
                "state": "ERROR",
                "snapshot": {},
                "evidence_files": evidence_files,
                "reason": "optical pose owner snapshot is not a mapping",
            }
        projection = dict(snapshot)
        # Never allow a UI QImage to cross into recovery/static JSON, even if
        # a legacy owner ignores include_frames=False.
        projection.pop("frozen_frames", None)
        projection.pop("frozen_native_frames", None)
        try:
            safe_projection = json.loads(
                json.dumps(projection, ensure_ascii=False)
            )
        except (TypeError, ValueError) as exc:
            return {
                "schema": "workstation-optical-pose-session-v1",
                "state": "ERROR",
                "snapshot": {},
                "evidence_files": evidence_files,
                "reason": f"snapshot is not JSON-safe: {exc}",
            }
        return {
            "schema": "workstation-optical-pose-session-v1",
            "state": str(snapshot.get("state") or "NO DATA"),
            "snapshot": safe_projection,
            "evidence_files": evidence_files,
        }

    def _recording_session_metadata(
        self,
        *,
        clock_sync_snapshot: dict[str, object] | None = None,
    ) -> dict[str, object]:
        stamp = self.session.recording_session_id or time.strftime("%Y%m%d_%H%M%S")
        clock_sync_snapshot = clock_sync_snapshot or self.boundary._camera_time_sync_snapshot("metadata_read")
        camera_time_metrics = clock_sync_snapshot["metrics"]
        if not isinstance(camera_time_metrics, dict):
            camera_time_metrics = {}
        triangulation_source = self.boundary._triangulation_extrinsics_source()
        try:
            measurement_methods = self.boundary.workstation_measurement_methods_snapshot()
        except (AttributeError, RuntimeError, TypeError, ValueError):
            measurement_methods = {}
        selected_method_row: dict[str, object] = {}
        if isinstance(measurement_methods, dict):
            selected_id = str(measurement_methods.get("selected_method_id") or "")
            selected_instance = str(measurement_methods.get("selected_instance_id") or "")
            method_rows = measurement_methods.get("all_methods", ())
            if not isinstance(method_rows, (tuple, list)) or not method_rows:
                method_rows = measurement_methods.get("ready_methods", ())
            selected_method_row = next(
                (
                    dict(row)
                    for row in method_rows
                    if isinstance(row, Mapping)
                    and str(row.get("method_id") or "") == selected_id
                    and (
                        not selected_instance
                        or str(row.get("instance_id") or "") == selected_instance
                    )
                ),
                {},
            )
        selected_method_id = str(selected_method_row.get("method_id") or "")
        selected_uses_pixhawk = bool(selected_method_row.get("requires_live_pixhawk"))
        selected_uses_gnss = bool(selected_method_row.get("requires_live_gnss"))
        selected_relative_pose = bool(selected_method_id and selected_method_id != "pixhawk_rtk_live")
        if selected_method_id:
            baseline_source = (
                "live_gnss"
                if selected_uses_gnss
                else "checkerboard"
                if selected_method_id == CHECKERBOARD_STEREO
                else "resolved_relative_pose"
            )
        else:
            _baseline_m, baseline_source, _baseline_warnings = self.boundary._active_baseline_m()
        session_owner = getattr(self.boundary, "_measurement_session", None)
        session_snapshotter = getattr(session_owner, "snapshot", None)
        try:
            session_snapshot = session_snapshotter() if callable(session_snapshotter) else {}
        except Exception:
            session_snapshot = {}
        session_snapshot = session_snapshot if isinstance(session_snapshot, Mapping) else {}
        return {
            "session_id": self.session.recording_session_id,
            "session_created_utc_ns": session_snapshot.get("created_utc_ns"),
            "measurement_id": str(getattr(self.boundary, "_measurement_id", "") or ""),
            "measurement_package_id": str(
                getattr(self.boundary, "_measurement_id", "") or ""
            ),
            "recording_mode": str(
                getattr(self.session, "_recording_mode", "manual") or "manual"
            ),
            "session_dir": str(self.boundary._recording_session_dir(stamp)),
            "measurement_dir": str(self.boundary._recording_session_dir(stamp)),
            "session_started_unix_s": self.session.recording_session_started_wall_s,
            "session_started_unix_ns": self.session.recording_session_started_unix_ns,
            "session_started_monotonic_ns": self.session.recording_session_started_monotonic_ns,
            "session_time_anchor_uncertainty_ns": self.session.recording_session_anchor_uncertainty_ns,
            "measurement_started_unix_s": self.session.recording_session_started_wall_s,
            "measurement_started_unix_ns": self.session.recording_session_started_unix_ns,
            "measurement_started_monotonic_ns": self.session.recording_session_started_monotonic_ns,
            "measurement_time_anchor_uncertainty_ns": self.session.recording_session_anchor_uncertainty_ns,
            "schema_version": "stereowiris-paired-frame-v5",
            "row_contract": "one row per terminal left/right matched detection outcome",
            "terminal_record_id_contract": (
                "attempt:<measurement_attempt_sha256> or "
                "blocked:<blocked_pair_sha256>"
            ),
            "left_video_path": str(self.video_state.video_recording_paths.get("left", "")),
            "right_video_path": str(self.video_state.video_recording_paths.get("right", "")),
            "matched_frame_pair_count": len(self.rows.distance_recording_rows),
            "native_completion_persistence_pending_count": len(
                self.native_state._pending_native_completions
            ),
            "native_completion_conflict_pending_count": len(
                self.native_state._pending_native_conflicts
            ),
            "native_completion_terminal_drop_count": (
                self.native_state._native_completion_terminal_drop_count
            ),
            "native_completion_terminal_drop_evictions": (
                self.native_state._native_completion_terminal_drop_evictions
            ),
            "native_completion_terminal_drops": [
                copy.deepcopy(record)
                for record in self.native_state._native_completion_terminal_drops
            ],
            "camera_left_gui_frame_count": len(self.rows.camera_recording_rows["left"]),
            "camera_right_gui_frame_count": len(self.rows.camera_recording_rows["right"]),
            "px4_left_sample_count": len(self.rows.px4_recording_rows["left"]),
            "px4_right_sample_count": len(self.rows.px4_recording_rows["right"]),
            "px4_drone_sample_count": len(self.rows.px4_recording_rows.get("drone", ())),
            "gnss_base_sample_count": len(self.rows.gnss_base_recording_rows),
            "extrinsics_comparison_sample_count": len(self.rows.extrinsics_comparison_recording_rows),
            "method_evaluation_bundle_count": len(
                self.rows.method_evaluation_bundle_rows
            ),
            "device_timeline_contract": {
                "camera_row": "one row per frame delivered to the GUI frame_changed callback",
                "camera_not_guaranteed": "not a one-to-one index of encoded MP4 frames",
                "mp4_frame_index_contract": "not_available",
                "px4_row": "one row per PX4 telemetry callback received by the GUI",
                "sample_monotonic": "producer/app receive CLOCK_MONOTONIC timestamp",
                "session_utc": "derived from session host Unix/monotonic anchor",
                "source_utc": "RTCP NTP or GNSS UTC only when explicitly valid",
                "mavlink_provenance": (
                    "endpoint, link-open/heartbeat, link target system/component, and received packet "
                    "source system/component/sequence/message id are recorded when available"
                ),
            },
            "camera_time_sync_status": camera_time_metrics["status"],
            "camera_time_sync_basis": camera_time_metrics["basis"],
            "camera_time_sync_warning": camera_time_metrics["warning"],
            "stream_sync_state": self.boundary.camera_sync_status,
            "stream_sync_reason": self.boundary.camera_sync_reason,
            "camera_time_sync_observation": clock_sync_snapshot,
            "camera_time_sync_observations": {
                "start": self.session._recording_clock_sync_start_observation,
                "end": self.session._recording_clock_sync_end_observation,
            },
            "camera_time_sync_drift": self.boundary._camera_time_sync_drift_metadata(),
            "pnp_hr_photo_capture": {
                "storage": "camera_internal",
                "ftp_downloaded": False,
                "event_file": "hr_captures.jsonl",
                "capture_command": "SIHR TRUE + CPTR + ICPT",
                "captures": list(getattr(self.capture_state, "pnp_photo_capture_records", ())),
            },
            "oz_photo_capture": {
                "storage": "camera_internal",
                "ftp_downloaded": False,
                "event_file": "oz_captures.jsonl",
                "capture_command": "SIVI TRUE + CPTR + ICPT",
                "captures": list(getattr(self.capture_state, "oz_photo_capture_records", ())),
            },
            "camera_timing_diagnostics": self._recording_camera_timing_diagnostics(),
            "pc_utc_sync": self.boundary._last_wiris_pc_utc_sync,
            "gnss_operation": self.boundary.gnss_operation_manager.metadata(),
            "gnss_operation_config": self.boundary.gnss_operation_manager.config.to_mapping(),
            "gnss_fixed_base_profile": (
                None
                if self.boundary.gnss_fixed_base_profile is None
                else self.boundary.gnss_fixed_base_profile.to_mapping()
            ),
            "rtcp_pair_delta_s": camera_time_metrics["pair_delta_s"],
            "rtcp_pc_offset_max_ms": camera_time_metrics["max_pc_offset_ms"],
            "rtcp_frame_transit_max_ms": camera_time_metrics["max_frame_transit_delay_ms"],
            "left_rtcp_frame_transit_ms": camera_time_metrics["left_frame_transit_delay_ms"],
            "right_rtcp_frame_transit_ms": camera_time_metrics["right_frame_transit_delay_ms"],
            "calculation": {
                "triangulation_extrinsics_source": triangulation_source,
                "measurement_method_id": str(
                    measurement_methods.get("selected_method_id", "")
                    if isinstance(measurement_methods, dict)
                    else ""
                ),
                "measurement_method_instance_id": str(
                    measurement_methods.get("selected_instance_id", "")
                    if isinstance(measurement_methods, dict)
                    else ""
                ),
                "measurement_method_selection_reason": str(
                    measurement_methods.get("selection_reason", "")
                    if isinstance(measurement_methods, dict)
                    else ""
                ),
                "measurement_method_fallback_order": (
                    list(measurement_methods.get("fallback_order", ()))
                    if isinstance(measurement_methods, dict)
                    else []
                ),
                "measurement_method_fill_policy": (
                    dict(measurement_methods.get("fill_policy", {}))
                    if isinstance(measurement_methods, dict)
                    else {}
                ),
                "distance_mode": self.boundary._distance_mode_label(),
                "baseline_source": baseline_source,
                "measurement_path": self.boundary._measurement_path_label(),
                "method": "two_ray_closest_midpoint",
                "measurement_frame": MEASUREMENT_FRAME,
                "measurement_axis_contract": MEASUREMENT_AXIS_CONTRACT,
                "public_vector_contract": PUBLIC_MEASUREMENT_VECTOR_CONTRACT,
                "legacy_xyz_alias_contract": LEGACY_XYZ_ALIAS_CONTRACT,
                "baseline_frame_scope": "measurement_attempt",
                "baseline_frame_origin_contract": (
                    "midpoint_of_left_optical_origin_and_selected_right_origin_proxy"
                ),
                "baseline_frame_origin_model": (
                    "resolved_relative_pose_optical_centers"
                    if selected_relative_pose
                    else "gps_rtk_antenna_baseline_proxy_no_lever_arm"
                ),
                "baseline_frame_axis_contract": (
                    "right=left_to_right_center; forward=projected_average_optical_forward; "
                    "down=forward_cross_right"
                ),
                "attitude_timing_policy": ATTITUDE_TIMING_POLICY,
                "attitude_timing_blocks_measurement": selected_uses_pixhawk,
                "attitude_stale_age_threshold_s": ATTITUDE_STALE_AGE_S,
                "attitude_very_stale_age_threshold_s": ATTITUDE_VERY_STALE_AGE_S,
                "attitude_stale_pair_delta_threshold_s": ATTITUDE_STALE_PAIR_DELTA_S,
                "attitude_very_stale_pair_delta_threshold_s": ATTITUDE_VERY_STALE_PAIR_DELTA_S,
                "mount_assumption_model": (
                    "checkerboard-stereo-extrinsics-v1"
                    if selected_method_id == CHECKERBOARD_STEREO
                    else "hybrid-candidate-live-components-v1"
                    if selected_relative_pose and (selected_uses_pixhawk or selected_uses_gnss)
                    else "optical-pose-relative-r-t-v1"
                    if selected_relative_pose
                    else "gps-rtk-baseline-only-v1"
                ),
            },
            "checkerboard_stereo_extrinsics": self.calibration_fields._record_current_checkerboard_extrinsics_parameters(),
            "timestamp_contract": {
                "manufacturer_confirmed": False,
                "distance_pairing_time_basis_requested": str(
                    self.boundary._yolo_time_basis()
                ),
                "distance_pairing_policy": "settings_preferred_with_app_receive_fallback",
                "distance_pairing_time_role": "per_pair_effective_authoritative_distance",
                "distance_pairing_max_abs_delta_ns": LIVE_DISTANCE_MAX_APP_DELTA_NS,
                "camera_captured_timestamp_clock": "WIRIS RTC checked by STUS/GDTI",
                "rtp_rtcp_timestamp_clock": "RTP relative media counter; RTCP Sender Report NTP-derived stream time",
                "rtp_timestamp_note": "RTP timestamp is a 90 kHz media clock counter; mapping RTCP NTP to WIRIS GDTI/UTC is unverified.",
                "app_receive_fallback_field": "app_receive_monotonic_ns",
                "ntp_evidence_role": "selected_only_when_requested_verified_current_same_epoch",
                "media_integrity_mode": "tolerant_decoded",
                "media_random_access_contract": (
                    "diagnostic_provenance_only; decoded Measurement admitted"
                ),
                "legacy_equivalent_field": "camera_rtc_frame_unix_s",
                "clock_offset_check": "Use the 시간 tab RTCP/NTP check plus direct WIRIS GDTI validation before absolute UTC claims.",
            },
            "intrinsics": self.calibration_fields._record_current_intrinsic_parameters(),
            "dual_geometry": {
                "state": "nominal_active_enabled",
                "nominal_profile_sha256": (
                    ""
                    if not isinstance(
                        getattr(self.references, "recording_nominal_intrinsics_snapshot", None),
                        StereoIntrinsicsSnapshot,
                    )
                    else self.references.recording_nominal_intrinsics_snapshot.content_sha256
                ),
                "nominal_intrinsics_profile_sha256": (
                    ""
                    if not isinstance(
                        getattr(self.references, "recording_nominal_intrinsics_snapshot", None),
                        StereoIntrinsicsSnapshot,
                    )
                    else self.references.recording_nominal_intrinsics_snapshot.intrinsics_content_sha256
                ),
                "nominal_optical_pose_profile_hash": str(
                    getattr(self.references, "recording_nominal_optical_pose_profile_hash", "") or ""
                ),
                "active_selection_id": str(
                    getattr(self.boundary, "_active_optical_pose_selection_id", "") or ""
                ),
                "selection_effective_from_pair_sequence": getattr(
                    self.boundary, "_active_optical_pose_effective_from_pair_sequence", None
                ),
                "same_detection_evaluated_twice": True,
                "coordinate_contract": "both evaluations use OpenCV optical RDF and baseline-center FRD output",
            },
            "optical_pose": self._recording_optical_pose_session_data(),
            "stream": {
                "left_rtsp_url": self.boundary.left_url_edit.text().strip(),
                "right_rtsp_url": self.boundary.right_url_edit.text().strip(),
                "backend": self.boundary.stream_backend_combo.currentData(),
                "transport": self.boundary.rtsp_transport_combo.currentData(),
                "decoder": self.boundary.decoder_combo.currentData(),
                "rtsp_latency_ms": self.boundary.rtsp_latency_spin.value(),
                **recording_integrity_contract(
                    self.boundary.stream_backend_combo.currentData()
                ),
            },
        }
