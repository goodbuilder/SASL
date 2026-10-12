"""Project frozen attempts and blocked outcomes into canonical recording rows.

Measurement-center FRD, left-camera RDF, and GNSS NED fields remain distinct;
primary admission and native-pixel scheduling are explicit collaborators."""
from __future__ import annotations
import hashlib
import json
from collections.abc import Mapping
from Workstation.workstation_core.measurement_attempt import BlockedMeasurementPair, LIVE_DISTANCE_MAX_APP_DELTA_NS, MeasurementAttempt, StereoIntrinsicsSnapshot, live_pair_timing_evidence
from Workstation.workstation_core.dual_geometry import evaluate_nominal_active
from Workstation.workstation_core.coordinate_contract import LEGACY_XYZ_ALIAS_CONTRACT, MEASUREMENT_AXIS_CONTRACT, MEASUREMENT_FRAME, PUBLIC_MEASUREMENT_VECTOR_CONTRACT
from Workstation.workstation_core.target_coordinates import target_coordinates_from_attempt
from Workstation.workstation_core.measurement_data_store import ROW_WRITE_ADMITTED, ROW_WRITE_RETRYABLE, ROW_WRITE_UNAVAILABLE, RowWriteAdmission
from .contracts import (RECORDING_KEY_CACHE_LIMIT)
from .state import (PrimaryPersistenceLedger, RecordingReferenceState, RecordingRowsState, RecordingSessionState, VideoSegmentState)
from .ports import MeasurementRowsPort
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .calibration_fields import CalibrationFields
    from .formatting import Formatting
    from .geometry_fields import GeometryFields
    from .native_completion import NativeCompletion
    from .native_submission import NativeSubmission
    from .presentation import Presentation
    from .primary_persistence import PrimaryPersistence
    from .timing_fields import TimingFields


class MeasurementRows:
    def __init__(self, *,
        ledger: PrimaryPersistenceLedger,
        references: RecordingReferenceState,
        rows: RecordingRowsState,
        session: RecordingSessionState,
        video_state: VideoSegmentState,
        boundary: MeasurementRowsPort,
    ) -> None:
        self.ledger = ledger
        self.references = references
        self.rows = rows
        self.session = session
        self.video_state = video_state
        self.boundary = boundary

    def bind_collaborators(self, *,
        calibration_fields: "CalibrationFields",
        formatting: "Formatting",
        geometry_fields: "GeometryFields",
        native_completion: "NativeCompletion",
        native_submission: "NativeSubmission",
        presentation: "Presentation",
        primary_persistence: "PrimaryPersistence",
        timing_fields: "TimingFields",
    ) -> None:
        self.calibration_fields = calibration_fields
        self.formatting = formatting
        self.geometry_fields = geometry_fields
        self.native_completion = native_completion
        self.native_submission = native_submission
        self.presentation = presentation
        self.primary_persistence = primary_persistence
        self.timing_fields = timing_fields
        self.blocked_timing_writer = timing_fields._record_measurement_sync_timestamp_parameters

    def _pair_recording_base_row(self, measurement_attempt: MeasurementAttempt) -> dict[str, object]:
        pair_mid_monotonic_s = measurement_attempt.detection_midpoint_monotonic_s
        pair_mid_monotonic_ns = int(round(pair_mid_monotonic_s * 1_000_000_000.0))
        session_elapsed_ns = (
            None
            if self.session.recording_session_started_monotonic_ns is None
            else max(0, pair_mid_monotonic_ns - self.session.recording_session_started_monotonic_ns)
        )
        row: dict[str, object] = {
            "session_id": self.session.recording_session_id,
            "measurement_id": str(getattr(self.boundary, "_measurement_id", "") or ""),
            "measurement_package_id": str(
                getattr(self.boundary, "_measurement_id", "") or ""
            ),
            "terminal_record_id": measurement_attempt.terminal_record_id,
            "measurement_attempt_id": str(measurement_attempt.attempt_id),
            "measurement_attempt_session_id": measurement_attempt.attempt_session_id,
            "measurement_attempt_sha256": measurement_attempt.canonical_sha256(),
            "measurement_attempt_canonical_json": measurement_attempt.canonical_json(),
            "measurement_detection_pair_key": json.dumps(
                measurement_attempt.detection_pair_key, separators=(",", ":")
            ),
            "schema_version": "stereowiris-paired-frame-v5",
            "measurement_attempt_content_sha256": measurement_attempt.content_sha256(),
            "measurement_attempt_instance_sha256": measurement_attempt.instance_sha256(),
            "measurement_attempt_timestamp_basis": measurement_attempt.timestamp_basis,
            "measurement_attempt_source_quality": measurement_attempt.source_quality,
            "measurement_attempt_warnings": "; ".join(measurement_attempt.warnings),
            "measurement_frame": MEASUREMENT_FRAME,
            "measurement_axis_contract": MEASUREMENT_AXIS_CONTRACT,
            "public_vector_contract": PUBLIC_MEASUREMENT_VECTOR_CONTRACT,
            "legacy_xyz_alias_contract": LEGACY_XYZ_ALIAS_CONTRACT,
            "baseline_frame_scope": "measurement_attempt",
            "baseline_frame_instance_id": measurement_attempt.instance_sha256(),
            "baseline_frame_origin_contract": (
                "midpoint_of_left_optical_origin_and_selected_right_origin_proxy"
            ),
                "baseline_frame_origin_model": (
                    "resolved_relative_pose_optical_centers"
                    if measurement_attempt.measurement_geometry_kind == "relative_pose"
                    else "gps_rtk_antenna_baseline_proxy_no_lever_arm"
                ),
            "baseline_frame_axis_construction": (
                "right=left_to_right_center;forward=projected_average_optical_forward;"
                "down=forward_cross_right"
            ),
            "measurement_valid": "false",
            "pair_status": "matched_geometry_pending",
            "session_started_unix_ns": self.session.recording_session_started_unix_ns,
            "session_started_monotonic_ns": self.session.recording_session_started_monotonic_ns,
            "recorded_unix_ns": int(round(measurement_attempt.created_wall_s * 1_000_000_000.0)),
            "pair_app_mid_monotonic_ns": pair_mid_monotonic_ns,
            "session_elapsed_ns": session_elapsed_ns,
            "session_elapsed_s": (
                "" if session_elapsed_ns is None else session_elapsed_ns * 1e-9
            ),
            "left_video_path": str(self.video_state.video_recording_paths.get("left", "")),
            "right_video_path": str(self.video_state.video_recording_paths.get("right", "")),
            "selected_left_pairing_timestamp_s": self.formatting._fmt_record_float(
                measurement_attempt.left_detection.pairing_timestamp_s, 9
            ),
            "selected_right_pairing_timestamp_s": self.formatting._fmt_record_float(
                measurement_attempt.right_detection.pairing_timestamp_s, 9
            ),
        }
        row.update(self.native_submission._persist_native_measurement_pair(measurement_attempt))
        row.update(self.timing_fields._record_bbox_parameters(measurement_attempt))
        row.update(self.timing_fields._record_measurement_sync_timestamp_parameters(measurement_attempt))
        row.update(self.calibration_fields._record_intrinsic_parameters(measurement_attempt))
        row.update(self.timing_fields._record_frame_timestamp_parameters(measurement_attempt))
        row.update(self.timing_fields._record_live_telemetry_snapshot_parameters(measurement_attempt))
        row.update(self.boundary._measurement_gnss_record_metadata())
        row.update(self.geometry_fields._record_external_parameters(estimate=None, measurement_attempt=measurement_attempt))
        row.update(self.geometry_fields._record_reference_delta_parameters(measurement_attempt))
        row.update(self._record_dual_geometry_parameters(measurement_attempt))
        return row

    def _snapshot_current_intrinsics(self) -> StereoIntrinsicsSnapshot | None:
        """Freeze the run-start K,D and checkerboard R,t for nominal comparison."""

        result = getattr(self.boundary, "active_intrinsics", None)
        if result is None:
            return None
        try:
            zoom = int(self.boundary.zoom_level_spin.value())
            return StereoIntrinsicsSnapshot.from_result(
                result,
                active_zoom_level=zoom,
                selected_profile_path=str(
                    getattr(self.boundary, "_active_intrinsics_profile_path", "")
                    or f"memory:nominal-calibration:zoom-{zoom:02d}"
                ),
                selected_extrinsics_profile_path=str(
                    getattr(self.boundary, "_active_extrinsics_profile_path", "") or ""
                ),
                intrinsics_source_zoom_level=getattr(
                    self.boundary, "_active_intrinsics_source_zoom_level", None
                ),
                extrinsics_source_zoom_level=getattr(
                    self.boundary, "_active_extrinsics_source_zoom_level", None
                ),
                selection_mode=str(
                    getattr(self.boundary, "_active_calibration_selection_mode", "")
                    or "legacy_combined"
                ),
            )
        except (AttributeError, TypeError, ValueError) as exc:
            logger = getattr(self.boundary, "_log", None)
            if callable(logger):
                logger(f"[저장] nominal K,D snapshot unavailable: {exc}")
            return None

    def _record_dual_geometry_parameters(
        self, measurement_attempt: MeasurementAttempt
    ) -> dict[str, object]:
        nominal = getattr(self.references, "recording_nominal_intrinsics_snapshot", None)
        active = measurement_attempt.intrinsics
        if (
            not isinstance(nominal, StereoIntrinsicsSnapshot)
            or not isinstance(active, StereoIntrinsicsSnapshot)
            or measurement_attempt.measurement_geometry_kind != "relative_pose"
        ):
            return {
                "dual_geometry_state": (
                    "not_applicable_live_pixhawk_path"
                    if measurement_attempt.measurement_geometry_kind != "relative_pose"
                    else "nominal_or_active_profile_unavailable"
                ),
                "nominal_profile_sha256": "" if not isinstance(nominal, StereoIntrinsicsSnapshot) else nominal.content_sha256,
                "active_profile_sha256": "" if not isinstance(active, StereoIntrinsicsSnapshot) else active.content_sha256,
            }
        try:
            nominal_eval, active_eval = evaluate_nominal_active(
                measurement_attempt,
                nominal_intrinsics=nominal,
                active_intrinsics=active,
                active_selection_id=str(
                    getattr(self.boundary, "_active_optical_pose_selection_id", "") or ""
                ),
                active_effective_from_pair_sequence=getattr(
                    self.boundary, "_active_optical_pose_effective_from_pair_sequence", None
                ),
            )
            fields = {}
            fields.update(nominal_eval.to_record_fields())
            fields.update(active_eval.to_record_fields())
            fields["dual_geometry_state"] = "evaluated"
            fields["nominal_optical_pose_profile_hash"] = str(
                getattr(self.references, "recording_nominal_optical_pose_profile_hash", "") or ""
            )
            return fields
        except (TypeError, ValueError, RuntimeError) as exc:
            return {
                "dual_geometry_state": "evaluation_error",
                "dual_geometry_error": str(exc),
                "nominal_profile_sha256": nominal.content_sha256,
                "active_profile_sha256": active.content_sha256,
            }

    @staticmethod
    def _measurement_result_evidence(
        *,
        attempt_sha256: str,
        status: str,
        valid: bool,
        method: str,
        quality: str,
        warnings: tuple[str, ...] | list[str] | str,
        vector_frd_range_m: tuple[float | None, float | None, float | None, float | None],
        policy_id: str = "field_measurement_v1",
        validity: str | None = None,
        hard_block_reasons: tuple[str, ...] = (),
        advisory_reasons: tuple[str, ...] = (),
        measurement_yaw: Mapping[str, object] | None = None,
        target_coordinates: Mapping[str, object] | None = None,
    ) -> dict[str, str]:
        warning_values = (
            [str(value) for value in warnings]
            if isinstance(warnings, (tuple, list))
            else [str(warnings)] if warnings else []
        )
        resolved_validity = validity or (
            "INVALID" if not valid else "ESTIMATED" if warning_values else "VALID"
        )
        payload = {
            "schema": "workstation-measurement-result-v2",
            "measurement_attempt_sha256": str(attempt_sha256),
            "status": str(status),
            "valid": bool(valid),
            "method": str(method),
            "quality": str(quality),
            "warnings": warning_values,
            "policy_id": str(policy_id),
            "validity": resolved_validity,
            "hard_block_reasons": [str(value) for value in hard_block_reasons],
            "advisory_reasons": [str(value) for value in advisory_reasons],
            "measurement_frame": MEASUREMENT_FRAME,
            "measurement_axis_contract": MEASUREMENT_AXIS_CONTRACT,
            "unit": "m",
            "vector_frd_range_m": list(vector_frd_range_m),
            "measurement_yaw": None if measurement_yaw is None else dict(measurement_yaw),
        }
        if target_coordinates is not None:
            payload.update(target_coordinates)
        encoded = json.dumps(
            payload,
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        return {
            "measurement_result_canonical_json": encoded,
            "measurement_result_sha256": hashlib.sha256(encoded.encode("utf-8")).hexdigest(),
        }

    def _record_unresolved_pair(
        self,
        measurement_attempt: MeasurementAttempt,
        *,
        status: str,
        warnings: tuple[str, ...] = (),
    ) -> None:
        if not self.session.distance_recording_enabled:
            return
        row = self._pair_recording_base_row(measurement_attempt)
        row.update(
            {
                "pair_status": status,
                "measurement_valid": "false",
                "measurement_policy_id": measurement_attempt.measurement_policy_id,
                "measurement_validity": "INVALID",
                "measurement_hard_block_reasons": json.dumps(
                    list(measurement_attempt.hard_block_reasons or (status,)),
                    separators=(",", ":"),
                    ensure_ascii=False,
                ),
                "measurement_advisory_reasons": json.dumps(
                    list(warnings or measurement_attempt.advisory_reasons),
                    separators=(",", ":"),
                    ensure_ascii=False,
                ),
                "warnings": "; ".join(warnings or measurement_attempt.warnings),
            }
        )
        row.update(
            self._measurement_result_evidence(
                attempt_sha256=measurement_attempt.canonical_sha256(),
                status=status,
                valid=False,
                method=measurement_attempt.measurement_method_id,
                quality=measurement_attempt.source_quality,
                warnings=warnings or measurement_attempt.warnings,
                policy_id=measurement_attempt.measurement_policy_id,
                validity="INVALID",
                hard_block_reasons=measurement_attempt.hard_block_reasons or (status,),
                advisory_reasons=warnings or measurement_attempt.advisory_reasons,
                vector_frd_range_m=(None, None, None, None),
            )
        )
        self.primary_persistence._upsert_pair_recording_row(measurement_attempt, row)

    def _record_blocked_measurement_pair(self, blocked: BlockedMeasurementPair) -> None:
        if not self.session.distance_recording_enabled:
            return
        key = tuple(blocked.detection_pair_key)
        blocked_sha256 = blocked.canonical_sha256()
        row: dict[str, object] = {
            "session_id": self.session.recording_session_id,
            "measurement_id": str(getattr(self.boundary, "_measurement_id", "") or ""),
            "measurement_package_id": str(
                getattr(self.boundary, "_measurement_id", "") or ""
            ),
            "terminal_record_id": blocked.terminal_record_id,
            "schema_version": "stereowiris-paired-frame-v5",
            "measurement_valid": "false",
            "measurement_policy_id": "field_measurement_v1",
            "measurement_validity": "INVALID",
            "measurement_hard_block_reasons": json.dumps(
                list(blocked.reasons), separators=(",", ":"), ensure_ascii=False
            ),
            "measurement_advisory_reasons": "[]",
            "pair_status": blocked.status,
            "measurement_geometry_kind": "unresolved",
            "measurement_method_id": "",
            "measurement_method_instance_id": "",
            "measurement_attempt_sha256": "",
            "blocked_pair_sha256": blocked_sha256,
            "blocked_pair_canonical_json": blocked.canonical_json(),
            "measurement_readiness_sha256": blocked.readiness_sha256,
            "measurement_readiness_canonical_json": blocked.readiness_payload_json,
            "measurement_attempt_timestamp_basis": blocked.timestamp_basis,
            "measurement_detection_pair_key": json.dumps(key, separators=(",", ":")),
            "warnings": "; ".join(blocked.reasons),
            "recorded_unix_ns": int(round(blocked.created_wall_s * 1_000_000_000.0)),
            "pair_app_mid_monotonic_ns": int(
                round(
                    (
                        blocked.left_detection.timestamp_s
                        + blocked.right_detection.timestamp_s
                    )
                    * 0.5
                    * 1_000_000_000.0
                )
            ),
            "selected_left_pairing_timestamp_s": blocked.left_detection.pairing_timestamp_s,
            "selected_right_pairing_timestamp_s": blocked.right_detection.pairing_timestamp_s,
            "left_bbox_xyxy_px": json.dumps(blocked.left_detection.bbox_xyxy_px),
            "right_bbox_xyxy_px": json.dumps(blocked.right_detection.bbox_xyxy_px),
            "left_class_id": blocked.left_detection.class_id,
            "right_class_id": blocked.right_detection.class_id,
        }
        row.update(
            self.native_submission._persist_native_detection_pair(
                left_detection=blocked.left_detection,
                right_detection=blocked.right_detection,
                pair_id=f"blocked-{blocked_sha256[:20]}",
                purpose="blocked-measurement",
                record_key=key,
            )
        )
        timing_writer = self.blocked_timing_writer
        if callable(timing_writer):
            row.update(timing_writer(blocked))
        else:
            timing = live_pair_timing_evidence(
                blocked.left_detection,
                blocked.right_detection,
                requested_basis=blocked.requested_timestamp_basis,
                effective_basis=blocked.timestamp_basis,
                fallback_reason=blocked.timing_fallback_reason,
            )
            rtcp_delta_s = timing.get("rtcp_pair_delta_s")
            row.update(
                {
                    "pairing_time_basis_requested": timing[
                        "requested_time_basis"
                    ],
                    "pairing_time_basis": timing["time_basis"],
                    "pairing_time_basis_role": timing["time_basis_role"],
                    "pairing_fallback_active": timing["fallback_active"],
                    "pairing_fallback_reason": timing["fallback_reason"],
                    "left_app_receive_monotonic_ns": timing[
                        "left_app_receive_monotonic_ns"
                    ],
                    "right_app_receive_monotonic_ns": timing[
                        "right_app_receive_monotonic_ns"
                    ],
                    "app_receive_pair_delta_ns": timing[
                        "app_receive_pair_delta_ns"
                    ],
                    "selected_pair_delta_ns": timing["selected_pair_delta_ns"],
                    "selected_pair_abs_delta_ns": timing[
                        "selected_pair_abs_delta_ns"
                    ],
                    "sync_delta_ns": timing["selected_pair_delta_ns"],
                    "rtcp_pair_delta_ns": (
                        ""
                        if rtcp_delta_s is None
                        else int(round(float(rtcp_delta_s) * 1_000_000_000.0))
                    ),
                    "ntp_evidence_role": timing["ntp_evidence_role"],
                    "pair_timing_limit_ns": LIVE_DISTANCE_MAX_APP_DELTA_NS,
                    "pair_timing_valid": True,
                }
            )
        row.update(
            self._measurement_result_evidence(
                attempt_sha256=blocked_sha256,
                status=blocked.status,
                valid=False,
                method="",
                quality="BLOCKED",
                warnings=blocked.reasons,
                vector_frd_range_m=(None, None, None, None),
            )
        )
        owner = self.rows.distance_recording_rows
        row_index = self.ledger._recording_pair_row_index.get(key)
        if row_index is None:
            row["sample_index"] = len(owner)
            row["pair_sequence"] = row["sample_index"]
            atomic_appender = getattr(owner, "append_with_admission", None)
            if callable(atomic_appender):
                admission = atomic_appender(row)
            else:
                appended = owner.append(row)
                admission = RowWriteAdmission(
                    ROW_WRITE_ADMITTED if appended is not False else ROW_WRITE_RETRYABLE,
                    row_index=len(owner) - 1,
                )
            if admission.state not in (ROW_WRITE_ADMITTED, ROW_WRITE_RETRYABLE):
                # The store did not (and will not) acquire a slot for this row,
                # so the BLOCKED evidence goes straight to the measurements
                # stream (same fallback as _upsert_pair_recording_row). The
                # pair is left untracked: there is no owner row for a native
                # completion to replace, and tracking len(owner) would alias
                # onto whatever row happens to occupy this index next.
                self.ledger._recording_pair_row_index.pop(key, None)
                self.ledger._pair_recording_row_leases.pop(key, None)
                self.primary_persistence._append_measurement_row_direct(row, reason=admission.reason)
                self.presentation._refresh_recording_status_label()
                self.boundary._refresh_operation_header()
                return
            row_index = int(admission.row_index if admission.row_index is not None else row["sample_index"])
            self.ledger._recording_pair_row_index[key] = row_index
            if admission.lease is not None:
                self.ledger._pair_recording_row_leases[key] = admission.lease
            self.rows.distance_recording_keys.append(key)
            if len(self.rows.distance_recording_keys) > RECORDING_KEY_CACHE_LIMIT:
                self.rows.distance_recording_keys.pop(0)
            minimum_cached_index = max(
                0,
                len(owner) - RECORDING_KEY_CACHE_LIMIT,
            )
            self.ledger._recording_pair_row_index = {
                cached_key: cached_index
                for cached_key, cached_index in self.ledger._recording_pair_row_index.items()
                if cached_index >= minimum_cached_index
            }
            self.ledger._pair_recording_row_leases = {
                cached_key: lease
                for cached_key, lease in self.ledger._pair_recording_row_leases.items()
                if cached_key in self.ledger._recording_pair_row_index
            }
            self.native_completion._track_pending_native_row(row_index, row, row_lease=admission.lease)
            self.presentation._append_recording_table_row(row)
        else:
            try:
                current = owner[row_index]
            except (IndexError, KeyError, TypeError):
                # The owner may have been cleared between the retryable append
                # and this bundle fallback.  Let replace_with_admission classify
                # the stale index/lease atomically so the canonical row can be
                # rerouted to direct persistence instead of escaping as an
                # indexing exception.
                current = {}
            sample_index = current.get("sample_index", row_index)
            pair_sequence = current.get("pair_sequence", row_index)
            current.update(row)
            current["sample_index"] = sample_index
            current["pair_sequence"] = pair_sequence
            atomic_replacer = getattr(owner, "replace_with_admission", None)
            if callable(atomic_replacer):
                admission = atomic_replacer(
                    row_index,
                    current,
                    lease=self.ledger._pair_recording_row_leases.get(key),
                )
            else:
                replacer = getattr(owner, "replace", None)
                try:
                    replaced = (
                        replacer(row_index, current)
                        if callable(replacer)
                        else owner.__setitem__(row_index, current)
                    )
                except (IndexError, OSError, RuntimeError, TypeError, ValueError) as exc:
                    admission = RowWriteAdmission(
                        ROW_WRITE_UNAVAILABLE,
                        f"legacy measurement replacement failed: {exc}",
                        row_index=row_index,
                    )
                else:
                    admission = RowWriteAdmission(
                        ROW_WRITE_ADMITTED if replaced is not False else ROW_WRITE_RETRYABLE,
                        "" if replaced is not False else "legacy measurement replacement was not admitted",
                        row_index=row_index,
                    )
            if admission.state == ROW_WRITE_UNAVAILABLE:
                # The lease is stale (row cleared/reused underneath us) or the
                # owner can no longer take writes; stop tracking this pair
                # instead of silently rewriting whatever row now lives at this
                # index, and keep the BLOCKED evidence via the direct path.
                self.ledger._recording_pair_row_index.pop(key, None)
                self.ledger._pair_recording_row_leases.pop(key, None)
                self.primary_persistence._append_measurement_row_direct(current, reason=admission.reason)
                self.presentation._refresh_recording_status_label()
                self.boundary._refresh_operation_header()
                return
            if admission.lease is not None:
                self.ledger._pair_recording_row_leases[key] = admission.lease
            self.native_completion._track_pending_native_row(
                row_index, row, row_lease=self.ledger._pair_recording_row_leases.get(key)
            )
            self.presentation._refresh_recording_table()
        self.presentation._refresh_recording_status_label()
        self.boundary._refresh_operation_header()

    def _compact_primary_retry_row(
        self,
        *,
        estimate,
        distance_m: float,
        method: str,
        baseline_source: str,
        baseline_m: float,
        point_m: tuple[float, float, float] | None,
        raw_point_m: tuple[float, float, float] | None,
        raw_point_left_m: tuple[float, float, float] | None,
        measurement_attempt: MeasurementAttempt,
        outcome_metadata: Mapping[str, object],
    ) -> dict[str, object]:
        """Build side-effect-free primary evidence after bounded row-cache eviction."""

        warnings = tuple(str(value) for value in getattr(estimate, "warnings", ()))
        vector = (
            (None, None, None, float(distance_m))
            if point_m is None
            else (
                float(point_m[0]),
                float(point_m[1]),
                float(point_m[2]),
                float(distance_m),
            )
        )
        row: dict[str, object] = {
            "record_kind": "primary_measurement_evidence",
            "event_type": "primary_measurement_retry_cache_miss",
            "session_id": str(getattr(self.session, "recording_session_id", "") or ""),
            "measurement_id": str(
                getattr(self.boundary, "_measurement_id", "")
                or outcome_metadata.get("runtime_generation")
                or ""
            ),
            "runtime_generation": str(
                outcome_metadata.get("runtime_generation") or ""
            ),
            "measurement_attempt_sha256": measurement_attempt.canonical_sha256(),
            "measurement_attempt_canonical_json": measurement_attempt.canonical_json(),
            "measurement_detection_pair_key": json.dumps(
                measurement_attempt.detection_pair_key,
                separators=(",", ":"),
            ),
            "measurement_valid": "true",
            "measurement_validity": "ESTIMATED" if warnings else "VALID",
            "pair_status": "measured",
            "distance_m": float(distance_m),
            "method": str(method),
            "quality": str(getattr(estimate, "sync_quality", "")),
            "baseline_source": str(baseline_source),
            "baseline_m": float(baseline_m),
            "baseline_center_forward_m": vector[0],
            "baseline_center_right_m": vector[1],
            "baseline_center_down_m": vector[2],
            "baseline_center_range_m": vector[3],
            "raw_baseline_center_vector_m": (
                None if raw_point_m is None else list(raw_point_m)
            ),
            "source_left_camera_rdf_m": (
                None if raw_point_left_m is None else list(raw_point_left_m)
            ),
            "native_pair_projection": "authoritative_native.frames_event",
            **{str(name): value for name, value in outcome_metadata.items()},
        }
        target_coordinates = target_coordinates_from_attempt(measurement_attempt.canonical_payload(), raw_point_left_m)
        row.update(target_coordinates)
        row.update(
            self._measurement_result_evidence(
                target_coordinates=target_coordinates,
                attempt_sha256=measurement_attempt.canonical_sha256(),
                status="measured",
                valid=True,
                method=method,
                quality=str(getattr(estimate, "sync_quality", "")),
                warnings=warnings,
                vector_frd_range_m=vector,
                policy_id=measurement_attempt.measurement_policy_id,
                validity="ESTIMATED" if warnings else "VALID",
                advisory_reasons=warnings,
            )
        )
        return row
