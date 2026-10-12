"""Commit primary measurement outcomes and publish bounded distance history.

History visibility and canonical row admission remain separate outcomes."""
from __future__ import annotations
import json
import math
import time
from collections.abc import Mapping
from Workstation.workstation_core.measurement_attempt import MeasurementAttempt
from Workstation.workstation_core.coordinate_contract import LEGACY_XYZ_ALIAS_CONTRACT, MEASUREMENT_AXIS_CONTRACT, MEASUREMENT_FRAME, PUBLIC_MEASUREMENT_VECTOR_CONTRACT
from Workstation.workstation_core.measurement_widgets import DistanceHistoryPoint
from Workstation.workstation_core.target_coordinates import TARGET_COORDINATE_FIELDS, target_coordinates_from_attempt
from Workstation.workstation_core.measurement_data_store import ROW_WRITE_ADMITTED, RowWriteAdmission
from .contracts import (DistanceHistoryCommitResult, PairRecordingCommitResult)
from .state import (PrimaryPersistenceLedger, RecordingRowsState, RecordingSessionState, VideoSegmentState)
from .ports import HistoryPort
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .calibration_fields import CalibrationFields
    from .formatting import Formatting
    from .geometry_fields import GeometryFields
    from .measurement_rows import MeasurementRows
    from .primary_persistence import PrimaryPersistence
    from .timing_fields import TimingFields


class History:
    def __init__(self, *,
        ledger: PrimaryPersistenceLedger,
        rows: RecordingRowsState,
        session: RecordingSessionState,
        video_state: VideoSegmentState,
        boundary: HistoryPort,
    ) -> None:
        self.ledger = ledger
        self.rows = rows
        self.session = session
        self.video_state = video_state
        self.boundary = boundary

    def bind_collaborators(self, *,
        calibration_fields: "CalibrationFields",
        formatting: "Formatting",
        geometry_fields: "GeometryFields",
        measurement_rows: "MeasurementRows",
        primary_persistence: "PrimaryPersistence",
        timing_fields: "TimingFields",
    ) -> None:
        self.calibration_fields = calibration_fields
        self.formatting = formatting
        self.geometry_fields = geometry_fields
        self.measurement_rows = measurement_rows
        self.primary_persistence = primary_persistence
        self.timing_fields = timing_fields

    def _append_distance_history(
        self,
        *,
        estimate,
        distance_m: float,
        method: str,
        baseline_source: str,
        baseline_m: float,
        point_m: tuple[float, float, float] | None = None,
        raw_point_m: tuple[float, float, float] | None = None,
        raw_point_left_m: tuple[float, float, float] | None = None,
        point_left_m: tuple[float, float, float] | None = None,
        measurement_attempt: MeasurementAttempt,
    ) -> bool:
        result = self._commit_distance_history(
            estimate=estimate,
            distance_m=distance_m,
            method=method,
            baseline_source=baseline_source,
            baseline_m=baseline_m,
            point_m=point_m,
            raw_point_m=raw_point_m,
            raw_point_left_m=raw_point_left_m,
            point_left_m=point_left_m,
            measurement_attempt=measurement_attempt,
            publish_history=True,
        )
        return result.history_added

    def _commit_primary_distance_outcome(
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
        publish_history: bool,
    ) -> DistanceHistoryCommitResult:
        key = tuple(measurement_attempt.detection_pair_key)
        if not publish_history:
            retry_row = self.primary_persistence._pending_primary_retry_row(key)
            if retry_row is not None:
                retry_row.update(
                    {str(name): value for name, value in outcome_metadata.items()}
                )
                with self.ledger._primary_persistence_lock:
                    if self.primary_persistence._consume_native_primary_persisted(key):
                        row_commit = PairRecordingCommitResult(
                            inserted=False,
                            enqueue_admitted=True,
                            row_index=-1,
                            admission_state=ROW_WRITE_ADMITTED,
                            admission_reason=(
                                "native completion already admitted the full primary row"
                            ),
                        )
                    else:
                        row_commit = self.primary_persistence._upsert_pair_recording_row(
                            measurement_attempt,
                            retry_row,
                        )
                    if row_commit.enqueue_admitted:
                        self.ledger._pending_primary_record_rows.pop(key, None)
                        self.ledger._primary_retry_required_keys.discard(key)
                    else:
                        self.primary_persistence._remember_pending_primary_row(key, retry_row)
                return DistanceHistoryCommitResult(
                    history_added=False,
                    row_commit=row_commit,
                )
            compact_row = self.measurement_rows._compact_primary_retry_row(
                estimate=estimate,
                distance_m=distance_m,
                method=method,
                baseline_source=measurement_attempt.baseline_source,
                baseline_m=measurement_attempt.baseline_m,
                point_m=point_m,
                raw_point_m=raw_point_m,
                raw_point_left_m=raw_point_left_m,
                measurement_attempt=measurement_attempt,
                outcome_metadata=outcome_metadata,
            )
            with self.ledger._primary_persistence_lock:
                if self.primary_persistence._consume_native_primary_persisted(key):
                    admission = RowWriteAdmission(
                        ROW_WRITE_ADMITTED,
                        "native completion already admitted the full primary row",
                    )
                    direct_persistence = False
                else:
                    admission = self.primary_persistence._append_measurement_row_direct(
                        compact_row,
                        reason="primary full-row retry cache is unavailable",
                        persistence_mode="direct_retry_cache_miss",
                    )
                    direct_persistence = admission.admitted
                    if admission.admitted:
                        self.primary_persistence._remember_compact_primary_persisted(key)
                        self.ledger._primary_retry_required_keys.discard(key)
            return DistanceHistoryCommitResult(
                history_added=False,
                row_commit=PairRecordingCommitResult(
                    inserted=False,
                    enqueue_admitted=admission.admitted,
                    row_index=-1,
                    admission_state=admission.state,
                    admission_reason=admission.reason,
                    direct_persistence=direct_persistence,
                ),
            )
        return self._commit_distance_history(
            estimate=estimate,
            distance_m=distance_m,
            method=method,
            baseline_source=baseline_source,
            baseline_m=baseline_m,
            point_m=point_m,
            raw_point_m=raw_point_m,
            raw_point_left_m=raw_point_left_m,
            measurement_attempt=measurement_attempt,
            publish_history=bool(publish_history),
            row_metadata=outcome_metadata,
        )

    def _commit_distance_history(
        self,
        *,
        estimate,
        distance_m: float,
        method: str,
        baseline_source: str,
        baseline_m: float,
        point_m: tuple[float, float, float] | None = None,
        raw_point_m: tuple[float, float, float] | None = None,
        raw_point_left_m: tuple[float, float, float] | None = None,
        point_left_m: tuple[float, float, float] | None = None,
        measurement_attempt: MeasurementAttempt,
        publish_history: bool,
        row_metadata: Mapping[str, object] | None = None,
    ) -> DistanceHistoryCommitResult:
        baseline_source = measurement_attempt.baseline_source
        baseline_m = measurement_attempt.baseline_m
        key = tuple(measurement_attempt.detection_pair_key)
        if (
            publish_history
            and row_metadata is None
            and key == self.boundary._last_distance_history_key
        ):
            return DistanceHistoryCommitResult(history_added=False)
        if point_m is None:
            point_m = point_left_m
        if raw_point_m is None:
            raw_point_m = raw_point_left_m
        timestamp_s = max(estimate.left.timestamp_s, estimate.right.timestamp_s)
        class_name = self._distance_history_class_name(estimate)
        recording_elapsed_s = None
        if self.session.distance_recording_enabled:
            if self.session.distance_recording_started_monotonic_s is None:
                self.session.distance_recording_started_monotonic_s = time.monotonic()
            recording_elapsed_s = max(0.0, time.monotonic() - self.session.distance_recording_started_monotonic_s)
        target_coordinates = target_coordinates_from_attempt(
            measurement_attempt.canonical_payload(), raw_point_left_m
        )
        point = DistanceHistoryPoint(
            timestamp_s=timestamp_s,
            distance_m=distance_m,
            quality=estimate.sync_quality,
            method=method,
            forward_m=None if point_m is None else float(point_m[0]),
            right_m=None if point_m is None else float(point_m[1]),
            down_m=None if point_m is None else float(point_m[2]),
            class_name=class_name,
            sample_key=tuple(measurement_attempt.detection_pair_key),
            recording_elapsed_s=recording_elapsed_s,
            **{name: target_coordinates[name] for name in TARGET_COORDINATE_FIELDS},
        )
        if not self.session.distance_recording_enabled:
            if publish_history:
                self.boundary._last_distance_history_key = key
                self.boundary.distance_history.append(point)
                self.boundary.distance_plot.add_point(point)
            return DistanceHistoryCommitResult(history_added=publish_history)
        raw_distance_m = None
        if raw_point_m is not None:
            raw_distance_m = math.sqrt(
                raw_point_m[0] * raw_point_m[0]
                + raw_point_m[1] * raw_point_m[1]
                + raw_point_m[2] * raw_point_m[2]
            )
        measurement_attempt_content_sha256 = measurement_attempt.content_sha256()
        measurement_attempt_instance_sha256 = measurement_attempt.instance_sha256()
        row = self.measurement_rows._pair_recording_base_row(measurement_attempt)
        row.update({
            "session_id": self.session.recording_session_id,
            "measurement_id": str(getattr(self.boundary, "_measurement_id", "") or ""),
            "measurement_attempt_id": str(measurement_attempt.attempt_id),
            "measurement_attempt_session_id": measurement_attempt.attempt_session_id,
            "measurement_attempt_sha256": measurement_attempt.canonical_sha256(),
            "measurement_detection_pair_key": json.dumps(
                measurement_attempt.detection_pair_key, separators=(",", ":")
            ),
            "measurement_attempt_timestamp_basis": measurement_attempt.timestamp_basis,
            "selected_left_pairing_timestamp_s": self.formatting._fmt_record_float(
                measurement_attempt.left_detection.pairing_timestamp_s, 6
            ),
            "selected_right_pairing_timestamp_s": self.formatting._fmt_record_float(
                measurement_attempt.right_detection.pairing_timestamp_s, 6
            ),
            "measurement_attempt_source_quality": measurement_attempt.source_quality,
            "measurement_attempt_warnings": "; ".join(measurement_attempt.warnings),
            "measurement_policy_id": measurement_attempt.measurement_policy_id,
            "measurement_validity": (
                "ESTIMATED" if estimate.warnings else "VALID"
            ),
            "measurement_hard_block_reasons": json.dumps([], separators=(",", ":")),
            "measurement_advisory_reasons": json.dumps(
                list(estimate.warnings), separators=(",", ":"), ensure_ascii=False
            ),
            "session_started_unix_s": self.formatting._fmt_record_float(self.session.recording_session_started_wall_s, 6),
            "left_video_path": str(self.video_state.video_recording_paths.get("left", "")),
            "right_video_path": str(self.video_state.video_recording_paths.get("right", "")),
            "timestamp_s": f"{timestamp_s:.6f}",
            "distance_m": f"{distance_m:.6f}",
            "method": method,
            "measurement_method_id": measurement_attempt.measurement_method_id,
            "measurement_geometry_kind": measurement_attempt.measurement_geometry_kind,
            "measurement_yaw_source": measurement_attempt.measurement_yaw_source,
            "measurement_yaw_selection_mode": measurement_attempt.measurement_yaw_selection_mode,
            "measurement_yaw_branch_offset_deg": self.formatting._fmt_record_float(
                measurement_attempt.measurement_yaw_branch_offset_deg, 6
            ),
            "measurement_yaw_baseline_azimuth_deg": self.formatting._fmt_record_float(
                measurement_attempt.measurement_yaw_baseline_azimuth_deg, 6
            ),
            "measurement_yaw_selected_deg": self.formatting._fmt_record_float(
                measurement_attempt.measurement_yaw_selected_deg, 6
            ),
            "measurement_yaw_p1_reference_deg": self.formatting._fmt_record_float(
                measurement_attempt.measurement_yaw_p1_reference_deg, 6
            ),
            "measurement_yaw_p2_reference_deg": self.formatting._fmt_record_float(
                measurement_attempt.measurement_yaw_p2_reference_deg, 6
            ),
            "measurement_yaw_p1_age_s": self.formatting._fmt_record_float(
                measurement_attempt.measurement_yaw_p1_age_s, 6
            ),
            "measurement_yaw_p2_age_s": self.formatting._fmt_record_float(
                measurement_attempt.measurement_yaw_p2_age_s, 6
            ),
            "measurement_yaw_score_minus90_deg": self.formatting._fmt_record_float(
                measurement_attempt.measurement_yaw_score_minus90_deg, 6
            ),
            "measurement_yaw_score_plus90_deg": self.formatting._fmt_record_float(
                measurement_attempt.measurement_yaw_score_plus90_deg, 6
            ),
            "measurement_yaw_reverse": bool(measurement_attempt.measurement_yaw_reverse),
            "measurement_yaw_consensus": measurement_attempt.measurement_yaw_consensus,
            "measurement_yaw_low_margin": bool(measurement_attempt.measurement_yaw_low_margin),
            "measurement_yaw_candidate_minus90_deg": self.formatting._fmt_record_float(
                measurement_attempt.measurement_yaw_candidate_minus90_deg, 6
            ),
            "measurement_yaw_candidate_plus90_deg": self.formatting._fmt_record_float(
                measurement_attempt.measurement_yaw_candidate_plus90_deg, 6
            ),
            "measurement_yaw_alternate_deg": self.formatting._fmt_record_float(
                measurement_attempt.measurement_yaw_alternate_deg, 6
            ),
            "measurement_yaw_p1_error_minus90_deg": self.formatting._fmt_record_float(
                measurement_attempt.measurement_yaw_p1_error_minus90_deg, 6
            ),
            "measurement_yaw_p1_error_plus90_deg": self.formatting._fmt_record_float(
                measurement_attempt.measurement_yaw_p1_error_plus90_deg, 6
            ),
            "measurement_yaw_p2_error_minus90_deg": self.formatting._fmt_record_float(
                measurement_attempt.measurement_yaw_p2_error_minus90_deg, 6
            ),
            "measurement_yaw_p2_error_plus90_deg": self.formatting._fmt_record_float(
                measurement_attempt.measurement_yaw_p2_error_plus90_deg, 6
            ),
            "geometry_left_quaternion_wxyz_alternate": (
                None
                if measurement_attempt.geometry_left_quaternion_wxyz_alternate is None
                else json.dumps(
                    list(measurement_attempt.geometry_left_quaternion_wxyz_alternate),
                    separators=(",", ":"),
                )
            ),
            "geometry_right_quaternion_wxyz_alternate": (
                None
                if measurement_attempt.geometry_right_quaternion_wxyz_alternate is None
                else json.dumps(
                    list(measurement_attempt.geometry_right_quaternion_wxyz_alternate),
                    separators=(",", ":"),
                )
            ),
            "quality": estimate.sync_quality,
            "disparity_px": f"{estimate.disparity_px:.6f}",
            "raw_disparity_px": f"{estimate.disparity_px:.6f}",
            "camera_delta_s": f"{estimate.time_delta_s:.6f}",
            "vertical_delta_px": f"{estimate.vertical_delta_px:.6f}",
            "baseline_source": baseline_source,
            "baseline_m": f"{baseline_m:.6f}",
            "measurement_frame": MEASUREMENT_FRAME,
            "measurement_axis_contract": MEASUREMENT_AXIS_CONTRACT,
            "legacy_xyz_alias_contract": LEGACY_XYZ_ALIAS_CONTRACT,
            "point_frame": MEASUREMENT_FRAME,
            "output_frame": MEASUREMENT_FRAME,
            "public_vector_contract": PUBLIC_MEASUREMENT_VECTOR_CONTRACT,
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
            "measurement_method": "two_ray_closest_midpoint",
            "range_m": self.formatting._fmt_record_float(distance_m, 6),
            "baseline_center_forward_m": self.formatting._fmt_record_float(point.forward_m, 6),
            "baseline_center_right_m": self.formatting._fmt_record_float(point.right_m, 6),
            "baseline_center_down_m": self.formatting._fmt_record_float(point.down_m, 6),
            "baseline_center_range_m": self.formatting._fmt_record_float(distance_m, 6),
            "raw_baseline_center_forward_m": self.formatting._fmt_record_float(
                None if raw_point_m is None else raw_point_m[0], 6
            ),
            "raw_baseline_center_right_m": self.formatting._fmt_record_float(
                None if raw_point_m is None else raw_point_m[1], 6
            ),
            "raw_baseline_center_down_m": self.formatting._fmt_record_float(
                None if raw_point_m is None else raw_point_m[2], 6
            ),
            "raw_baseline_center_range_m": self.formatting._fmt_record_float(raw_distance_m, 6),
            "source_left_camera_rdf_right_m": self.formatting._fmt_record_float(
                None if raw_point_left_m is None else raw_point_left_m[0], 6
            ),
            "source_left_camera_rdf_down_m": self.formatting._fmt_record_float(
                None if raw_point_left_m is None else raw_point_left_m[1], 6
            ),
            "source_left_camera_rdf_forward_m": self.formatting._fmt_record_float(
                None if raw_point_left_m is None else raw_point_left_m[2], 6
            ),
            "recording_elapsed_s": self.formatting._fmt_record_float(recording_elapsed_s, 6),
            "warnings": "; ".join(estimate.warnings),
            "pair_status": "measured",
            "measurement_valid": "true",
        })
        if row_metadata:
            row.update({str(key): value for key, value in row_metadata.items()})
        row.update(self.timing_fields._record_bbox_parameters(estimate))
        row.update(self.timing_fields._record_measurement_sync_timestamp_parameters(estimate))
        row.update(self.calibration_fields._record_intrinsic_parameters(measurement_attempt))
        row.update(self.timing_fields._record_frame_timestamp_parameters(measurement_attempt))
        row.update(self.timing_fields._record_live_telemetry_snapshot_parameters(measurement_attempt))
        row.update(self.geometry_fields._record_external_parameters(estimate=estimate, measurement_attempt=measurement_attempt))
        row.update(target_coordinates)
        row.update(
            self.measurement_rows._measurement_result_evidence(
                target_coordinates=target_coordinates,
                attempt_sha256=measurement_attempt.canonical_sha256(),
                status="measured",
                valid=True,
                method=method,
                quality=estimate.sync_quality,
                warnings=tuple(estimate.warnings),
                policy_id=measurement_attempt.measurement_policy_id,
                validity="ESTIMATED" if estimate.warnings else "VALID",
                advisory_reasons=tuple(estimate.warnings),
                measurement_yaw={
                    "source": measurement_attempt.measurement_yaw_source,
                    "selection_mode": measurement_attempt.measurement_yaw_selection_mode,
                    "branch_offset_deg": measurement_attempt.measurement_yaw_branch_offset_deg,
                    "baseline_azimuth_deg": measurement_attempt.measurement_yaw_baseline_azimuth_deg,
                    "selected_yaw_deg": measurement_attempt.measurement_yaw_selected_deg,
                    "p1_reference_yaw_deg": measurement_attempt.measurement_yaw_p1_reference_deg,
                    "p2_reference_yaw_deg": measurement_attempt.measurement_yaw_p2_reference_deg,
                    "p1_age_s": measurement_attempt.measurement_yaw_p1_age_s,
                    "p2_age_s": measurement_attempt.measurement_yaw_p2_age_s,
                    "score_minus90_deg": measurement_attempt.measurement_yaw_score_minus90_deg,
                    "score_plus90_deg": measurement_attempt.measurement_yaw_score_plus90_deg,
                    "reverse": measurement_attempt.measurement_yaw_reverse,
                    "consensus": measurement_attempt.measurement_yaw_consensus,
                    "low_margin": measurement_attempt.measurement_yaw_low_margin,
                    "candidate_minus90_deg": measurement_attempt.measurement_yaw_candidate_minus90_deg,
                    "candidate_plus90_deg": measurement_attempt.measurement_yaw_candidate_plus90_deg,
                    "alternate_yaw_deg": measurement_attempt.measurement_yaw_alternate_deg,
                    "p1_error_minus90_deg": measurement_attempt.measurement_yaw_p1_error_minus90_deg,
                    "p1_error_plus90_deg": measurement_attempt.measurement_yaw_p1_error_plus90_deg,
                    "p2_error_minus90_deg": measurement_attempt.measurement_yaw_p2_error_minus90_deg,
                    "p2_error_plus90_deg": measurement_attempt.measurement_yaw_p2_error_plus90_deg,
                    "geometry_left_quaternion_wxyz_alternate": (
                        None
                        if measurement_attempt.geometry_left_quaternion_wxyz_alternate is None
                        else list(measurement_attempt.geometry_left_quaternion_wxyz_alternate)
                    ),
                    "geometry_right_quaternion_wxyz_alternate": (
                        None
                        if measurement_attempt.geometry_right_quaternion_wxyz_alternate is None
                        else list(measurement_attempt.geometry_right_quaternion_wxyz_alternate)
                    ),
                },
                vector_frd_range_m=(
                    point.forward_m,
                    point.right_m,
                    point.down_m,
                    distance_m,
                ),
            )
        )
        row.pop("measurement_attempt_content_sha256", None)
        row.pop("measurement_attempt_instance_sha256", None)
        row["measurement_attempt_content_sha256"] = measurement_attempt_content_sha256
        row["measurement_attempt_instance_sha256"] = measurement_attempt_instance_sha256
        row_commit = self.primary_persistence._upsert_pair_recording_row(measurement_attempt, row)
        if row_metadata is not None:
            # Every other mutator of this retry-tracking state
            # (_persist_compact_primary_measurement, _release_primary_retry_ownership,
            # _commit_primary_distance_outcome) holds _primary_persistence_lock
            # before touching _primary_retry_required_keys /
            # _pending_primary_record_rows; do the same here.
            with self.ledger._primary_persistence_lock:
                if row_commit.enqueue_admitted:
                    self.ledger._pending_primary_record_rows.pop(key, None)
                    self.ledger._primary_retry_required_keys.discard(key)
                else:
                    self.ledger._primary_retry_required_keys.add(key)
                    self.primary_persistence._remember_pending_primary_row(key, row)
        if publish_history:
            self.boundary._last_distance_history_key = key
            self.boundary.distance_history.append(point)
            self.boundary.distance_plot.add_point(point)
        if row_commit.row_index >= 0 and publish_history:
            self._append_extrinsics_comparison_snapshot(
                row_commit.row_index,
                timestamp_s,
            )
        return DistanceHistoryCommitResult(
            history_added=publish_history,
            row_commit=row_commit,
        )

    def _append_extrinsics_comparison_snapshot(self, pair_sequence: int, timestamp_s: float) -> None:
        """Snapshot analysis-only per-method previews for one accepted pair.

        ``geometry_comparison_rows`` is recomputed every ``_refresh_distance``
        tick; only the value at the exact moment a pair is accepted is worth
        persisting, so this is called once from ``_append_distance_history``
        rather than on every tick. These rows are a separate analysis namespace,
        never an official Measurement result or MeasurementAttempt.
        """

        if not self.session.distance_recording_enabled:
            return
        for comparison_row in getattr(self.boundary, "geometry_comparison_rows", ()) or ():
            status = str(comparison_row.get("status", ""))
            # A method with no inputs yields nothing worth a row: the same
            # "waiting for inputs" placeholder repeated per pair only bloats
            # the stream. Its absence is the signal that nothing was compared.
            if status.startswith("waiting for inputs"):
                continue
            record = {
                "pair_sequence": pair_sequence,
                "timestamp_s": self.formatting._fmt_record_float(timestamp_s, 6),
                "method": str(comparison_row.get("method", "")),
                "forward_m": self.formatting._fmt_record_float(
                    comparison_row.get("forward_m"), 6
                ),
                "right_m": self.formatting._fmt_record_float(
                    comparison_row.get("right_m"), 6
                ),
                "down_m": self.formatting._fmt_record_float(
                    comparison_row.get("down_m"), 6
                ),
                "range_m": self.formatting._fmt_record_float(
                    comparison_row.get("range_m"), 6
                ),
                "status": str(comparison_row.get("status", "")),
            }
            for name in (
                "schema_version",
                "evidence_namespace",
                "result_class",
                "not_measurement_result",
                "source_state",
                "source_profile_id",
                "source_profile_hash",
                "candidate_profile_id",
                "candidate_profile_hash",
                "candidate_intrinsics_hash",
                "active_intrinsics_hash",
                "candidate_applied",
                "candidate_applicability",
                "candidate_quality",
            ):
                value = comparison_row.get(name)
                if value is not None:
                    record[name] = value
            self.rows.extrinsics_comparison_recording_rows.append(record)

    def _distance_history_class_name(self, estimate) -> str | None:
        left_class_name = str(getattr(estimate.left, "class_name", "") or "").strip()
        right_class_name = str(getattr(estimate.right, "class_name", "") or "").strip()
        if left_class_name and right_class_name:
            if left_class_name == right_class_name:
                return left_class_name
            return f"{left_class_name}/{right_class_name}"
        return left_class_name or right_class_name or None
