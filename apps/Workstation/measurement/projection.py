"""Result-to-record projections; public points are baseline-center FRD.

Attitudes: body FRD -> earth NED, wxyz. GPS baselines: NED metres.
Optical R,t: X_right_RDF = R_right_from_left X_left_RDF + t_m.
Public point_m: baseline-center FRD; source_point_left_rdf_m: left optical RDF.
"""

from __future__ import annotations
from Workstation.workstation_core.target_level_coordinates import LEVEL_OUTPUT_FIELDS
import json
import math
import time
from collections.abc import Mapping
from dataclasses import dataclass, replace
from Workstation.workstation_core.measurement_methods import (
    PIXHAWK_RTK_LIVE,
    PIXHAWK_RP_GPS_BASELINE_YAW,
)
from Workstation.workstation_core.measurement_attempt import MeasurementAttempt
from Workstation.workstation_core.method_evaluation import (
    MethodEvaluationBundle,
    MethodEvaluationOutcome,
)
from Workstation.workstation_core.stereo import StereoEstimate
from typing import Any, Callable
from Workstation.measurement import limits
from Workstation.measurement.ports import ValueInput
from Workstation.measurement.state import UNSET, read, ResultProjectionState


@dataclass(frozen=True)
class ResultProjectionInputs:
    pass


@dataclass(frozen=True)
class ResultProjectionActions:
    pass


class ResultProjectionService:
    """Result-to-record projections; public points are baseline-center FRD."""

    def __init__(
        self,
        *,
        state: ResultProjectionState,
        inputs: ResultProjectionInputs,
        actions: ResultProjectionActions,
    ) -> None:
        self.state = state
        self.inputs = inputs
        self.actions = actions

    _METHOD_ROW_PREFIXES = {
        PIXHAWK_RTK_LIVE: "rtk",
        PIXHAWK_RP_GPS_BASELINE_YAW: "yaw90",
    }

    @staticmethod
    def _primary_outcome_key(outcome: MethodEvaluationOutcome) -> tuple[str, str, int]:
        return (
            str(outcome.measurement_id),
            str(outcome.pair_id),
            int(outcome.request_generation),
        )

    @staticmethod
    def _failed_primary_outcome(
        outcome: MethodEvaluationOutcome, reason: str
    ) -> MethodEvaluationOutcome:
        message = str(reason)
        return replace(
            outcome,
            status="FAILED",
            result={},
            reasons=(message,),
            warnings=tuple(dict.fromkeys((*outcome.warnings, message))),
        )

    @staticmethod
    def _method_bundle_key(bundle: MethodEvaluationBundle) -> tuple[str, str, int]:
        return (
            str(bundle.measurement_id),
            str(bundle.pair_id),
            int(bundle.request_generation),
        )

    @staticmethod
    def _row_scalar(value: Any) -> Any:
        """JSON-friendly scalar for a flattened row cell (numbers stay numeric)."""
        if isinstance(value, bool):
            return "true" if value else "false"
        if isinstance(value, (int, float)):
            number = float(value)
            if number != number or number in (float("inf"), float("-inf")):
                return None
            return round(number, 6)
        if value is None:
            return None
        if isinstance(value, (list, tuple)):
            return json.dumps(
                [ResultProjectionService._row_scalar(item) for item in value]
            )
        return str(value)

    @classmethod
    def _method_row_prefix(cls, method_id: str) -> str:
        return cls._METHOD_ROW_PREFIXES.get(str(method_id), str(method_id))

    def _method_result_row(
        self, bundle: MethodEvaluationBundle, outcome: MethodEvaluationOutcome
    ) -> dict[str, Any]:
        """One thin ``method_results`` row: status, F/R/D/Range, diagnostics."""
        from Workstation.workstation_core.method_result_evidence import (
            derive_method_result,
        )

        record = outcome.to_record()
        derived = derive_method_result(record)
        result = record.get("result")
        result = result if isinstance(result, Mapping) else {}
        match = result.get("match")
        match = match if isinstance(match, Mapping) else {}
        return {
            "schema": "workstation-method-result-row-v1",
            "measurement_id": outcome.measurement_id,
            "pair_id": outcome.pair_id,
            "pair_sequence": outcome.pair_sequence,
            "request_generation": outcome.request_generation,
            "method_id": outcome.method_id,
            "method_instance_id": outcome.method_instance_id,
            "status": outcome.status,
            "reasons": "; ".join((str(item) for item in outcome.reasons)),
            "warning_count": len(outcome.warnings),
            "forward_m": self._row_scalar(derived.get("forward_m")),
            "right_m": self._row_scalar(derived.get("right_m")),
            "down_m": self._row_scalar(derived.get("down_m")),
            "range_m": self._row_scalar(derived.get("range_m")),
            "target_level_evidence": derived.get("target_level_evidence", {}),
            **{
                name: derived.get(name)
                for name in (
                    "target_latitude_deg",
                    "target_longitude_deg",
                    "target_altitude_ellipsoid_m",
                    "target_altitude_msl_m",
                    *LEVEL_OUTPUT_FIELDS,
                )
            },
            "ray_gap_m": self._row_scalar(derived.get("ray_gap_m")),
            "ray_angle_deg": self._row_scalar(derived.get("ray_angle_deg")),
            "left_depth_m": self._row_scalar(derived.get("left_depth_m")),
            "right_depth_m": self._row_scalar(derived.get("right_depth_m")),
            "disparity_px": self._row_scalar(match.get("disparity_px")),
            "vertical_delta_px": self._row_scalar(match.get("vertical_delta_px")),
            "time_delta_s": self._row_scalar(match.get("time_delta_s")),
            "class_name": str(match.get("class_name") or ""),
            "queue_wait_ms": self._row_scalar(outcome.queue_wait_ms),
            "evaluation_duration_ms": self._row_scalar(outcome.evaluation_duration_ms),
            "attempt_sha256": outcome.attempt_sha256,
            "method_evaluation_bundle_sha256": bundle.to_record().get(
                "method_evaluation_bundle_sha256", ""
            ),
        }

    def _bundle_flat_fields(self, bundle: MethodEvaluationBundle) -> dict[str, Any]:
        """Per-method columns (``m_<prefix>_*``) plus baseline-yaw selection fields."""
        from Workstation.workstation_core.method_result_evidence import (
            derive_method_result,
        )

        fields: dict[str, Any] = {}
        ranges: dict[str, float | None] = {}
        for outcome in bundle.outcomes:
            prefix = f"m_{self._method_row_prefix(outcome.method_id)}"
            derived = derive_method_result(outcome.to_record())
            range_value = derived.get("range_m")
            ranges[outcome.method_id] = (
                None if range_value is None else float(range_value)
            )
            fields.update(
                {
                    f"{prefix}_status": outcome.status,
                    f"{prefix}_reason": "; ".join(
                        (str(item) for item in outcome.reasons)
                    ),
                    f"{prefix}_forward_m": self._row_scalar(derived.get("forward_m")),
                    f"{prefix}_right_m": self._row_scalar(derived.get("right_m")),
                    f"{prefix}_down_m": self._row_scalar(derived.get("down_m")),
                    **{
                        f"{prefix}_{name}": derived.get(name)
                        for name in (
                            "target_latitude_deg",
                            "target_longitude_deg",
                            "target_altitude_ellipsoid_m",
                            "target_altitude_msl_m",
                            *LEVEL_OUTPUT_FIELDS,
                        )
                    },
                    f"{prefix}_range_m": self._row_scalar(derived.get("range_m")),
                    f"{prefix}_ray_gap_m": self._row_scalar(derived.get("ray_gap_m")),
                    f"{prefix}_ray_angle_deg": self._row_scalar(
                        derived.get("ray_angle_deg")
                    ),
                }
            )
        rtk_range = ranges.get(PIXHAWK_RTK_LIVE)
        yaw90_range = ranges.get(PIXHAWK_RP_GPS_BASELINE_YAW)
        fields["m_yaw90_over_rtk_range_ratio"] = (
            None
            if not rtk_range or yaw90_range is None
            else self._row_scalar(yaw90_range / rtk_range)
        )
        fields["m_method_ids"] = ",".join(
            (outcome.method_id for outcome in bundle.outcomes)
        )
        yaw_outcome = bundle.outcome_for(PIXHAWK_RP_GPS_BASELINE_YAW)
        payload = yaw_outcome.attempt_payload if yaw_outcome is not None else None
        if isinstance(payload, Mapping):
            for key, value in payload.items():
                if str(key).startswith("measurement_yaw_"):
                    fields[str(key)] = self._row_scalar(value)
            fields["measurement_yaw_fields_source"] = PIXHAWK_RP_GPS_BASELINE_YAW
        return fields

    @staticmethod
    def _method_bundle_with_primary_failure(
        bundle: MethodEvaluationBundle,
        reason: str,
        *,
        primary_method_id: str = PIXHAWK_RTK_LIVE,
    ) -> MethodEvaluationBundle:
        primary = bundle.outcome_for(primary_method_id)
        if primary is None:
            return bundle
        failed = replace(
            primary,
            status="FAILED",
            result={},
            reasons=(str(reason),),
            warnings=tuple(dict.fromkeys((*primary.warnings, str(reason)))),
        )
        return replace(
            bundle,
            outcomes=tuple(
                (
                    failed if outcome.method_id == primary_method_id else outcome
                    for outcome in bundle.outcomes
                )
            ),
        )

    @staticmethod
    def _method_history_payload(
        outcome: MethodEvaluationOutcome, attempt: MeasurementAttempt
    ) -> dict[str, object]:
        result = outcome.result
        match = result.get("match")
        triangulated = result.get("triangulated")
        estimate_payload = result.get("estimate")
        if not all(
            (
                isinstance(value, Mapping)
                for value in (match, triangulated, estimate_payload)
            )
        ):
            raise ValueError("worker result is missing match/triangulated/estimate")

        def finite_number(mapping: Mapping[str, object], name: str) -> float:
            value = float(mapping[name])
            if not math.isfinite(value):
                raise ValueError(f"{name} must be finite")
            return value

        def vector3(
            mapping: Mapping[str, object], name: str
        ) -> tuple[float, float, float]:
            raw = mapping[name]
            if not isinstance(raw, (tuple, list)) or len(raw) != 3:
                raise ValueError(f"{name} must contain three values")
            result_vector = tuple((float(value) for value in raw))
            if not all((math.isfinite(value) for value in result_vector)):
                raise ValueError(f"{name} must be finite")
            return result_vector

        point_m = vector3(triangulated, "point_m")
        source_raw = triangulated.get("source_point_left_rdf_m")
        source_point = (
            None
            if source_raw is None
            else vector3(
                {"source_point_left_rdf_m": source_raw}, "source_point_left_rdf_m"
            )
        )
        distance_m = finite_number(triangulated, "range_m")
        estimate = StereoEstimate(
            distance_m=finite_number(estimate_payload, "distance_m"),
            disparity_px=finite_number(match, "disparity_px"),
            time_delta_s=finite_number(match, "time_delta_s"),
            vertical_delta_px=finite_number(match, "vertical_delta_px"),
            left=attempt.left_detection,
            right=attempt.right_detection,
            left_extrinsic=attempt.left_attitude,
            right_extrinsic=attempt.right_attitude,
            extrinsic_time_delta_s=(
                None
                if attempt.left_attitude is None or attempt.right_attitude is None
                else abs(
                    attempt.left_attitude.timestamp_s
                    - attempt.right_attitude.timestamp_s
                )
            ),
            left_attitude_age_s=attempt.left_attitude_age_s,
            right_attitude_age_s=attempt.right_attitude_age_s,
            attitude_pair_delta_s=attempt.attitude_pair_delta_s,
            relative_roll_deg=attempt.relative_roll_deg,
            relative_pitch_deg=attempt.relative_pitch_deg,
            relative_yaw_deg=attempt.relative_yaw_deg,
            sync_quality=str(
                estimate_payload.get("sync_quality")
                or match.get("sync_quality")
                or "unknown"
            ),
            warnings=tuple(
                (str(value) for value in estimate_payload.get("warnings", ()) or ())
            ),
        )
        return {
            "estimate": estimate,
            "distance_m": distance_m,
            "method": str(triangulated.get("method") or "unknown"),
            "point_m": point_m,
            "source_point_left_rdf_m": source_point,
        }


def optical_relative_rotation_fields(
    attempt_payload: Mapping[str, Any] | None = None,
    *,
    accepted: Mapping[str, Any] | None = None,
    now_ns: int,
) -> dict[str, Any]:
    """Latest accepted optical-pose suggestion's relative rotation, flattened.

    Lets one measurement row show the SIFT far-field relative yaw next to
    the Pixhawk relative yaw it competes with, plus how old that optical
    evidence is at the time of the pair.
    """
    fields: dict[str, Any] = {
        "optical_relative_source": "",
        "optical_relative_mode": "",
        "optical_relative_suggestion_id": "",
        "optical_relative_accepted_utc_ns": None,
        "optical_relative_age_s": None,
        "optical_relative_yaw_deg": None,
        "optical_relative_pitch_deg": None,
        "optical_relative_roll_deg": None,
        "optical_relative_fit_inlier_count": None,
        "optical_relative_fit_residual_px": None,
        "optical_far_field_du_median_px": None,
        "pixhawk_relative_yaw_deg": None,
        "optical_minus_pixhawk_relative_yaw_deg": None,
    }
    payload = attempt_payload if isinstance(attempt_payload, Mapping) else {}
    pixhawk_relative = None
    try:
        left = payload.get("left_attitude") or {}
        right = payload.get("right_attitude") or {}
        if isinstance(left, Mapping) and isinstance(right, Mapping):
            left_yaw = left.get("yaw_deg")
            right_yaw = right.get("yaw_deg")
            if left_yaw is not None and right_yaw is not None:
                delta = float(right_yaw) - float(left_yaw)
                pixhawk_relative = (delta + 180.0) % 360.0 - 180.0
    except (TypeError, ValueError):
        pixhawk_relative = None
    fields["pixhawk_relative_yaw_deg"] = ResultProjectionService._row_scalar(
        pixhawk_relative
    )
    if not isinstance(accepted, Mapping):
        return fields
    rotation = accepted.get("relative_rotation")
    rotation = rotation if isinstance(rotation, Mapping) else {}
    accepted_ns = accepted.get("accepted_utc_ns")
    fields.update(
        {
            "optical_relative_source": "optical_pose_auto_suggestion_accepted",
            "optical_relative_mode": str(accepted.get("mode") or ""),
            "optical_relative_suggestion_id": str(accepted.get("suggestion_id") or ""),
            "optical_relative_accepted_utc_ns": (
                None if accepted_ns is None else int(accepted_ns)
            ),
            "optical_relative_age_s": (
                None
                if accepted_ns is None
                else ResultProjectionService._row_scalar(
                    (now_ns - int(accepted_ns)) / 1000000000.0
                )
            ),
        }
    )
    if not rotation.get("available"):
        return fields
    optical_yaw = rotation.get("yaw_right_minus_left_deg")
    fields.update(
        {
            "optical_relative_yaw_deg": ResultProjectionService._row_scalar(
                optical_yaw
            ),
            "optical_relative_pitch_deg": ResultProjectionService._row_scalar(
                rotation.get("pitch_right_minus_left_deg")
            ),
            "optical_relative_roll_deg": ResultProjectionService._row_scalar(
                rotation.get("roll_right_minus_left_deg")
            ),
            "optical_relative_fit_inlier_count": rotation.get("fit_inlier_count"),
            "optical_relative_fit_residual_px": ResultProjectionService._row_scalar(
                rotation.get("fit_residual_px")
            ),
            "optical_far_field_du_median_px": ResultProjectionService._row_scalar(
                rotation.get("far_field_du_median_px")
            ),
        }
    )
    if optical_yaw is not None and pixhawk_relative is not None:
        try:
            delta = float(optical_yaw) - float(pixhawk_relative)
            fields["optical_minus_pixhawk_relative_yaw_deg"] = (
                ResultProjectionService._row_scalar((delta + 180.0) % 360.0 - 180.0)
            )
        except (TypeError, ValueError):
            pass
    return fields
