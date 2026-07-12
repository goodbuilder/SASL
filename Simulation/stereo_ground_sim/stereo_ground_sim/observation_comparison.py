from __future__ import annotations

import math
from dataclasses import dataclass
from numbers import Integral

import numpy as np

from stereo_ground_sim.coordinate_frames import FrameId, FramePoint
from stereo_ground_sim.ideal_projection import project_nominal_stereo_point
from stereo_ground_sim.stereo_triangulation_adapter import (
    pinhole_intrinsics_from_horizontal_fov,
    triangulate_nominal_stereo_centers,
)
from stereo_ground_sim.triangulation_core import PinholeIntrinsics


OBSERVATION_SOURCE_IDEAL = "gazebo_target_reference_link_ideal_projection"


def _non_negative_int(value: object, *, field_name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, Integral):
        raise TypeError(f"{field_name} must be a non-negative integer")
    parsed = int(value)
    if parsed < 0:
        raise ValueError(f"{field_name} must be non-negative")
    return parsed


def _finite_float(value: object, *, field_name: str) -> float:
    if isinstance(value, (bool, str)):
        raise TypeError(f"{field_name} must be a finite float")
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{field_name} must be a finite float") from exc
    if not math.isfinite(parsed):
        raise ValueError(f"{field_name} must be finite")
    return parsed


def _non_negative_float(value: object, *, field_name: str) -> float:
    parsed = _finite_float(value, field_name=field_name)
    if parsed < 0.0:
        raise ValueError(f"{field_name} must be non-negative")
    return parsed


def _non_empty_string(value: object, *, field_name: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{field_name} must be a non-empty string")
    return value


def _point_error(
    estimate_m: tuple[float, float, float] | None,
    snapshot: "GroundTruthSnapshot",
) -> tuple[tuple[float, float, float] | None, float | None]:
    if estimate_m is None or not snapshot.oracle_valid or snapshot.point_baseline_center_frd is None:
        return None, None
    error = np.asarray(estimate_m, dtype=float) - np.asarray(snapshot.point_baseline_center_frd.xyz_m, dtype=float)
    return (
        (float(error[0]), float(error[1]), float(error[2])),
        float(np.linalg.norm(error)),
    )


def _pixel_bounds_status_from_flags(left_in_bounds: bool | None, right_in_bounds: bool | None) -> str:
    if left_in_bounds is None or right_in_bounds is None:
        return "not_evaluated"
    if left_in_bounds and right_in_bounds:
        return "all_in_bounds"
    if not left_in_bounds and not right_in_bounds:
        return "both_out_of_bounds"
    if not left_in_bounds:
        return "left_out_of_bounds"
    return "right_out_of_bounds"


@dataclass(frozen=True)
class GroundTruthSnapshot:
    """Immutable oracle sample in baseline_center_frd metres.

    `point_baseline_center_frd` uses components `(Forward, Right, Down)` in
    metres. Invalid current oracle samples carry no point/range and must not be
    replaced with last-valid data.
    """

    sequence: int
    received_monotonic_s: float
    point_baseline_center_frd: FramePoint | None
    range_m: float | None
    oracle_valid: bool
    oracle_status: str
    oracle_source_role: str | None
    oracle_matched_entity_name: str | None
    oracle_fallback_used: bool

    def __post_init__(self) -> None:
        object.__setattr__(self, "sequence", _non_negative_int(self.sequence, field_name="sequence"))
        object.__setattr__(
            self,
            "received_monotonic_s",
            _non_negative_float(self.received_monotonic_s, field_name="received_monotonic_s"),
        )
        object.__setattr__(self, "oracle_status", _non_empty_string(self.oracle_status, field_name="oracle_status"))
        if self.oracle_valid:
            if self.point_baseline_center_frd is None:
                raise ValueError("valid snapshot requires point_baseline_center_frd")
            if self.point_baseline_center_frd.frame != FrameId.BASELINE_CENTER_FRD:
                raise ValueError("point_baseline_center_frd must use baseline_center_frd")
            if self.range_m is None:
                raise ValueError("valid snapshot requires range_m")
            object.__setattr__(self, "range_m", _non_negative_float(self.range_m, field_name="range_m"))
        else:
            if self.point_baseline_center_frd is not None or self.range_m is not None:
                raise ValueError("invalid snapshot must not contain point or range")


@dataclass(frozen=True)
class PixelObservation:
    """Left/right pixel centers in pixels for one BBox observation."""

    left_u_px: float
    left_v_px: float
    right_u_px: float
    right_v_px: float
    observation_source: str
    received_monotonic_s: float

    def __post_init__(self) -> None:
        for name in ("left_u_px", "left_v_px", "right_u_px", "right_v_px"):
            object.__setattr__(self, name, _finite_float(getattr(self, name), field_name=name))
        object.__setattr__(
            self,
            "observation_source",
            _non_empty_string(self.observation_source, field_name="observation_source"),
        )
        object.__setattr__(
            self,
            "received_monotonic_s",
            _non_negative_float(self.received_monotonic_s, field_name="received_monotonic_s"),
        )


@dataclass(frozen=True)
class ObservationMeasurementResult:
    """Observation result in baseline_center_frd metres and pixels.

    Estimated points and error vectors use `(Forward, Right, Down)` in metres.
    Pixel fields use `(u, v)` in pixels. Ray gap is metres and intersection
    angle is radians. Unavailable invalid-result values are `None`.
    """

    valid: bool
    status: str
    observation_source: str
    oracle_sequence: int
    oracle_received_monotonic_s: float
    observation_received_monotonic_s: float | None
    left_u_px: float | None
    left_v_px: float | None
    right_u_px: float | None
    right_v_px: float | None
    signed_disparity_px: float | None
    vertical_disparity_px: float | None
    pixel_bounds_status: str
    estimated_point_baseline_center_frd_m: tuple[float, float, float] | None
    estimated_range_m: float | None
    error_vector_baseline_center_frd_m: tuple[float, float, float] | None
    error_norm_m: float | None
    ray_gap_m: float | None
    intersection_angle_rad: float | None
    left_ray_parameter_m: float | None
    right_ray_parameter_m: float | None


@dataclass(frozen=True)
class ParallelObservationComparison:
    """BBox-minus-Ideal residuals for same-oracle observation research.

    Pixel deltas are `(du, dv)` in pixels. Position deltas and identity residuals
    use baseline_center_frd `(Forward, Right, Down)` metres.
    """

    valid: bool
    status: str
    ideal: ObservationMeasurementResult
    bbox: ObservationMeasurementResult
    same_oracle_sequence: bool
    pixel_delta_left_px: tuple[float, float] | None
    pixel_delta_right_px: tuple[float, float] | None
    disparity_delta_px: float | None
    bbox_minus_ideal_position_baseline_center_frd_m: tuple[float, float, float] | None
    bbox_minus_ideal_norm_m: float | None
    error_identity_residual_m: float | None


def _measurement_from_triangulation(
    *,
    snapshot: GroundTruthSnapshot,
    observation_source: str,
    observation_received_monotonic_s: float | None,
    left_u_px: float | None,
    left_v_px: float | None,
    right_u_px: float | None,
    right_v_px: float | None,
    signed_disparity_px: float | None,
    vertical_disparity_px: float | None,
    pixel_bounds_status: str,
    triangulation,
    require_bounds: bool,
) -> ObservationMeasurementResult:
    result = None if triangulation is None else triangulation.triangulation_result
    point_m = None if result is None else result.point_m
    range_m = None if result is None else result.range_m
    error_vector, error_norm = _point_error(point_m, snapshot)

    valid_geometry = (
        pixel_bounds_status == "all_in_bounds" or not require_bounds
    ) and result is not None and bool(result.valid)
    if result is None:
        status = "triangulation_not_computed"
    elif pixel_bounds_status != "all_in_bounds" and require_bounds:
        status = "out_of_bounds"
    elif not result.valid:
        status = f"triangulation_invalid:{result.status.value}"
    elif not snapshot.oracle_valid:
        status = "ok_no_ground_truth"
    else:
        status = "ok"

    return ObservationMeasurementResult(
        valid=bool(valid_geometry),
        status=status,
        observation_source=observation_source,
        oracle_sequence=snapshot.sequence,
        oracle_received_monotonic_s=snapshot.received_monotonic_s,
        observation_received_monotonic_s=observation_received_monotonic_s,
        left_u_px=left_u_px,
        left_v_px=left_v_px,
        right_u_px=right_u_px,
        right_v_px=right_v_px,
        signed_disparity_px=signed_disparity_px,
        vertical_disparity_px=vertical_disparity_px,
        pixel_bounds_status=pixel_bounds_status,
        estimated_point_baseline_center_frd_m=point_m,
        estimated_range_m=range_m,
        error_vector_baseline_center_frd_m=error_vector,
        error_norm_m=error_norm,
        ray_gap_m=None if result is None else result.ray_gap_m,
        intersection_angle_rad=None if result is None else result.intersection_angle_rad,
        left_ray_parameter_m=None if result is None else result.left_ray_parameter_m,
        right_ray_parameter_m=None if result is None else result.right_ray_parameter_m,
    )


def ideal_measurement_from_snapshot(
    snapshot: GroundTruthSnapshot,
    *,
    intrinsics: PinholeIntrinsics,
    baseline_m: float,
    clip_near_m: float,
    clip_far_m: float,
) -> ObservationMeasurementResult:
    if not isinstance(snapshot, GroundTruthSnapshot):
        raise TypeError("snapshot must be a GroundTruthSnapshot")
    if not snapshot.oracle_valid or snapshot.point_baseline_center_frd is None:
        return ObservationMeasurementResult(
            valid=False,
            status="invalid_oracle_snapshot",
            observation_source=OBSERVATION_SOURCE_IDEAL,
            oracle_sequence=snapshot.sequence,
            oracle_received_monotonic_s=snapshot.received_monotonic_s,
            observation_received_monotonic_s=snapshot.received_monotonic_s,
            left_u_px=None,
            left_v_px=None,
            right_u_px=None,
            right_v_px=None,
            signed_disparity_px=None,
            vertical_disparity_px=None,
            pixel_bounds_status="not_evaluated",
            estimated_point_baseline_center_frd_m=None,
            estimated_range_m=None,
            error_vector_baseline_center_frd_m=None,
            error_norm_m=None,
            ray_gap_m=None,
            intersection_angle_rad=None,
            left_ray_parameter_m=None,
            right_ray_parameter_m=None,
        )

    projection = project_nominal_stereo_point(
        snapshot.point_baseline_center_frd,
        intrinsics,
        baseline_m,
        clip_near_m=clip_near_m,
        clip_far_m=clip_far_m,
    )
    pixel_bounds_status = _pixel_bounds_status_from_flags(projection.left.in_bounds, projection.right.in_bounds)
    if projection.left.u_px is None or projection.right.u_px is None:
        return ObservationMeasurementResult(
            valid=False,
            status=f"projection_invalid:{projection.status.value}",
            observation_source=OBSERVATION_SOURCE_IDEAL,
            oracle_sequence=snapshot.sequence,
            oracle_received_monotonic_s=snapshot.received_monotonic_s,
            observation_received_monotonic_s=snapshot.received_monotonic_s,
            left_u_px=projection.left.u_px,
            left_v_px=projection.left.v_px,
            right_u_px=projection.right.u_px,
            right_v_px=projection.right.v_px,
            signed_disparity_px=projection.signed_disparity_px,
            vertical_disparity_px=projection.vertical_disparity_px,
            pixel_bounds_status=pixel_bounds_status,
            estimated_point_baseline_center_frd_m=None,
            estimated_range_m=None,
            error_vector_baseline_center_frd_m=None,
            error_norm_m=None,
            ray_gap_m=None,
            intersection_angle_rad=None,
            left_ray_parameter_m=None,
            right_ray_parameter_m=None,
        )

    triangulation = triangulate_nominal_stereo_centers(
        left_cx_px=projection.left.u_px,
        left_cy_px=projection.left.v_px,
        right_cx_px=projection.right.u_px,
        right_cy_px=projection.right.v_px,
        width_px=intrinsics.width_px,
        height_px=intrinsics.height_px,
        horizontal_fov_rad=2.0 * math.atan((intrinsics.width_px * 0.5) / intrinsics.fx_px),
        baseline_m=baseline_m,
    )
    measurement = _measurement_from_triangulation(
        snapshot=snapshot,
        observation_source=OBSERVATION_SOURCE_IDEAL,
        observation_received_monotonic_s=snapshot.received_monotonic_s,
        left_u_px=projection.left.u_px,
        left_v_px=projection.left.v_px,
        right_u_px=projection.right.u_px,
        right_v_px=projection.right.v_px,
        signed_disparity_px=projection.signed_disparity_px,
        vertical_disparity_px=projection.vertical_disparity_px,
        pixel_bounds_status=pixel_bounds_status,
        triangulation=triangulation,
        require_bounds=True,
    )
    if measurement.status == "ok" and not projection.valid:
        return ObservationMeasurementResult(**{**measurement.__dict__, "valid": False, "status": f"projection_invalid:{projection.status.value}"})
    return measurement


def bbox_measurement_from_pixels(
    snapshot: GroundTruthSnapshot,
    pixels: PixelObservation,
    *,
    width_px: int,
    height_px: int,
    horizontal_fov_rad: float,
    baseline_m: float,
) -> ObservationMeasurementResult:
    if not isinstance(snapshot, GroundTruthSnapshot):
        raise TypeError("snapshot must be a GroundTruthSnapshot")
    if not isinstance(pixels, PixelObservation):
        raise TypeError("pixels must be a PixelObservation")
    triangulation = triangulate_nominal_stereo_centers(
        left_cx_px=pixels.left_u_px,
        left_cy_px=pixels.left_v_px,
        right_cx_px=pixels.right_u_px,
        right_cy_px=pixels.right_v_px,
        width_px=width_px,
        height_px=height_px,
        horizontal_fov_rad=horizontal_fov_rad,
        baseline_m=baseline_m,
    )
    return _measurement_from_triangulation(
        snapshot=snapshot,
        observation_source=pixels.observation_source,
        observation_received_monotonic_s=pixels.received_monotonic_s,
        left_u_px=pixels.left_u_px,
        left_v_px=pixels.left_v_px,
        right_u_px=pixels.right_u_px,
        right_v_px=pixels.right_v_px,
        signed_disparity_px=float(pixels.left_u_px - pixels.right_u_px),
        vertical_disparity_px=float(pixels.left_v_px - pixels.right_v_px),
        pixel_bounds_status=triangulation.pixel_bounds_status,
        triangulation=triangulation,
        require_bounds=True,
    )


def compare_observation_measurements(
    ideal: ObservationMeasurementResult,
    bbox: ObservationMeasurementResult,
) -> ParallelObservationComparison:
    if not isinstance(ideal, ObservationMeasurementResult):
        raise TypeError("ideal must be an ObservationMeasurementResult")
    if not isinstance(bbox, ObservationMeasurementResult):
        raise TypeError("bbox must be an ObservationMeasurementResult")
    same_sequence = ideal.oracle_sequence == bbox.oracle_sequence
    if not same_sequence:
        return ParallelObservationComparison(False, "different_oracle_sequence", ideal, bbox, False, None, None, None, None, None, None)
    if not ideal.valid:
        return ParallelObservationComparison(False, "ideal_invalid", ideal, bbox, True, None, None, None, None, None, None)
    if not bbox.valid:
        return ParallelObservationComparison(False, "bbox_invalid", ideal, bbox, True, None, None, None, None, None, None)
    required = (
        ideal.left_u_px,
        ideal.left_v_px,
        ideal.right_u_px,
        ideal.right_v_px,
        ideal.signed_disparity_px,
        bbox.left_u_px,
        bbox.left_v_px,
        bbox.right_u_px,
        bbox.right_v_px,
        bbox.signed_disparity_px,
        ideal.estimated_point_baseline_center_frd_m,
        bbox.estimated_point_baseline_center_frd_m,
        ideal.error_vector_baseline_center_frd_m,
        bbox.error_vector_baseline_center_frd_m,
    )
    if any(value is None for value in required):
        return ParallelObservationComparison(False, "comparison_unavailable", ideal, bbox, True, None, None, None, None, None, None)

    pixel_delta_left = (float(bbox.left_u_px - ideal.left_u_px), float(bbox.left_v_px - ideal.left_v_px))
    pixel_delta_right = (float(bbox.right_u_px - ideal.right_u_px), float(bbox.right_v_px - ideal.right_v_px))
    disparity_delta = float(bbox.signed_disparity_px - ideal.signed_disparity_px)
    position_delta = np.asarray(bbox.estimated_point_baseline_center_frd_m, dtype=float) - np.asarray(
        ideal.estimated_point_baseline_center_frd_m,
        dtype=float,
    )
    e_i = np.asarray(ideal.error_vector_baseline_center_frd_m, dtype=float)
    e_b = np.asarray(bbox.error_vector_baseline_center_frd_m, dtype=float)
    identity_residual = e_b - (e_i + position_delta)
    return ParallelObservationComparison(
        valid=True,
        status="ok",
        ideal=ideal,
        bbox=bbox,
        same_oracle_sequence=True,
        pixel_delta_left_px=pixel_delta_left,
        pixel_delta_right_px=pixel_delta_right,
        disparity_delta_px=disparity_delta,
        bbox_minus_ideal_position_baseline_center_frd_m=(
            float(position_delta[0]),
            float(position_delta[1]),
            float(position_delta[2]),
        ),
        bbox_minus_ideal_norm_m=float(np.linalg.norm(position_delta)),
        error_identity_residual_m=float(np.linalg.norm(identity_residual)),
    )


__all__ = [
    "GroundTruthSnapshot",
    "ObservationMeasurementResult",
    "OBSERVATION_SOURCE_IDEAL",
    "ParallelObservationComparison",
    "PixelObservation",
    "bbox_measurement_from_pixels",
    "compare_observation_measurements",
    "ideal_measurement_from_snapshot",
]
