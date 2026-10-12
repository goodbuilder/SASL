"""Mountain/Cloud candidates from captured native pixels and GNSS evidence.

Inputs are native u/v pixels, optical RDF rays, and right-minus-left NED
baseline metres. No live panel, GPS lookup, device or file access occurs.
The left capture reference remains the absolute anchor; no truth is injected.
"""

from __future__ import annotations
import math
import time
import uuid
from typing import Any, Mapping
from Workstation.workstation_core.optical_pose import (
    CameraPoseNed,
    OpticalPoseCandidate,
    PoseDofMask,
    stereo_pose_from_absolute,
)
from Workstation.workstation_core.optical_pose_auto_select import validate_mountain_point_distribution
from Workstation.workstation_core.optical_yaw90_reference import canonical_evidence
from Workstation.workstation_core.pose_solvers import MountainRotationConfig, solve_mountain_relative_rotation_robust
from .geometry_values import (
    _finite_vector,
    _matvec,
    _mat_transpose,
    _matmul,
    _pose_mapping,
    _payload_sha256,
    _mask_observability,
)
from .observations import validate_annotation_identity, annotation_points, undistort_pixels
from .policy import _ALL_DOF, _ROTATION_DOF, CLOUD_FRAME_PAIR_MAX_DELTA_S, _MOUNTAIN_MINIMUM_GNSS_FIX_TYPE


def resolve_mountain_lever_arm(
    payload: Mapping[str, Any],
) -> tuple[tuple[float, float, float], str]:
    """Resolve an explicit optical-center correction, never an estimate.

    The correction is the vector to *add* to the antenna-to-antenna GNSS
    baseline (already expressed in Left camera RDF) to obtain the
    optical-center-to-optical-center baseline. This module never
    estimates it: without an explicit
    ``lever_arm_baseline_correction_left_rdf_m`` payload override the
    status is ``UNMODELED`` and the correction is the zero vector, which
    must be surfaced to the caller as an assumption, not a measurement.
    """

    raw = payload.get("lever_arm_baseline_correction_left_rdf_m")
    if raw is None:
        return (0.0, 0.0, 0.0), "UNMODELED"
    correction = _finite_vector(raw, length=3, name="lever_arm_baseline_correction_left_rdf_m")
    return correction, "CALIBRATED"


def solve_mountain_candidate(
    payload: Mapping[str, Any], *, profile, priors, reference, annotations, artifacts, rotation_only: bool, simultaneity
) -> OpticalPoseCandidate:
    points_by_side: dict[str, tuple[tuple[float, float], ...]] = {}
    input_hashes: list[tuple[str, str]] = [("operating_intrinsics_profile_sha256", profile.profile_sha256)]
    source_ids: list[str] = []
    for side in ("left", "right"):
        annotation = annotations.get(side)
        if not isinstance(annotation, Mapping):
            continue
        validate_annotation_identity(artifacts.get(side), side, annotation)
        points = annotation_points(annotation, "ridge_polyline")
        if points:
            points_by_side[side] = points
            source_ids.append(str(annotation["frame_id"]))
            input_hashes.append((f"{side}_mountain_uv_sha256", _payload_sha256(annotation)))
    if set(points_by_side) != {"left", "right"}:
        raise ValueError("GNSS + Mountain requires frozen left/right Mountain u,v clicks")
    left_points = points_by_side["left"]
    right_points = points_by_side["right"]
    if len(left_points) != len(right_points):
        raise ValueError(
            "GNSS + Mountain requires the same number of left/right clicks; "
            "click corresponding mountain features in the same order"
        )
    if len(left_points) < 4:
        raise ValueError("GNSS + Mountain requires at least four left/right u,v correspondence pairs")
    validate_mountain_point_distribution(left_points, side="left")
    validate_mountain_point_distribution(right_points, side="right")

    baseline_ned = tuple(reference["baseline_ned_m"])
    gnss_details = {
        "source": "yaw90_capture_reference",
        "reference_sha256": reference["sha256"],
        "ned_m": list(baseline_ned),
        "baseline_m": math.sqrt(sum(v * v for v in baseline_ned)),
        "apply_eligible": True,
        "quality_warnings": reference["warnings"],
        "fix_quality": "RTK_FIXED"
        if all((reference["gps"][s] or {}).get("fix_type") == 6 for s in ("left", "right"))
        else "DEGRADED",
        **{
            f"{s}_{k}": (reference["gps"][s] or {}).get(k) for s in ("left", "right") for k in ("fix_type", "rtk_label")
        },
    }
    baseline_left_rdf = _matvec(
        _mat_transpose(priors[0].rotation_ned_from_camera_rdf),
        baseline_ned,
    )
    lever_arm_correction, lever_arm_status = resolve_mountain_lever_arm(payload)
    optical_baseline_left_rdf = tuple(baseline_left_rdf[index] + lever_arm_correction[index] for index in range(3))
    left_bearings = tuple((*normalized, 1.0) for normalized in undistort_pixels(profile.left, left_points))
    right_bearings = tuple((*normalized, 1.0) for normalized in undistort_pixels(profile.right, right_points))
    prior_stereo = stereo_pose_from_absolute(*priors)
    left = priors[0]
    left_anchor_sha256 = _payload_sha256(_pose_mapping(left))
    rotation_config = MountainRotationConfig(quality_advisory=True)
    try:
        solve = solve_mountain_relative_rotation_robust(
            left_bearings_rdf=left_bearings,
            right_bearings_rdf=right_bearings,
            baseline_direction_left_rdf=optical_baseline_left_rdf,
            prior_rotation_right_from_left_rdf=prior_stereo.rotation_right_from_left_rdf,
            config=rotation_config,
        )
    except ValueError as exc:
        raise ValueError(f"GNSS + Mountain rotation solve failed: {exc}") from exc

    right_rotation_ned = _matmul(
        left.rotation_ned_from_camera_rdf,
        _mat_transpose(solve.rotation_right_from_left_rdf),
    )
    optical_baseline_ned = _matvec(
        left.rotation_ned_from_camera_rdf,
        optical_baseline_left_rdf,
    )
    right = CameraPoseNed(
        right_rotation_ned,
        tuple(left.center_ned_m[index] + optical_baseline_ned[index] for index in range(3)),
    )

    focal_scale = (
        math.sqrt(
            float(profile.left.camera_matrix[0][0])
            * float(profile.left.camera_matrix[1][1])
            * float(profile.right.camera_matrix[0][0])
            * float(profile.right.camera_matrix[1][1])
        )
        ** 0.25
    )
    rotation_apply_eligible = bool(solve.apply_eligible)
    gnss_apply_eligible = bool(gnss_details.get("apply_eligible"))
    sync_verified: bool | None = None
    sync_delta_s: float | None = None
    sync_reason = ""
    if rotation_only:
        gnss_apply_eligible = False
        sync_verified, sync_delta_s, sync_reason = simultaneity
        if sync_verified is False:
            rotation_apply_eligible = False
    observable = PoseDofMask(
        roll_deg=rotation_apply_eligible,
        pitch_deg=rotation_apply_eligible,
        yaw_deg=rotation_apply_eligible,
        north_m=gnss_apply_eligible,
        east_m=gnss_apply_eligible,
        down_m=gnss_apply_eligible,
    )
    uv_hash = _payload_sha256(
        {
            "left": [{"u_px": point[0], "v_px": point[1]} for point in left_points],
            "right": [{"u_px": point[0], "v_px": point[1]} for point in right_points],
            "contract": "same-order paired frozen native OpenCV pixels; u right, v down",
        }
    )
    input_hashes.extend(
        (
            ("mountain_uv_pairs_sha256", uv_hash),
            ("gnss_baseline_sha256", _payload_sha256(gnss_details)),
            ("left_anchor_pose_sha256", left_anchor_sha256),
            ("lever_arm_status", lever_arm_status),
            ("pose_component_sources.left_rotation", "capture_gnss_yaw90_p1_roll_pitch_assumed"),
            ("pose_component_sources.left_position", "capture_P1_local_origin_assumed"),
            (
                "pose_component_sources.relative_rotation",
                "cloud_far_field_correspondences" if rotation_only else "mountain_correspondences",
            ),
            (
                "pose_component_sources.relative_translation",
                "not_applied_rotation_only" if rotation_only else "capture_gnss_baseline",
            ),
            ("pose_component_sources.right_rotation", "derived"),
            ("pose_component_sources.right_position", "derived"),
            ("pose_component_sources.lever_arm", lever_arm_status),
        )
    )
    if rotation_only:
        input_hashes.append(
            (
                "frame_pair_simultaneity",
                "VERIFIED" if sync_verified else "EXCEEDED" if sync_verified is False else "UNVERIFIED",
            )
        )
    warnings: list[str] = [
        (
            "Cloud candidate is rotation-only: translation is never applied; the Cloud "
            "measurement method fills N/E/D from live GNSS"
            if rotation_only
            else "GNSS is authoritative for right-minus-left metric translation in world NED"
        ),
        "Mountain uses only same-order left/right frozen u,v clicks; no GeoTIFF or terrain height",
        "relative R is conditioned on the selected Left absolute anchor; changing the anchor "
        "changes/invalidates this candidate even for identical pixels and GNSS baseline",
        "Left absolute pose is not measured by Mountain; it remains the prior/anchor",
        f"GNSS fix quality={gnss_details['fix_quality']}",
        (
            "lever arm CALIBRATED: an explicit optical-center correction was applied"
            if lever_arm_status == "CALIBRATED"
            else "lever arm UNMODELED: GNSS antenna baseline is used as an optical-baseline "
            "proxy with zero correction; this is an assumption, not a measurement"
        ),
    ]
    warnings.extend(solve.quality_warnings)
    warnings.extend(gnss_details.get("quality_warnings", ()))
    warnings.append("gate_policy=mountain-cloud-availability-v1; quality thresholds are advisory")
    input_hashes.append(("gate_policy", "mountain-cloud-availability-v1"))
    input_hashes.extend(
        (("yaw90_reference_sha256", reference["sha256"]), ("reference_source", "capture_gnss_yaw90_p1_roll_pitch"))
    )
    warnings.extend(reference["assumptions"])
    if rotation_only:
        warnings.append(
            "correspondences are far-field sky/cloud texture with no ridge bias; the "
            "GNSS baseline direction only conditions the rotation fit"
        )
        if sync_verified is True:
            warnings.append(
                f"frame pair simultaneity VERIFIED: |right-left| = {sync_delta_s:.3f}s "
                f"<= {CLOUD_FRAME_PAIR_MAX_DELTA_S:.3f}s"
            )
        elif sync_verified is False:
            warnings.append(f"frame pair simultaneity EXCEEDED: {sync_reason}; rotation not apply-eligible")
        else:
            warnings.append(f"frame pair simultaneity UNVERIFIED: {sync_reason}")
    elif not gnss_apply_eligible:
        warnings.append(
            "GNSS + Mountain translation requires fix_type >= 3 (3D fix or better) on "
            "both Left and Right; "
            f"left_fix_type={gnss_details.get('left_fix_type')} "
            f"right_fix_type={gnss_details.get('right_fix_type')}"
        )
    elif not (
        str(gnss_details.get("left_rtk_label") or "").upper() == "RTK_FIXED"
        and str(gnss_details.get("right_rtk_label") or "").upper() == "RTK_FIXED"
    ):
        warnings.append(
            "GNSS + Mountain translation accepted from finite capture positions without "
            "RTK_FIXED on both sides; the Mountain measurement remains ESTIMATED; "
            f"left={gnss_details.get('left_rtk_label')} "
            f"right={gnss_details.get('right_rtk_label')}"
        )
    if not rotation_apply_eligible:
        reasons = list(solve.ineligibility_reasons)
        if rotation_only and sync_verified is False:
            reasons.append(sync_reason)
        warnings.append(
            ("Cloud" if rotation_only else "Mountain") + " rotation is not apply-eligible: " + "; ".join(reasons)
        )
    uncertainty: list[tuple[str, float]] = []
    if math.isfinite(solve.jacobian_condition):
        uncertainty.append(("rotation_jacobian_condition", float(solve.jacobian_condition)))
    quality: list[tuple[str, float]] = [
        ("mountain_uv_pair_count", float(len(left_points))),
        ("inlier_count", float(len(solve.inlier_indices))),
        ("outlier_count", float(len(solve.outlier_indices))),
        (
            "inlier_ratio",
            float(len(solve.inlier_indices)) / float(solve.total_count) if solve.total_count else 0.0,
        ),
        ("gnss_baseline_m", float(gnss_details["baseline_m"])),
        ("gnss_minimum_fix_type", float(_MOUNTAIN_MINIMUM_GNSS_FIX_TYPE)),
    ]
    if rotation_only and sync_delta_s is not None:
        quality.append(("frame_pair_delta_s", float(sync_delta_s)))
    for side in ("left", "right"):
        fix_type = gnss_details.get(f"{side}_fix_type")
        if fix_type is not None:
            quality.append((f"gnss_{side}_fix_type", float(fix_type)))
    if math.isfinite(solve.sampson_rms_normalized) and math.isfinite(solve.sampson_p95_normalized):
        quality.extend(
            (
                ("sampson_rms_normalized", float(solve.sampson_rms_normalized)),
                ("sampson_p95_normalized", float(solve.sampson_p95_normalized)),
                ("sampson_rms_px_equivalent", float(solve.sampson_rms_normalized) * focal_scale),
                ("sampson_p95_px_equivalent", float(solve.sampson_p95_normalized) * focal_scale),
            )
        )
    else:
        warnings.append("Sampson residual metrics are unavailable (non-finite; too few inliers)")
    candidate = OpticalPoseCandidate(
        candidate_id=(
            f"cloud-rotation-{uuid.uuid4().hex}" if rotation_only else f"gnss-mountain-uv-{uuid.uuid4().hex}"
        ),
        method="cloud-far-field-rotation" if rotation_only else "gnss-constrained-mountain-uv",
        reference_evidence_json=canonical_evidence(reference),
        left_pose_ned=left,
        right_pose_ned=right,
        optimized_dofs=_ROTATION_DOF if rotation_only else _ALL_DOF,
        observability=_mask_observability(observable),
        source_observation_ids=tuple(dict.fromkeys((*source_ids, f"gnss-baseline:{_payload_sha256(gnss_details)}"))),
        input_hashes=tuple(input_hashes),
        quality_metrics=tuple(quality),
        uncertainty_metrics=tuple(uncertainty),
        warnings=tuple(warnings),
        created_utc_ns=time.time_ns(),
        observability_by_side=tuple(
            (
                side,
                _mask_observability(observable),
            )
            for side in ("left", "right")
        ),
    )
    return candidate
