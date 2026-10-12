"""Drone RTK pose candidates from explicit captured/imported observations.

Camera rays use optical RDF. Centers/target positions and LOS use one
local NED origin in metres; timestamps remain evidence, never ground truth.
The adapter owns live telemetry lookup, file reads, and candidate registration.
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
    PoseParametersNed,
    estimate_drone_yaw_deg,
    pose_parameters_from_camera_pose,
)
from Workstation.workstation_core.pose_solvers import (
    fixed_center_orientation_ray_residuals,
    pose_target_ray_residuals,
    solve_constrained_pose_least_squares,
)
from Workstation.workstation_core.rtk_geometry import GpsPositionState, ned_delta_m
from .geometry_values import _finite_vector, _mask_observability, _payload_sha256, _subtract, _vector_value
from .observations import coerce_gps_state, validate_annotation_identity, annotation_points, undistort_pixels
from .policy import _ALL_DOF, _DRONE_ROTATION_DOF, _YAW_DOF, _DRONE_RTK_MULTIVIEW_MIN_OBSERVATIONS


def solve_drone_multiview_candidate(
    *, profile, priors, root, source_file_sha256: str, live_geometry
) -> OpticalPoseCandidate:
    observation_root = root.get("observations", root)
    initial_centers = {
        "left": tuple(priors[0].center_ned_m),
        "right": tuple(priors[1].center_ned_m),
    }
    if live_geometry is not None:
        initial_centers = {
            "left": (0.0, 0.0, 0.0),
            "right": tuple(live_geometry["baseline_ned"]),
        }
    center_root = root.get("camera_centers_ned_m")
    if isinstance(center_root, Mapping):
        for side in ("left", "right"):
            if side in center_root:
                initial_centers[side] = _vector_value(
                    center_root[side],
                    name=f"camera_centers_ned_m.{side}",
                )

    origin_state: GpsPositionState | None = None
    if live_geometry is not None:
        origin_state = live_geometry["states"]["left"]
    elif root.get("origin_gps") is not None:
        origin_state = coerce_gps_state(
            root.get("origin_gps"),
            side="origin",
            now_s=time.monotonic(),
        )

    solved: dict[str, tuple[CameraPoseNed, Any, tuple[dict[str, Any], ...]]] = {}
    source_ids: list[str] = []
    input_hashes: list[tuple[str, str]] = [
        ("operating_intrinsics_profile_sha256", profile.profile_sha256),
    ]
    if source_file_sha256:
        input_hashes.append(("drone_rtk_multiview_json_sha256", source_file_sha256))
    collection_sha256 = str(root.get("collection_sha256") or "")
    if collection_sha256:
        input_hashes.append(("drone_rtk_multiview_collection_sha256", collection_sha256))
    warning_set: list[str] = [
        "Drone RTK target positions and camera rays are solved jointly for full R,t",
        "camera pose coordinate contract: X_NED = R_NED<-optical_RDF X_RDF + C_NED",
    ]
    if str(root.get("source") or "") == "session_accumulated_frozen_targets":
        warning_set.extend(
            (
                "input source: in-session Frozen target pairs with capture-associated Drone RTK",
                "local NED origin is the first READY sample's P1 GNSS position",
                "P1/P2 GNSS antenna positions initialize camera centers; optical lever arms remain unmodeled",
            )
        )
    for side, prior, camera in (
        ("left", priors[0], profile.left),
        ("right", priors[1], profile.right),
    ):
        raw_rows = observation_root.get(side)
        if not isinstance(raw_rows, (tuple, list)):
            raise ValueError(f"Drone RTK multi-view requires a {side} observation array")
        if len(raw_rows) < _DRONE_RTK_MULTIVIEW_MIN_OBSERVATIONS:
            raise ValueError(f"{side}: Drone RTK multi-view requires at least four observations")
        rays: list[tuple[float, float, float]] = []
        targets: list[tuple[float, float, float]] = []
        rows_for_hash: list[dict[str, Any]] = []
        side_source_ids: list[str] = []
        for index, raw_row in enumerate(raw_rows):
            if not isinstance(raw_row, Mapping):
                raise ValueError(f"{side}[{index}] must be an object")
            row = dict(raw_row)
            ray_value = row.get("observed_ray_camera_rdf")
            if ray_value is None:
                pixel = row.get("pixel_uv", row.get("image_point_px", row.get("target_pixel")))
                if isinstance(pixel, Mapping):
                    pixel = (pixel.get("u_px", pixel.get("u")), pixel.get("v_px", pixel.get("v")))
                pixel_uv = _finite_vector(pixel or (), length=2, name=f"{side}[{index}].pixel_uv")
                normalized = undistort_pixels(camera, (tuple(pixel_uv),))[0]
                ray_value = (normalized[0], normalized[1], 1.0)
            ray = _finite_vector(
                ray_value,
                length=3,
                name=f"{side}[{index}].observed_ray_camera_rdf",
            )
            target_value = row.get("target_point_ned_m", row.get("drone_position_ned_m"))
            if target_value is None:
                target_gps = row.get("drone_gps", row.get("target_gps"))
                if target_gps is None or origin_state is None:
                    raise ValueError(f"{side}[{index}] requires target_point_ned_m or Drone RTK GPS plus origin_gps")
                target_state = coerce_gps_state(
                    target_gps,
                    side="drone",
                    now_s=time.monotonic(),
                )
                target_value = None if target_state is None else ned_delta_m(origin_state, target_state)
            target = _vector_value(target_value, name=f"{side}[{index}].target_point_ned_m")
            rays.append(ray)
            targets.append(target)
            observation_id = str(row.get("observation_id") or row.get("frame_id") or f"{side}-{index:04d}")
            side_source_ids.append(f"drone-rtk:{side}:{observation_id}")
            if row.get("timestamp_alignment_verified") is not True:
                warning_set.append(f"{side}[{index}]: image/Drone RTK timestamp alignment is not verified")
            rows_for_hash.append(row)
        before_pose = CameraPoseNed(
            prior.rotation_ned_from_camera_rdf,
            initial_centers[side],
        )
        before = pose_parameters_from_camera_pose(before_pose)

        def _residuals(
            parameters: PoseParametersNed,
            *,
            _rays=tuple(rays),
            _targets=tuple(targets),
        ) -> tuple[float, ...]:
            return pose_target_ray_residuals(
                parameters,
                observed_rays_camera_rdf=_rays,
                target_points_ned_m=_targets,
            )

        result = solve_constrained_pose_least_squares(
            before,
            _ALL_DOF,
            _residuals,
            parameter_scales=(1.0, 1.0, 1.0, 1.0, 1.0, 1.0),
        )
        if not result.success:
            raise ValueError(f"{side}: Drone RTK multi-view R,t solve did not converge ({result.message})")
        if not result.jacobian_diagnostics.full_rank:
            raise ValueError(
                f"{side}: Drone RTK multi-view observations are geometrically degenerate; "
                "use targets with varied 3-D positions"
            )
        solved[side] = (result.parameters.to_camera_pose_ned(), result, tuple(rows_for_hash))
        source_ids.extend(side_source_ids)
        input_hashes.append((f"{side}_drone_rtk_multiview_sha256", _payload_sha256({"observations": rows_for_hash})))

    left = solved["left"][0]
    right = solved["right"][0]
    quality: list[tuple[str, float]] = []
    uncertainty: list[tuple[str, float]] = []
    for side, (_pose, result, rows) in solved.items():
        residuals = tuple(float(value) for value in result.residuals)
        rms = math.sqrt(sum(value * value for value in residuals) / len(residuals))
        quality.extend(
            (
                (f"{side}_observation_count", float(len(rows))),
                (f"{side}_angular_rms_chord", rms),
            )
        )
        condition = float(result.jacobian_diagnostics.condition_number)
        if math.isfinite(condition):
            uncertainty.append((f"{side}_jacobian_condition", condition))
    input_hashes.append(
        (
            "multiview_coordinate_contract_sha256",
            _payload_sha256(
                {
                    "camera_origin": "local NED origin from imported origin_gps/current left GNSS or prior",
                    "target": "Drone RTK position in same local NED frame",
                    "ray": "camera optical RDF",
                }
            ),
        )
    )
    candidate = OpticalPoseCandidate(
        candidate_id=f"drone-rtk-multiview-{uuid.uuid4().hex}",
        method="drone-rtk-multiview-r-t",
        left_pose_ned=left,
        right_pose_ned=right,
        optimized_dofs=_ALL_DOF,
        observability=_mask_observability(_ALL_DOF),
        source_observation_ids=tuple(source_ids),
        input_hashes=tuple(input_hashes),
        quality_metrics=tuple(quality),
        uncertainty_metrics=tuple(uncertainty),
        warnings=tuple(dict.fromkeys(warning_set)),
        created_utc_ns=time.time_ns(),
    )
    return candidate


def solve_drone_yaw_candidate(
    *, profile, priors, root, drone_file_sha256: str, annotations, artifacts
) -> tuple[OpticalPoseCandidate, dict[str, dict[str, Any]]]:
    solved: dict[str, tuple[CameraPoseNed, PoseDofMask, tuple[tuple[str, float], ...]]] = {}
    observation_counts: dict[str, int] = {}
    warnings: list[str] = []
    rotation_context: dict[str, dict[str, Any]] = {}
    hashes: list[tuple[str, str]] = [("operating_intrinsics_profile_sha256", profile.profile_sha256)]
    if drone_file_sha256:
        hashes.append(("drone_yaw_json_sha256", drone_file_sha256))
    for side, prior, camera in (
        ("left", priors[0], profile.left),
        ("right", priors[1], profile.right),
    ):
        raw_row = root.get(side)
        if isinstance(raw_row, Mapping):
            observation_rows: tuple[Mapping[str, Any], ...] = (raw_row,)
        elif isinstance(raw_row, (list, tuple)):
            observation_rows = tuple(item for item in raw_row if isinstance(item, Mapping))
        else:
            observation_rows = ()
        if not observation_rows:
            continue
        observation_counts[side] = len(observation_rows)
        annotation = annotations.get(side)
        before = pose_parameters_from_camera_pose(prior)

        if len(observation_rows) == 1:
            # A single RTK target/ray pair only constrains yaw (roll/pitch
            # held at the prior); use the existing closed-form solve.
            row = observation_rows[0]
            ray_value = row.get("observed_ray_camera_rdf")
            if ray_value is None:
                if not isinstance(annotation, Mapping):
                    raise ValueError(f"{side}: provide observed_ray_camera_rdf or a frozen target_point annotation")
                validate_annotation_identity(artifacts.get(side), side, annotation)
                points = annotation_points(annotation, "target_point")
                if len(points) != 1:
                    raise ValueError(f"{side}: exactly one frozen target pixel is required")
                normalized = undistort_pixels(camera, points)[0]
                ray = (normalized[0], normalized[1], 1.0)
                hashes.append((f"{side}_annotation_sha256", _payload_sha256(annotation)))
            else:
                ray = _finite_vector(ray_value, length=3, name=f"{side}.observed_ray_camera_rdf")
            los_value = row.get("expected_los_ned")
            if los_value is None:
                camera_center = _finite_vector(
                    row.get("camera_center_ned_m", ()),
                    length=3,
                    name=f"{side}.camera_center_ned_m",
                )
                target_point = _finite_vector(
                    row.get("target_point_ned_m", ()),
                    length=3,
                    name=f"{side}.target_point_ned_m",
                )
                los = _subtract(target_point, camera_center)
            else:
                los = _finite_vector(los_value, length=3, name=f"{side}.expected_los_ned")
            estimate = estimate_drone_yaw_deg(
                ray,
                los,
                roll_deg=before.roll_deg,
                pitch_deg=before.pitch_deg,
            )
            if not estimate.observable or estimate.yaw_deg is None:
                raise ValueError(f"{side}: Drone RTK yaw is unobservable ({estimate.reason})")
            updated = PoseParametersNed(
                before.roll_deg,
                before.pitch_deg,
                estimate.yaw_deg,
                before.north_m,
                before.east_m,
                before.down_m,
            ).to_camera_pose_ned()
            solved[side] = (
                updated,
                _YAW_DOF,
                ((f"{side}_angular_residual_rad", float(estimate.angular_residual_rad)),),
            )
            rotation_context[side] = {
                "mode": "yaw_only",
                "observed_ray_camera_rdf": tuple(float(value) for value in ray),
                "expected_los_ned": tuple(float(value) for value in los),
                "timestamp_alignment_verified": row.get("timestamp_alignment_verified") is True,
            }
            hashes.append((f"{side}_drone_input_sha256", _payload_sha256(row)))
            if row.get("timestamp_alignment_verified") is not True:
                warnings.append(f"{side}: camera pixel and target/camera RTK timestamp alignment is not verified")
            continue

        # Two or more RTK target/ray pairs over-determine the full
        # rotation (roll, pitch, yaw) with the optical center still fixed
        # at the prior; position is never inferred from this solve.
        rays: list[tuple[float, float, float]] = []
        targets: list[tuple[float, float, float]] = []
        for index, row in enumerate(observation_rows):
            ray_value = row.get("observed_ray_camera_rdf")
            if ray_value is None:
                raise ValueError(
                    f"{side}[{index}]: multi-observation Drone yaw requires "
                    "observed_ray_camera_rdf explicitly per observation; frozen "
                    "annotations only support a single observation"
                )
            rays.append(_finite_vector(ray_value, length=3, name=f"{side}[{index}].observed_ray_camera_rdf"))
            los_value = row.get("expected_los_ned")
            if los_value is None:
                target_point = _finite_vector(
                    row.get("target_point_ned_m", ()),
                    length=3,
                    name=f"{side}[{index}].target_point_ned_m",
                )
            else:
                los = _finite_vector(los_value, length=3, name=f"{side}[{index}].expected_los_ned")
                target_point = tuple(prior.center_ned_m[axis] + los[axis] for axis in range(3))
            targets.append(target_point)
            if row.get("timestamp_alignment_verified") is not True:
                warnings.append(
                    f"{side}[{index}]: camera pixel and target/camera RTK timestamp alignment is not verified"
                )
        hashes.append(
            (
                f"{side}_drone_multi_input_sha256",
                _payload_sha256({"observations": [dict(row) for row in observation_rows]}),
            )
        )

        def _residuals(
            parameters: PoseParametersNed,
            *,
            _center=prior.center_ned_m,
            _rays=tuple(rays),
            _targets=tuple(targets),
        ) -> tuple[float, ...]:
            return fixed_center_orientation_ray_residuals(
                (parameters.roll_deg, parameters.pitch_deg, parameters.yaw_deg),
                camera_center_ned_m=_center,
                observed_rays_camera_rdf=_rays,
                target_points_ned_m=_targets,
            )

        result = solve_constrained_pose_least_squares(before, _DRONE_ROTATION_DOF, _residuals)
        if not result.success:
            raise ValueError(f"{side}: Drone RTK multi-observation rotation solve did not converge ({result.message})")
        updated = result.parameters.to_camera_pose_ned()
        rms_chord_residual = math.sqrt(sum(value * value for value in result.residuals) / len(result.residuals))
        solved[side] = (
            updated,
            _DRONE_ROTATION_DOF,
            (
                (f"{side}_rotation_rms_chord_residual", rms_chord_residual),
                (f"{side}_observation_count", float(len(observation_rows))),
            ),
        )
        rotation_context[side] = {
            "mode": "full_rotation",
            "observation_count": len(observation_rows),
        }
    if not solved:
        raise ValueError("Drone yaw requires explicit per-side target/camera LOS and ray or frozen target pixel")
    left = solved.get("left", (priors[0],))[0]
    right = solved.get("right", (priors[1],))[0]
    quality = tuple(item for (_pose, _mask, metrics) in solved.values() for item in metrics)
    combined_mask = (
        _DRONE_ROTATION_DOF
        if any(mask is _DRONE_ROTATION_DOF for _pose, mask, _metrics in solved.values())
        else _YAW_DOF
    )
    candidate = OpticalPoseCandidate(
        candidate_id=f"drone-yaw-{uuid.uuid4().hex}",
        method="drone-rtk-target-yaw",
        left_pose_ned=left,
        right_pose_ned=right,
        optimized_dofs=combined_mask,
        observability=_mask_observability(combined_mask),
        source_observation_ids=tuple(f"{side}:explicit-drone-rtk-los×{observation_counts[side]}" for side in solved),
        input_hashes=tuple(hashes),
        quality_metrics=quality,
        warnings=tuple(warnings),
        created_utc_ns=time.time_ns(),
        observability_by_side=tuple(
            (
                side,
                _mask_observability(solved[side][1] if side in solved else PoseDofMask.from_names(())),
            )
            for side in ("left", "right")
        ),
    )
    return candidate, rotation_context
