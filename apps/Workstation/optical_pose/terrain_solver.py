"""Terrain PnP candidate from explicit frozen pixel and NED correspondences.

The owning boundary validates/imports the GeoTIFF and reads correspondence
files. This computation does no file or UI access, and never substitutes
GNSS ellipsoid altitude for an explicitly declared terrain vertical datum.
"""

from __future__ import annotations

import math
import time
import uuid
from typing import Any, Mapping, TYPE_CHECKING
from Workstation.workstation_core.operating_intrinsics import OperatingIntrinsicsProfile
from Workstation.workstation_core.terrain import TerrainMetadata

if TYPE_CHECKING:
    from Workstation.workstation_core.native_frame_store import NativeFrameArtifact
from Workstation.workstation_core.optical_pose import CameraPoseNed, OpticalPoseCandidate, PoseDofMask
from Workstation.workstation_core.pose_solvers import PnpCorrespondences, solve_pnp_ensemble
from .geometry_values import _mask_observability, _payload_sha256
from .observations import annotation_points, validate_annotation_identity
from .policy import _ALL_DOF


def solve_terrain_candidate(
    *,
    profile: OperatingIntrinsicsProfile,
    priors: tuple[CameraPoseNed, CameraPoseNed],
    annotations: Mapping[str, Mapping[str, Any]],
    artifacts: Mapping[str, NativeFrameArtifact],
    correspondence_root: Mapping[str, Any],
    correspondence_file_sha256: str,
    terrain_metadata: TerrainMetadata,
) -> OpticalPoseCandidate:
    """Solve supplied frozen correspondences; adapter validates GeoTIFF provenance.

    The adapter must reject unknown vertical datum before admitting this call.
    No source path is opened here, even when metadata retains provenance.
    """
    solved: dict[str, Any] = {}
    failures: list[str] = []
    for side, camera in (("left", profile.left), ("right", profile.right)):
        annotation = annotations.get(side)
        row = correspondence_root.get(side)
        if not isinstance(annotation, Mapping) or not isinstance(row, Mapping):
            continue
        validate_annotation_identity(artifacts.get(side), side, annotation)
        image_points = annotation_points(annotation, "ridge_polyline")
        object_points = row.get("object_points_ned_m")
        if not isinstance(object_points, (tuple, list)):
            failures.append(f"{side}: provide object_points_ned_m matched one-to-one with frozen ridge pixels")
            continue
        if len(image_points) != len(object_points):
            failures.append(
                f"{side}: ridge/object correspondence counts differ ({len(image_points)} != {len(object_points)})"
            )
            continue
        try:
            correspondences = PnpCorrespondences.from_raw_camera_pixels(
                object_points_ned_m=object_points,
                image_points_px=image_points,
                camera_matrix=camera.camera_matrix,
                distortion_coefficients=camera.distortion_coefficients,
            )
            ensemble = solve_pnp_ensemble(correspondences)
            best = ensemble.best_candidate
            if best is None:
                failures.append(f"{side}: all PnP solvers failed: {ensemble.failures}")
                continue
            solved[side] = (best, ensemble, annotation)
        except Exception as exc:
            failures.append(f"{side}: {exc}")
    if not solved:
        detail = (
            "; ".join(failures)
            or "provide per-side frozen ridge annotations plus explicit object_points_ned_m matched one-to-one; a skyline polyline alone is not a 3D/2D PnP correspondence set"
        )
        raise ValueError(f"terrain PnP correspondence evidence is incomplete: {detail}")
    left = solved.get("left", (None,))[0].pose_ned if "left" in solved else priors[0]
    right = solved.get("right", (None,))[0].pose_ned if "right" in solved else priors[1]
    full_rank = all((row[0].jacobian_diagnostics.full_rank for row in solved.values()))
    observable = _ALL_DOF if full_rank else PoseDofMask.from_names(())
    quality: list[tuple[str, float]] = []
    uncertainty: list[tuple[str, float]] = []
    source_ids: list[str] = [f"terrain:{terrain_metadata.sha256}"]
    hashes: list[tuple[str, str]] = [
        ("operating_intrinsics_profile_sha256", profile.profile_sha256),
        ("terrain_geotiff_sha256", terrain_metadata.sha256),
    ]
    if correspondence_file_sha256:
        hashes.append(("terrain_correspondence_json_sha256", correspondence_file_sha256))
    for side, (best, _ensemble, annotation) in solved.items():
        quality.extend(
            (
                (f"{side}_reprojection_rmse_px", best.reprojection_rmse),
                (f"{side}_reprojection_p95_px", best.reprojection_p95),
                (f"{side}_positive_depth_ratio", best.positive_depth_ratio),
            )
        )
        condition = float(best.jacobian_diagnostics.condition_number)
        if math.isfinite(condition) and condition >= 0.0:
            uncertainty.append((f"{side}_jacobian_condition", condition))
        source_ids.append(str(annotation["frame_id"]))
        hashes.append((f"{side}_annotation_sha256", _payload_sha256(annotation)))
    quality.append(("reprojection_error_px", max((row[0].reprojection_rmse for row in solved.values()))))
    warnings = list(failures)
    if not full_rank:
        warnings.append("PnP Jacobian is rank deficient; no DOF is marked applicable")
    candidate = OpticalPoseCandidate(
        candidate_id=f"terrain-pnp-{uuid.uuid4().hex}",
        method="terrain-pnp-ensemble",
        left_pose_ned=left,
        right_pose_ned=right,
        optimized_dofs=_ALL_DOF,
        observability=_mask_observability(observable),
        source_observation_ids=tuple(dict.fromkeys(source_ids)),
        input_hashes=tuple(hashes),
        quality_metrics=tuple(quality),
        uncertainty_metrics=tuple(uncertainty),
        warnings=tuple(warnings),
        created_utc_ns=time.time_ns(),
        observability_by_side=tuple(
            (
                (side, _mask_observability(observable if side in solved else PoseDofMask.from_names(())))
                for side in ("left", "right")
            )
        ),
    )
    return candidate
