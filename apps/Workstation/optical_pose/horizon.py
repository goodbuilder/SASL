"""Horizon candidate computation from explicit profile/prior/annotations.

Gravity is NED Down; priors map optical RDF to NED. Roll/Pitch output is
virtual-FRD degrees, centres remain NED metres, annotations remain pixels.
No panel, active-profile lookup, artifact persistence or candidate registry.
"""

from __future__ import annotations
import math
import time
import uuid
from typing import Any, Mapping, Sequence, TYPE_CHECKING
from Workstation.workstation_core.operating_intrinsics import OperatingIntrinsicsProfile

if TYPE_CHECKING:
    from Workstation.workstation_core.native_frame_store import NativeFrameArtifact
from Workstation.workstation_core.optical_pose import (
    CameraPoseNed,
    OpticalPoseCandidate,
    PoseDofMask,
    PoseParametersNed,
    geometric_horizon_dip_rad,
    pose_parameters_from_camera_pose,
)
from Workstation.workstation_core.pose_solvers import fit_normalized_horizon_line, solve_spherical_horizon_roll_pitch
from Workstation.workstation_core.optical_pose_auto_select import HORIZON_MIN_ENDPOINT_SEPARATION_FRACTION
from .geometry_values import _matvec, _mat_transpose, _mask_observability, _payload_sha256
from .observations import validate_annotation_identity, annotation_points, sample_horizon_pixels, undistort_pixels

from .policy import _HORIZON_DOF


class HorizonPoseSolver:
    _HORIZON_CLICK_UNCERTAINTY_PX = 1.0
    _HORIZON_MIN_ENDPOINT_SEPARATION_FRACTION = HORIZON_MIN_ENDPOINT_SEPARATION_FRACTION
    _HORIZON_MODELS = ("flat_local", "spherical_geometric")

    def _solve_horizon_roll_pitch(
        self,
        normalized_points: Sequence[tuple[float, float]],
        *,
        horizon_model: str,
        dip_rad: float | None,
        prior: CameraPoseNed,
    ) -> tuple[float, float, float, float]:
        """Return ``(roll_deg, pitch_deg, rms_normalized, max_abs_normalized)``.

        ``flat_local`` treats the horizon as ``g.r=0`` (a flat/unrefracted
        line fit); the prior's expected gravity only resolves the ``g`` vs
        ``-g`` projective sign ambiguity, it never supplies the Roll/Pitch
        value itself. ``spherical_geometric`` instead solves ``g.r=sin(dip)``
        per sampled ray with a fixed prior Yaw -- Yaw and position are never
        variables of this fit.
        """
        before = pose_parameters_from_camera_pose(prior)
        if horizon_model == "spherical_geometric":
            assert dip_rad is not None
            rays = tuple(((x, y, 1.0) for (x, y) in normalized_points))
            result = solve_spherical_horizon_roll_pitch(
                rays,
                dip_rad=dip_rad,
                prior_roll_deg=before.roll_deg,
                prior_pitch_deg=before.pitch_deg,
                prior_yaw_deg=before.yaw_deg,
            )
            return (result.roll_deg, result.pitch_deg, result.rms_normalized, result.max_abs_normalized)
        expected_gravity = _matvec(_mat_transpose(prior.rotation_ned_from_camera_rdf), (0.0, 0.0, 1.0))
        fit = fit_normalized_horizon_line(normalized_points, expected_gravity_camera_rdf=expected_gravity)
        return (fit.roll_deg, fit.pitch_deg, fit.rms_normalized, fit.max_abs_normalized)

    def _horizon_endpoint_sensitivity_deg(
        self,
        *,
        raw_points: Sequence[tuple[float, float]],
        camera: Any,
        horizon_model: str,
        dip_rad: float | None,
        prior: CameraPoseNed,
        nominal_roll_deg: float,
        nominal_pitch_deg: float,
    ) -> tuple[float, float]:
        """Deterministic one-axis-at-a-time +/-1px endpoint perturbation.

        This is a worst-case manual-click sensitivity, not a statistical
        1-sigma uncertainty: each of the two endpoints is perturbed along u
        and v independently by ``_HORIZON_CLICK_UNCERTAINTY_PX``, the same
        model is re-solved, and the maximum absolute Roll/Pitch delta from
        the nominal solve is retained.
        """
        max_roll_delta = 0.0
        max_pitch_delta = 0.0
        step = self._HORIZON_CLICK_UNCERTAINTY_PX
        for endpoint_index in range(2):
            for axis in (0, 1):
                for sign in (1.0, -1.0):
                    perturbed = list(raw_points)
                    (u, v) = perturbed[endpoint_index]
                    if axis == 0:
                        u = u + sign * step
                    else:
                        v = v + sign * step
                    perturbed[endpoint_index] = (u, v)
                    sampled = sample_horizon_pixels(tuple(perturbed))
                    normalized = undistort_pixels(camera, sampled)
                    (roll_deg, pitch_deg, _rms, _max_abs) = self._solve_horizon_roll_pitch(
                        normalized, horizon_model=horizon_model, dip_rad=dip_rad, prior=prior
                    )
                    max_roll_delta = max(max_roll_delta, abs(roll_deg - nominal_roll_deg))
                    max_pitch_delta = max(max_pitch_delta, abs(pitch_deg - nominal_pitch_deg))
        return (max_roll_delta, max_pitch_delta)

    def solve(
        self,
        payload: Mapping[str, Any],
        *,
        profile: OperatingIntrinsicsProfile,
        priors: tuple[CameraPoseNed, CameraPoseNed],
        annotations: Mapping[str, Mapping[str, Any]],
        artifacts: Mapping[str, NativeFrameArtifact],
    ) -> OpticalPoseCandidate:
        horizon_model = str(payload.get("horizon_model") or "flat_local")
        if horizon_model not in self._HORIZON_MODELS:
            raise ValueError(f"unsupported horizon_model: {horizon_model!r}")
        dip_rad: float | None = None
        camera_height_m: float | None = None
        if horizon_model == "spherical_geometric":
            height_raw = payload.get("camera_height_above_reference_surface_m")
            if height_raw is None:
                raise ValueError(
                    "spherical_geometric horizon requires an explicit camera_height_above_reference_surface_m; GNSS ellipsoid altitude is not automatically substituted"
                )
            try:
                camera_height_m = float(height_raw)
            except (TypeError, ValueError) as exc:
                raise ValueError("camera_height_above_reference_surface_m must be numeric") from exc
            if not math.isfinite(camera_height_m) or camera_height_m < 0.0:
                raise ValueError("camera_height_above_reference_surface_m must be a finite, non-negative metres value")
            dip_rad = geometric_horizon_dip_rad(camera_height_m)
        solved: dict[str, dict[str, Any]] = {}
        for side, prior, camera in (("left", priors[0], profile.left), ("right", priors[1], profile.right)):
            annotation = annotations.get(side)
            if not isinstance(annotation, Mapping):
                continue
            validate_annotation_identity(artifacts.get(side), side, annotation)
            points = annotation_points(annotation, "horizon_line")
            if len(points) < 2:
                continue
            sensitivity: tuple[float, float] | None = None
            if len(points) == 2:
                width = float(annotation["source_width_px"])
                separation = math.hypot(points[1][0] - points[0][0], points[1][1] - points[0][1])
                minimum_separation = self._HORIZON_MIN_ENDPOINT_SEPARATION_FRACTION * width
                if separation < minimum_separation:
                    raise ValueError(
                        f"{side}: horizon endpoints are only {separation:.1f}px apart; at least {minimum_separation:.1f}px ({self._HORIZON_MIN_ENDPOINT_SEPARATION_FRACTION:.0%} of image width) is required so +/-1px click noise stays bounded"
                    )
            sampled = sample_horizon_pixels(points)
            normalized = undistort_pixels(camera, sampled)
            (roll_deg, pitch_deg, rms_normalized, max_abs_normalized) = self._solve_horizon_roll_pitch(
                normalized, horizon_model=horizon_model, dip_rad=dip_rad, prior=prior
            )
            if not (math.isfinite(roll_deg) and math.isfinite(pitch_deg)):
                raise ValueError(f"{side}: horizon solve returned a non-finite Roll/Pitch")
            if len(points) == 2:
                sensitivity = self._horizon_endpoint_sensitivity_deg(
                    raw_points=points,
                    camera=camera,
                    horizon_model=horizon_model,
                    dip_rad=dip_rad,
                    prior=prior,
                    nominal_roll_deg=roll_deg,
                    nominal_pitch_deg=pitch_deg,
                )
            before = pose_parameters_from_camera_pose(prior)
            updated = PoseParametersNed(
                roll_deg, pitch_deg, before.yaw_deg, before.north_m, before.east_m, before.down_m
            ).to_camera_pose_ned()
            solved[side] = {
                "pose": updated,
                "rms_normalized": rms_normalized,
                "max_abs_normalized": max_abs_normalized,
                "sensitivity_deg": sensitivity,
                "annotation": annotation,
            }
        if not solved:
            raise ValueError("frozen horizon annotations require at least two native points on left or right")
        left = solved.get("left", {}).get("pose", priors[0])
        right = solved.get("right", {}).get("pose", priors[1])
        quality: list[tuple[str, float]] = []
        source_ids: list[str] = []
        input_hashes: list[tuple[str, str]] = [
            ("operating_intrinsics_profile_sha256", profile.profile_sha256),
            ("horizon_model", horizon_model),
        ]
        if camera_height_m is not None:
            input_hashes.append(("camera_height_above_reference_surface_m", f"{camera_height_m:.6f}"))
        for side, row in solved.items():
            approximate_px = float(row["rms_normalized"]) * math.sqrt(
                float(getattr(profile, side).camera_matrix[0][0]) * float(getattr(profile, side).camera_matrix[1][1])
            )
            quality.extend(
                (
                    (f"{side}_rms_normalized", float(row["rms_normalized"])),
                    (f"{side}_max_abs_normalized", float(row["max_abs_normalized"])),
                    (f"{side}_rms_px_equivalent", approximate_px),
                )
            )
            sensitivity = row["sensitivity_deg"]
            if sensitivity is not None:
                (roll_sensitivity, pitch_sensitivity) = sensitivity
                quality.extend(
                    (
                        (f"{side}_roll_sensitivity_1px_deg", float(roll_sensitivity)),
                        (f"{side}_pitch_sensitivity_1px_deg", float(pitch_sensitivity)),
                    )
                )
            source_ids.append(str(row["annotation"]["frame_id"]))
            input_hashes.append((f"{side}_annotation_sha256", _payload_sha256(row["annotation"])))
        warnings: list[str] = [
            "yaw and optical-center position are unobservable from a horizon",
            "the prior expected gravity resolves only the g vs -g projective sign; it is not the source of the recovered Roll/Pitch value",
            "rms_normalized is a line/model fit-quality metric, not a pose uncertainty; see *_roll_sensitivity_1px_deg / *_pitch_sensitivity_1px_deg for a worst-case +/-1px manual endpoint-click sensitivity",
            "use only a physical/geometric horizon compatible with the selected model; a mountain ridge, building skyline, or tree line is not a valid horizon input (use the Mountain method for a ridge)",
        ]
        if horizon_model == "flat_local":
            warnings.append("flat/unrefracted horizon model; Earth curvature dip is not applied")
        else:
            warnings.append("geometric spherical-Earth horizon; atmospheric refraction is not modeled")
        candidate = OpticalPoseCandidate(
            candidate_id=f"horizon-{uuid.uuid4().hex}",
            method="flat-horizon-roll-pitch" if horizon_model == "flat_local" else "spherical-horizon-roll-pitch",
            left_pose_ned=left,
            right_pose_ned=right,
            optimized_dofs=_HORIZON_DOF,
            observability=_mask_observability(_HORIZON_DOF),
            source_observation_ids=tuple(dict.fromkeys(source_ids)),
            input_hashes=tuple(input_hashes),
            quality_metrics=tuple(quality),
            warnings=tuple(warnings),
            created_utc_ns=time.time_ns(),
            observability_by_side=tuple(
                (
                    (side, _mask_observability(_HORIZON_DOF if side in solved else PoseDofMask.from_names(())))
                    for side in ("left", "right")
                )
            ),
        )
        return candidate
