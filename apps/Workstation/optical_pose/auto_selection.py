"""Auto-selection computation over owned frozen inputs, without a panel.

K,D use the native optical pixel grid. Priors rotate optical RDF into NED;
GNSS baseline is right-minus-left NED metres. Outputs are native u/v pixel
suggestions. Detection/approval policy and core algorithms are unchanged.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import time
import uuid
from typing import Any, Callable, Mapping
from PyQt5 import QtGui

from Workstation.workstation_core.immutable_json import FrozenJsonObject, thaw_json
from Workstation.workstation_core.native_frame_store import NativeFrameArtifact
from Workstation.workstation_core.operating_intrinsics import OperatingCameraIntrinsics, OperatingIntrinsicsProfile
from Workstation.workstation_core.optical_pose import CameraPoseNed, stereo_pose_from_absolute
from Workstation.workstation_core.optical_pose_auto_select import (
    AutoSelectionResult,
    MountainAutoSelection,
    validate_horizon_suggestion_points,
    validate_mountain_suggestion_points,
)
from .geometry_values import _mat_transpose, _matvec, _payload_sha256, _pose_mapping


@dataclass(frozen=True)
class FrozenAutoSelectionFrame:
    side: str
    _image: QtGui.QImage = field(repr=False)
    artifact: NativeFrameArtifact

    def __post_init__(self) -> None:
        if not isinstance(self._image, QtGui.QImage) or self._image.isNull():
            raise RuntimeError(f"{self.side} frozen native frame is unavailable")
        object.__setattr__(self, "_image", self._image.copy())

    def materialize(self) -> dict[str, Any]:
        # QImage value copies use copy-on-write, including mutable bits(). A
        # detector can alter its own image without changing the frozen input.
        return {"image": QtGui.QImage(self._image), "artifact": self.artifact}


@dataclass(frozen=True)
class FrozenAutoSelectionInputs:
    frames: tuple[FrozenAutoSelectionFrame, FrozenAutoSelectionFrame]
    profile: OperatingIntrinsicsProfile | None
    priors: tuple[CameraPoseNed, CameraPoseNed] | None
    applicability_warnings: tuple[str, ...]
    baseline_ned_m: tuple[float, float, float] | None
    baseline_details: FrozenJsonObject
    baseline_error: str
    ridge_margin_fraction: float
    max_points: int


@dataclass(frozen=True)
class AutoSelectionDetectors:
    horizon: Callable[..., AutoSelectionResult]
    mountain: Callable[..., MountainAutoSelection]

    def __post_init__(self) -> None:
        if not callable(self.horizon) or not callable(self.mountain):
            raise TypeError("Auto-selection requires both detector operations")


@dataclass(frozen=True)
class OpticalPoseAutoSelectJob:
    inputs: FrozenAutoSelectionInputs
    mode: str
    detectors: AutoSelectionDetectors

    def run(self) -> dict[str, Any]:
        return OpticalAutoSelection(self.inputs, self.detectors)._compute_auto_selection(self.mode, {})


class OpticalAutoSelection:
    """One computation's values only: no live providers or UI state."""

    def __init__(self, inputs: FrozenAutoSelectionInputs, detectors: AutoSelectionDetectors) -> None:
        self._inputs = inputs
        self._selectors = detectors
        self._frozen_frames = {frame.side: frame.materialize() for frame in inputs.frames}
        self._mountain_ridge_margin_fraction = inputs.ridge_margin_fraction
        self._mountain_max_points = inputs.max_points

    def _auto_select_inputs(self, _payload):
        return self._inputs.profile, self._inputs.priors, list(self._inputs.applicability_warnings)

    def _resolve_gnss_baseline_for_mountain(self, _payload):
        if self._inputs.baseline_error:
            raise RuntimeError(self._inputs.baseline_error)
        return self._inputs.baseline_ned_m, thaw_json(self._inputs.baseline_details)

    def _compute_auto_selection(
        self,
        mode: str,
        payload: Mapping[str, Any],
    ) -> dict[str, Any]:
        normalized_mode = str(mode or "").strip().lower()
        if normalized_mode == "horizon":
            return self._compute_auto_select_horizon(payload)
        if normalized_mode == "mountain":
            return self._compute_auto_select_mountain(payload)
        if normalized_mode == "cloud":
            return self._compute_auto_select_cloud(payload)
        raise ValueError(f"unsupported optical-pose auto-selection mode: {mode!r}")

    def _compute_auto_select_horizon(
        self,
        payload: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Create a reviewed Horizon suggestion from the frozen native burst."""

        values = dict(payload or {})
        profile, priors, applicability_warnings = self._auto_select_inputs(values)
        results: dict[str, AutoSelectionResult] = {}
        for side_index, side in enumerate(("left", "right")):
            prior = None if priors is None else priors[side_index]
            camera = self._resolution_safe_camera(side, profile)
            expected_gravity = (
                None if prior is None else _matvec(_mat_transpose(prior.rotation_ned_from_camera_rdf), (0.0, 0.0, 1.0))
            )
            results[side] = self._selectors.horizon(
                self._frozen_frames[side]["image"],
                camera_matrix=None if camera is None else camera.camera_matrix,
                distortion_coefficients=None if camera is None else camera.distortion_coefficients,
                expected_gravity_camera_rdf=expected_gravity,
            )
        side_mappings = {
            side: self._auto_suggestion_side_mapping(
                side,
                result,
                self._frozen_frames[side],
                annotation_key="horizon_line",
            )
            for side, result in results.items()
        }
        try:
            validate_horizon_suggestion_points(
                results["left"].points,
                results["right"].points,
                left_image_size=(
                    self._frozen_frames["left"]["artifact"].width,
                    self._frozen_frames["left"]["artifact"].height,
                ),
                right_image_size=(
                    self._frozen_frames["right"]["artifact"].width,
                    self._frozen_frames["right"]["artifact"].height,
                ),
            )
        except (KeyError, TypeError, ValueError) as exc:
            copy_eligible = False
            copy_block_reason = str(exc)
        else:
            copy_eligible = True
            copy_block_reason = ""
        both_eligible = set(results) == {"left", "right"} and all(results[side].eligible for side in ("left", "right"))
        any_points = [result for result in results.values() if result.points]
        state = "GOOD" if both_eligible else "LOW_CONFIDENCE" if any_points else "NO_DATA"
        confidence = max((result.confidence for result in results.values()), default=0.0)
        inliers = sum(result.inlier_count for result in results.values())
        residuals = [result.residual_px for result in results.values() if result.residual_px is not None]
        failure_reasons = [result.reason for result in results.values() if result.reason != "ok"]
        suggestion_id = f"auto-horizon-{uuid.uuid4().hex}"
        suggestion = {
            "schema": "optical-pose-auto-suggestion-v1",
            "suggestion_id": suggestion_id,
            "mode": "horizon",
            "state": state,
            "provenance_state": "AUTO_PROPOSED",
            "detection_method": " / ".join(sorted({result.detection_method for result in results.values()})),
            "confidence": confidence,
            "inlier_count": inliers,
            "residual_px": max(residuals) if residuals else None,
            "eligible": both_eligible,
            # Detector quality remains evidence; copy permission is the
            # independent native-endpoint structure contract above.
            "copy_eligible": copy_eligible,
            "copy_block_reason": copy_block_reason,
            "failure_reason": "; ".join(dict.fromkeys(failure_reasons)),
            "warnings": [
                *applicability_warnings,
                "automatic Horizon selection proposes endpoints only; no solver was executed",
                "flat horizon observes Roll/Pitch, not Yaw",
            ],
            "sides": side_mappings,
            "source_frames": {
                side: {
                    "frame_id": self._frozen_frames[side]["artifact"].frame_id,
                    "image_sha256": self._frozen_frames[side]["artifact"].sha256,
                }
                for side in ("left", "right")
            },
            "input_hashes": {
                **({"operating_intrinsics_profile_sha256": profile.profile_sha256} if profile is not None else {}),
                **(
                    {
                        "prior_pose_sha256": _payload_sha256(
                            {"left": _pose_mapping(priors[0]), "right": _pose_mapping(priors[1])}
                        )
                    }
                    if priors is not None
                    else {}
                ),
            },
            "created_utc_ns": time.time_ns(),
        }
        return suggestion

    def _compute_auto_select_cloud(
        self,
        payload: Mapping[str, Any] | None = None,
    ) -> dict[str, Any]:
        """Cloud mode: the Mountain matcher without any ridge bias.

        Every mutual match is treated as a far-field sky/cloud correspondence
        and the resulting points feed the rotation-only Cloud solve.
        """

        return self._compute_auto_select_mountain(payload, far_field_only=True, mode="cloud")

    def _compute_auto_select_mountain(
        self,
        payload: Mapping[str, Any] | None = None,
        *,
        far_field_only: bool = False,
        mode: str = "mountain",
    ) -> dict[str, Any]:
        """Create a reviewed stereo-feature Mountain suggestion.

        The selector may still return image matches when a baseline is
        temporarily unavailable so the operator can inspect the evidence, but
        solver approval remains blocked until the existing GNSS baseline
        contract is satisfied.
        """

        values = dict(payload or {})
        profile, priors, applicability_warnings = self._auto_select_inputs(values)
        baseline_ned: tuple[float, float, float] | None = None
        baseline_details: dict[str, Any] = {}
        baseline_warning = ""
        try:
            baseline_ned, baseline_details = self._resolve_gnss_baseline_for_mountain(values)
        except (RuntimeError, ValueError) as exc:
            baseline_warning = str(exc)
        prior_rotation = None if priors is None else stereo_pose_from_absolute(*priors).rotation_right_from_left_rdf
        left_camera = self._resolution_safe_camera("left", profile)
        right_camera = self._resolution_safe_camera("right", profile)
        result: MountainAutoSelection = self._selectors.mountain(
            self._frozen_frames["left"]["image"],
            self._frozen_frames["right"]["image"],
            left_camera_matrix=None if left_camera is None else left_camera.camera_matrix,
            right_camera_matrix=None if right_camera is None else right_camera.camera_matrix,
            left_distortion_coefficients=None if left_camera is None else left_camera.distortion_coefficients,
            right_distortion_coefficients=None if right_camera is None else right_camera.distortion_coefficients,
            baseline_ned_m=baseline_ned,
            prior_rotation_right_from_left_rdf=prior_rotation,
            ridge_margin_fraction=self._mountain_ridge_margin_fraction,
            max_points=self._mountain_max_points,
            far_field_only=far_field_only,
        )
        sides = {
            "left": self._auto_suggestion_side_mapping(
                "left", result.left, self._frozen_frames["left"], annotation_key="ridge_polyline"
            ),
            "right": self._auto_suggestion_side_mapping(
                "right", result.right, self._frozen_frames["right"], annotation_key="ridge_polyline"
            ),
        }
        try:
            validate_mountain_suggestion_points(result.left.points, result.right.points)
        except (TypeError, ValueError) as exc:
            copy_eligible = False
            copy_block_reason = str(exc)
        else:
            copy_eligible = True
            copy_block_reason = ""
        warnings = [*applicability_warnings, *result.warnings]
        if baseline_warning:
            warnings.append(f"GNSS baseline unavailable during suggestion: {baseline_warning}")
        suggestion_id = f"auto-{mode}-{uuid.uuid4().hex}"
        input_hashes: dict[str, str] = {}
        if profile is not None:
            input_hashes["operating_intrinsics_profile_sha256"] = profile.profile_sha256
        if priors is not None:
            input_hashes["prior_pose_sha256"] = _payload_sha256(
                {"left": _pose_mapping(priors[0]), "right": _pose_mapping(priors[1])}
            )
        if baseline_details:
            input_hashes["gnss_baseline_sha256"] = _payload_sha256(baseline_details)
        suggestion = {
            "schema": "optical-pose-auto-suggestion-v1",
            "suggestion_id": suggestion_id,
            "mode": mode,
            "state": result.state,
            "provenance_state": "AUTO_PROPOSED",
            "detection_method": result.detection_method,
            "confidence": result.confidence,
            "inlier_count": result.inlier_count,
            "match_count": result.match_count,
            "residual_px": result.residual_px,
            "eligible": bool(result.eligible and baseline_ned is not None),
            # Mountain point-copy permission is structural and deliberately
            # independent from detector confidence and GNSS solve readiness.
            "copy_eligible": copy_eligible,
            "copy_block_reason": copy_block_reason,
            "failure_reason": result.failure_reason or baseline_warning,
            "warnings": warnings,
            "sides": sides,
            # Rotation-only far-field fit over every RANSAC inlier (see
            # optical_pose_auto_select._fit_far_field_relative_rotation).
            "relative_rotation": dict(result.metadata.get("relative_rotation") or {}),
            "source_frames": {
                side: {
                    "frame_id": self._frozen_frames[side]["artifact"].frame_id,
                    "image_sha256": self._frozen_frames[side]["artifact"].sha256,
                }
                for side in ("left", "right")
            },
            "input_hashes": input_hashes,
            "gnss_baseline": dict(baseline_details),
            "created_utc_ns": time.time_ns(),
        }
        return suggestion

    def _resolution_safe_camera(
        self, side: str, profile: OperatingIntrinsicsProfile | None
    ) -> OperatingCameraIntrinsics | None:
        """This side's calibrated camera, or ``None`` if it was calibrated at
        a different resolution than the frozen burst's native frame.

        Undistortion is not merely less accurate when the calibration
        resolution does not match the image being undistorted -- the
        principal point and focal length are only meaningful in the pixel
        grid they were calibrated in, so ``cv2.undistort`` warps the frame
        into something unrelated to the original picture (large black
        borders, the real content compressed into a fisheye-like remainder).
        A resolution-mismatched K,D must be treated exactly like a missing
        camera model, not like a merely non-exact one.
        """

        if profile is None or side not in self._frozen_frames:
            return None
        camera: OperatingCameraIntrinsics = getattr(profile, side)
        image = self._frozen_frames[side]["image"]
        if int(image.width()) != int(camera.width_px) or int(image.height()) != int(camera.height_px):
            return None
        return camera

    @staticmethod
    def _auto_suggestion_side_mapping(
        side: str,
        result: AutoSelectionResult,
        frozen: Mapping[str, Any],
        *,
        annotation_key: str,
    ) -> dict[str, Any]:
        artifact: NativeFrameArtifact = frozen["artifact"]
        value = result.to_mapping()
        value.update(
            {
                "side": side,
                "frame_id": artifact.frame_id,
                "image_sha256": artifact.sha256,
                "source_width_px": artifact.width,
                "source_height_px": artifact.height,
                "annotation_key": annotation_key,
                # ``points`` is the stable generic selector schema.  The
                # annotation key is included so consumers can route it to the
                # existing Horizon/Ridge buffers without guessing.
                annotation_key: list(value.get("points", ())),
            }
        )
        return value
