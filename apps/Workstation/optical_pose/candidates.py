"""Single owner of solved optical candidates, priors and selection history.

Candidate poses use optical RDF -> NED and NED centres in metres; delta
RPY displays use virtual body FRD degrees. A preview never applies geometry.
"""

from __future__ import annotations

import hashlib
import json
from typing import Any, Mapping
from Workstation.workstation_core.optical_pose import (
    CameraPoseNed,
    OpticalPoseCandidate,
    OpticalPoseSelection,
    POSE_DOF_NAMES,
    pose_parameters_from_camera_pose,
)
from Workstation.workstation_core.run_evidence import canonical_json_bytes
from .geometry_values import _pose_mapping, _payload_sha256


def _candidate_mapping(candidate: OpticalPoseCandidate) -> dict[str, Any]:
    left = pose_parameters_from_camera_pose(candidate.left_pose_ned)
    right = pose_parameters_from_camera_pose(candidate.right_pose_ned)
    residuals = dict(candidate.quality_metrics)
    side_observability = {side: dict(values) for side, values in candidate.observability_by_side}
    common_observability = {
        name: all(side_observability[side].get(name, False) for side in ("left", "right")) for name in POSE_DOF_NAMES
    }
    mapping = {
        "schema": "optical-pose-candidate-v1",
        "candidate_id": candidate.candidate_id,
        "method": candidate.method,
        "left_pose": _pose_mapping(candidate.left_pose_ned),
        "right_pose": _pose_mapping(candidate.right_pose_ned),
        "left_delta_rpy_deg": [left.roll_deg, left.pitch_deg, left.yaw_deg],
        "right_delta_rpy_deg": [right.roll_deg, right.pitch_deg, right.yaw_deg],
        "relative_delta_yaw_deg": candidate.relative_boresight_yaw_right_minus_left_deg,
        "residual": (
            residuals.get("angular_residual_rad")
            if residuals.get("angular_residual_rad") is not None
            else residuals.get("reprojection_error_px")
            if residuals.get("reprojection_error_px") is not None
            else residuals.get("sampson_rms_px_equivalent")
            if residuals.get("sampson_rms_px_equivalent") is not None
            else residuals.get("rms_px_equivalent")
        ),
        "observable_dof": common_observability,
        "observable_dof_by_side": side_observability,
        "optimized_dof": {name: bool(getattr(candidate.optimized_dofs, name)) for name in POSE_DOF_NAMES},
        "applicable": True,
        "state": "WARNING" if candidate.warnings else "CANDIDATE",
        "warnings": list(candidate.warnings),
        "source_observation_ids": list(candidate.source_observation_ids),
        "input_hashes": dict(candidate.input_hashes),
        "quality_metrics": residuals,
        "uncertainty_metrics": dict(candidate.uncertainty_metrics),
        "created_utc_ns": candidate.created_utc_ns,
    }
    if candidate.method in {"gnss-constrained-mountain-uv", "cloud-far-field-rotation"}:
        relative_pose = candidate.stereo_pose_rdf
        mapping["relative_pose_rdf"] = {
            "rotation_right_from_left_rdf": [list(row) for row in relative_pose.rotation_right_from_left_rdf],
            "translation_right_from_left_rdf_m": list(relative_pose.translation_right_from_left_rdf_m),
            "right_origin_in_left_rdf_m": list(relative_pose.right_origin_in_left_rdf_m),
            "frame_contract": ("X_right = R_right_from_left_rdf * X_left + t_right_from_left_rdf_m"),
            "unit": "m",
        }
    if candidate.reference_evidence_json:
        mapping["yaw90_reference"] = json.loads(candidate.reference_evidence_json)
        mapping["pose_roles"] = {"left": "assumed_reference", "right": "derived_from_reference_and_images"}
    return mapping


def candidate_mapping_with_prior(
    candidate: OpticalPoseCandidate, prior: tuple[CameraPoseNed, CameraPoseNed]
) -> dict[str, Any]:
    mapping = _candidate_mapping(candidate)
    deltas: list[list[float]] = []
    for pose, base in zip((candidate.left_pose_ned, candidate.right_pose_ned), prior):
        current = pose_parameters_from_camera_pose(pose)
        previous = pose_parameters_from_camera_pose(base)
        deltas.append(
            [
                current.roll_deg - previous.roll_deg,
                current.pitch_deg - previous.pitch_deg,
                (current.yaw_deg - previous.yaw_deg + 180.0) % 360.0 - 180.0,
            ]
        )
    mapping["left_delta_rpy_deg"] = deltas[0]
    mapping["right_delta_rpy_deg"] = deltas[1]
    mapping["prior_pose_sha256"] = _payload_sha256({"left": _pose_mapping(prior[0]), "right": _pose_mapping(prior[1])})
    return mapping


class OpticalCandidateRegistry:
    def __init__(self) -> None:
        self._candidates: dict[str, OpticalPoseCandidate] = {}
        self._candidate_priors: dict[str, tuple[CameraPoseNed, CameraPoseNed]] = {}
        self._candidate_drone_rotation_context: dict[str, dict[str, dict[str, Any]]] = {}
        self._active_selection: OpticalPoseSelection | None = None
        self._active_poses: tuple[CameraPoseNed, CameraPoseNed] | None = None
        self._selection_history: list[OpticalPoseSelection] = []

    def validate_new(self, candidate: OpticalPoseCandidate) -> None:
        if not isinstance(candidate, OpticalPoseCandidate):
            raise TypeError("candidate must be OpticalPoseCandidate")
        if candidate.candidate_id in self._candidates:
            raise ValueError(f"duplicate optical pose candidate: {candidate.candidate_id}")

    def register(
        self, candidate: OpticalPoseCandidate, prior_poses: tuple[CameraPoseNed, CameraPoseNed]
    ) -> dict[str, Any]:
        self.validate_new(candidate)
        if (
            not isinstance(prior_poses, tuple)
            or len(prior_poses) != 2
            or not all(isinstance(pose, CameraPoseNed) for pose in prior_poses)
        ):
            raise TypeError("candidate prior_poses must contain left/right CameraPoseNed")
        self._candidates[candidate.candidate_id] = candidate
        self._candidate_priors[candidate.candidate_id] = prior_poses
        return candidate_mapping_with_prior(candidate, prior_poses)

    def select(self, selection: OpticalPoseSelection, poses: tuple[CameraPoseNed, CameraPoseNed]) -> None:
        self._active_poses = poses
        self._active_selection = selection
        self._selection_history.append(selection)

    def restore_selection(self, snapshot: Mapping[str, Any]) -> None:
        self._active_selection = snapshot.get("active_selection")
        self._active_poses = snapshot.get("active_poses")
        self._selection_history[:] = list(snapshot.get("selection_history") or ())

    def latest_poses(self) -> tuple[dict[str, Any], ...]:
        """Return the most recently solved candidate per method, unapplied included.

        This is a read-only preview accessor for geometry-comparison displays;
        it never mutates ``active_intrinsics``/``selected_extrinsics_profile``.
        """
        latest_by_method: dict[str, OpticalPoseCandidate] = {}
        for candidate in self._candidates.values():
            current = latest_by_method.get(candidate.method)
            if current is None or candidate.created_utc_ns > current.created_utc_ns:
                latest_by_method[candidate.method] = candidate
        active_candidate_id = "" if self._active_selection is None else self._active_selection.candidate_id
        rows: list[dict[str, Any]] = []
        for candidate in latest_by_method.values():
            candidate_payload = candidate_mapping_with_prior(candidate, self._candidate_priors[candidate.candidate_id])
            rows.append(
                {
                    "candidate_id": candidate.candidate_id,
                    "method": candidate.method,
                    "left_pose_ned": candidate.left_pose_ned,
                    "right_pose_ned": candidate.right_pose_ned,
                    "optimized_dofs": candidate.optimized_dofs.selected_names,
                    "candidate_profile_hash": hashlib.sha256(canonical_json_bytes(candidate_payload)).hexdigest(),
                    "candidate_intrinsics_hash": str(
                        dict(candidate.input_hashes).get("operating_intrinsics_profile_sha256", "")
                    ),
                    "candidate_quality": dict(candidate.quality_metrics),
                    "candidate_warnings": list(candidate.warnings),
                    "candidate_applied": candidate.candidate_id == active_candidate_id,
                }
            )
        return tuple(rows)
