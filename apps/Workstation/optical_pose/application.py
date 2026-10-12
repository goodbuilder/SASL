"""Optical profile preparation and one guarded apply/revert transaction.

Absolute inputs are optical RDF -> world NED and NED metres. Derived profiles
retain X_right=R_right_from_left*X_left+T with RDF translation in metres.
Geometry remains in the shared ActiveGeometryStore; only restore tokens live here.
"""

from __future__ import annotations
import hashlib
import json
import time
import uuid
from dataclasses import dataclass
from typing import Any, Callable, Mapping
from Workstation.workstation_core.calibration import (
    StereoExtrinsics,
    StereoExtrinsicsProfile,
    StereoIntrinsicsResult,
    compose_stereo_calibration,
)
from Workstation.workstation_core.operating_intrinsics import OperatingIntrinsicsProfile
from Workstation.workstation_core.optical_pose import (
    CameraPoseNed,
    OpticalPoseCandidate,
    OpticalPoseSelection,
    PoseDofMask,
    PoseParametersNed,
    POSE_DOF_NAMES,
    apply_selected_dofs,
    pose_parameters_from_camera_pose,
    stereo_pose_from_absolute,
    boresight_relative_yaw_right_minus_left_deg,
)
from Workstation.workstation_core.run_evidence import canonical_json_bytes
from .geometry_values import _pose_mapping

_KD_INDEPENDENT_CANDIDATE_METHODS = frozenset({"reference/checkerboard", "reference/pixhawk-rtk-translation"})


@dataclass(frozen=True)
class _PreparedOpticalPoseApplication:
    """Complete optical-pose active state prepared without mutating the owner."""

    selection: OpticalPoseSelection
    poses: tuple[CameraPoseNed, CameraPoseNed]
    profile: StereoExtrinsicsProfile
    active_intrinsics: Any


def pose_mask(payload: Mapping[str, Any]) -> PoseDofMask:
    values = payload.get("dof")
    values = values if isinstance(values, Mapping) else payload
    aliases = {
        "roll_deg": "roll",
        "pitch_deg": "pitch",
        "yaw_deg": "yaw",
        "north_m": "north",
        "east_m": "east",
        "down_m": "down",
    }
    kwargs = {
        internal: bool(values.get(external, values.get(internal, False))) for internal, external in aliases.items()
    }
    return PoseDofMask(**kwargs)


def apply_candidate_with_constraints(
    candidate: OpticalPoseCandidate,
    mask: PoseDofMask,
    prior_left: CameraPoseNed,
    prior_right: CameraPoseNed,
) -> tuple[PoseParametersNed, PoseParametersNed]:
    """Apply exactly the candidate-owned optimized/observable DOF mask."""

    left_parameters = apply_selected_dofs(
        pose_parameters_from_camera_pose(prior_left),
        pose_parameters_from_camera_pose(candidate.left_pose_ned),
        mask,
    )
    right_parameters = apply_selected_dofs(
        pose_parameters_from_camera_pose(prior_right),
        pose_parameters_from_camera_pose(candidate.right_pose_ned),
        mask,
    )
    return left_parameters, right_parameters


def active_intrinsics_for_apply(
    intrinsics_snapshot: Mapping[str, Any],
    *,
    current,
    active_path,
    active_hash,
) -> tuple[StereoIntrinsicsResult, OperatingIntrinsicsProfile]:
    """Require the applicable profile to be the exact current active K,D.

    Measurement may deliberately use a ``NOMINAL_UNVERIFIED`` fallback,
    but an optical-pose Apply is an active-state transition. It therefore
    requires the existing active matrix, path, content hash, and profile
    math to describe one identity before any R,t state is prepared.
    """

    if not isinstance(current, StereoIntrinsicsResult):
        raise RuntimeError("active exact K,D is required before optical pose Apply")
    raw_profile = intrinsics_snapshot.get("profile")
    if not isinstance(raw_profile, Mapping):
        raise RuntimeError("active exact K,D profile identity is unavailable")
    profile = OperatingIntrinsicsProfile.from_mapping(raw_profile)
    if (
        not active_path
        or len(active_hash) != 64
        or any(character not in "0123456789abcdef" for character in active_hash)
    ):
        raise RuntimeError("active exact K,D path/hash identity is incomplete")
    if profile.source_path != active_path or profile.source_content_sha256 != active_hash:
        raise RuntimeError("applicable operating K,D is not the current active path/hash identity")
    active_math = OperatingIntrinsicsProfile.from_stereo_result(
        current,
        source_zoom_index=None,
    ).intrinsics_math_sha256
    if active_math != profile.intrinsics_math_sha256:
        raise RuntimeError("active K,D matrix/raster differs from its operating profile")
    return current, profile


def prepare_selected_profile(
    selection: OpticalPoseSelection,
    candidate: OpticalPoseCandidate,
    poses: tuple[CameraPoseNed, CameraPoseNed],
    intrinsics_snapshot: Mapping[str, Any],
    *,
    current_intrinsics: StereoIntrinsicsResult,
    operating_profile: OperatingIntrinsicsProfile,
    previous,
) -> _PreparedOpticalPoseApplication:
    """Validate and construct the complete next active state without mutation."""

    stereo = stereo_pose_from_absolute(*poses)
    previous_extrinsics = getattr(previous, "extrinsics", None)
    quality = dict(candidate.quality_metrics)
    reprojection_error = quality.get("reprojection_error_px")
    if reprojection_error is None:
        reprojection_error = float(getattr(previous_extrinsics, "reprojection_error_px", 0.0))
    operating_zoom = intrinsics_snapshot["applicability"].get("operating_zoom_index")
    translation_zoom = (
        operating_zoom
        if any(getattr(candidate.optimized_dofs, name) for name in ("north_m", "east_m", "down_m"))
        else getattr(previous, "source_zoom_level", None)
    )
    rotation_zoom = (
        operating_zoom
        if any(getattr(candidate.optimized_dofs, name) for name in ("roll_deg", "pitch_deg", "yaw_deg"))
        else getattr(previous, "source_zoom_level", None)
    )
    known_sources = {value for value in (rotation_zoom, translation_zoom) if value is not None}
    source_zoom = next(iter(known_sources)) if len(known_sources) == 1 else None
    extrinsics = StereoExtrinsics(
        rotation_left_to_right=[list(row) for row in stereo.rotation_right_from_left_rdf],
        translation_left_to_right_m=list(stereo.translation_right_from_left_rdf_m),
        reprojection_error_px=float(reprojection_error),
        pair_count=len(candidate.source_observation_ids),
    )
    profile = StereoExtrinsicsProfile(
        extrinsics=extrinsics,
        source_zoom_level=source_zoom,
        method=f"optical_pose/{candidate.method}",
        source_intrinsics_path=operating_profile.source_path,
        source_intrinsics_sha256=operating_profile.intrinsics_math_sha256,
        profile_id=selection.selection_id,
        metadata={
            "schema": "optical-pose-selected-extrinsics-v1",
            "candidate_id": candidate.candidate_id,
            "effective_from_pair_sequence": selection.effective_from_pair_sequence,
            "selected_dofs": list(selection.selected_dofs.selected_names),
            "rotation_observation_zoom_index": rotation_zoom,
            "translation_source_zoom_index": translation_zoom,
            "operating_intrinsics_profile_sha256": operating_profile.profile_sha256,
            "previous_profile_id": selection.previous_profile_id,
            "coordinate_contract": "OpenCV optical RDF; X_right=R_right_from_left*X_left+T metres",
            **(
                {
                    "yaw90_reference": json.loads(candidate.reference_evidence_json),
                    "left_pose": _pose_mapping(poses[0]),
                    "right_pose": _pose_mapping(poses[1]),
                }
                if candidate.reference_evidence_json
                else {}
            ),
        },
    )
    active_intrinsics = compose_stereo_calibration(current_intrinsics, profile)
    return _PreparedOpticalPoseApplication(
        selection=selection,
        poses=poses,
        profile=profile,
        active_intrinsics=active_intrinsics,
    )


def selection_mapping(
    selection: OpticalPoseSelection,
    poses: tuple[CameraPoseNed, CameraPoseNed],
) -> dict[str, Any]:
    stereo = stereo_pose_from_absolute(*poses)
    value = {
        "schema": "optical-pose-selection-v1",
        "state": "ACTIVE",
        "selection_id": selection.selection_id,
        "candidate_id": selection.candidate_id,
        "selected_dof": {
            name.removesuffix("_deg").removesuffix("_m"): bool(getattr(selection.selected_dofs, name))
            for name in POSE_DOF_NAMES
        },
        "effective_from_pair_sequence": selection.effective_from_pair_sequence,
        "applied_utc_ns": selection.applied_utc_ns,
        "previous_profile_id": selection.previous_profile_id,
        "left_pose": _pose_mapping(poses[0]),
        "right_pose": _pose_mapping(poses[1]),
        "relative_boresight_yaw_right_minus_left_deg": boresight_relative_yaw_right_minus_left_deg(*poses),
        "derived_stereo": {
            "R_right_from_left_rdf": [list(row) for row in stereo.rotation_right_from_left_rdf],
            "T_right_from_left_rdf_m": list(stereo.translation_right_from_left_rdf_m),
            "contract": "X_right_RDF=R_right<-left*X_left_RDF+T_right<-left metres",
        },
        "quality_snapshot": dict(selection.quality_snapshot),
    }
    value["profile_sha256"] = hashlib.sha256(canonical_json_bytes(value)).hexdigest()
    return value


def profile_geometry_values(prepared: _PreparedOpticalPoseApplication) -> dict[str, object]:
    """The coherent field set published by one prepared optical-pose state."""
    profile = prepared.profile
    return {
        "selected_extrinsics_profile": profile,
        "active_extrinsics_profile": profile,
        "optical_pose_extrinsics_profile": profile,
        "_selected_extrinsics_profile_path": f"memory:{profile.profile_id}",
        "_selected_extrinsics_profile_hash": profile.content_sha256,
        "_active_extrinsics_profile_path": f"memory:{profile.profile_id}",
        "_active_extrinsics_profile_hash": profile.content_sha256,
        "_active_optical_pose_profile_hash": profile.content_sha256,
        "_active_optical_pose_selection_id": prepared.selection.selection_id,
        "_active_optical_pose_effective_from_pair_sequence": prepared.selection.effective_from_pair_sequence,
        "_active_extrinsics_source_zoom_level": profile.source_zoom_level,
        "_active_calibration_selection_mode": "manual_independent_zoom"
        if profile.source_zoom_level is None
        else "manual_same_zoom",
        "active_intrinsics": prepared.active_intrinsics,
    }


@dataclass(frozen=True)
class OpticalApplicationInputs:
    intrinsics_snapshot: Mapping[str, Any]
    current_intrinsics: object
    active_intrinsics_path: str
    active_intrinsics_hash: str
    previous_profile: object
    next_pair_sequence: int


@dataclass(frozen=True)
class OpticalApplicationEffects:
    """Bounded effects at the active geometry/persistence boundary.

    capture/restore include the exact old geometry fields, provenance and cache.
    revision names the shared geometry publication; no unrelated UI is exposed.
    """

    capture: Callable
    commit_geometry: Callable
    restore: Callable
    invalidate: Callable
    persist: Callable
    revision: Callable
    preflight: Callable = lambda: None


class OpticalProfileApplication:
    def __init__(self, registry):
        self.registry = registry
        self._profile_restore_stack = []
        self._busy = False

    def reset(self):
        if self._busy:
            raise RuntimeError("Optical pose application is already in progress")
        self._profile_restore_stack.clear()

    def apply(self, payload, inputs, effects):
        if self._busy:
            raise RuntimeError("Optical pose application is already in progress")
        self._busy = True
        try:
            return self._apply(payload, inputs, effects)
        finally:
            self._busy = False

    def _apply(self, payload, inputs, effects):
        candidate_id = str(payload.get("candidate_id") or "")
        candidate = self.registry._candidates.get(candidate_id)
        if candidate is None:
            raise ValueError(f"unknown optical pose candidate: {candidate_id}")
        mask = candidate.optimized_dofs
        if not mask.selected_names:
            raise ValueError("candidate did not optimize any pose DOF")
        side_observable = {side: dict(values) for side, values in candidate.observability_by_side}
        unobservable = tuple(
            name
            for name in mask.selected_names
            if not all(side_observable[side].get(name, False) for side in ("left", "right"))
        )
        if unobservable:
            raise ValueError(
                "candidate optimized DOF are not observable on both camera sides: " + ", ".join(unobservable)
            )
        # Legacy callers may still send the former UI mask. Accept it only
        # when it exactly matches the owner-derived candidate contract; a
        # stale/partial mask must fail closed instead of silently dropping t.
        if isinstance(payload.get("dof"), Mapping):
            requested_mask = pose_mask(payload)
            if requested_mask.selected_names != mask.selected_names:
                raise ValueError(
                    "candidate DOF is owner-derived; payload override does not match: "
                    f"expected={','.join(mask.selected_names)} "
                    f"requested={','.join(requested_mask.selected_names) or 'none'}"
                )
        applicability = inputs.intrinsics_snapshot
        if not bool(applicability["applicability"]["geometry_allowed"]):
            raise RuntimeError(
                "exact operating K,D is not applicable: " + ", ".join(applicability["applicability"]["reasons"])
            )
        if candidate.method not in _KD_INDEPENDENT_CANDIDATE_METHODS:
            expected_profile_hash = dict(candidate.input_hashes).get(
                "operating_intrinsics_profile_sha256",
                "",
            )
            active_profile_hash = str(applicability["applicability"]["profile_sha256"])
            if not expected_profile_hash:
                raise RuntimeError("candidate operating K,D profile SHA-256 is missing; capture/solve again")
            if expected_profile_hash != active_profile_hash:
                raise RuntimeError("operating K,D changed after candidate creation; capture/solve again")
        current_intrinsics, operating_profile = active_intrinsics_for_apply(
            applicability,
            current=inputs.current_intrinsics,
            active_path=inputs.active_intrinsics_path,
            active_hash=inputs.active_intrinsics_hash,
        )
        effects.preflight()
        # Resolve every non-I/O persistence precondition before any active or
        # active-like owner field is changed. Actual write failures remain
        # rollback-protected below.
        prior_left, prior_right = self.registry._candidate_priors[candidate_id]
        left_parameters, right_parameters = apply_candidate_with_constraints(
            candidate,
            mask,
            prior_left,
            prior_right,
        )
        active = (left_parameters.to_camera_pose_ned(), right_parameters.to_camera_pose_ned())
        next_pair_sequence = inputs.next_pair_sequence
        previous_profile = inputs.previous_profile
        previous = str(getattr(previous_profile, "profile_id", "") or "")
        selection = OpticalPoseSelection(
            selection_id=f"pose-selection-{uuid.uuid4().hex}",
            candidate_id=candidate_id,
            selected_dofs=mask,
            effective_from_pair_sequence=next_pair_sequence,
            applied_utc_ns=time.time_ns(),
            previous_profile_id=previous,
            quality_snapshot=candidate.quality_metrics,
        )
        prepared = prepare_selected_profile(
            selection,
            candidate,
            active,
            applicability,
            current_intrinsics=current_intrinsics,
            operating_profile=operating_profile,
            previous=inputs.previous_profile,
        )
        previous_state = effects.capture()
        # The exact restore object is also its transaction identity. Never pop
        # a possibly newer restore point from the end during an old failure.
        self._profile_restore_stack.append(previous_state)
        committed_revision = effects.revision()
        try:
            effects.commit_geometry(profile_geometry_values(prepared))
            self.registry.select(prepared.selection, prepared.poses)
            committed_revision = effects.revision()
            effects.invalidate()
            result = selection_mapping(selection, active)
            result["selected_extrinsics_profile_id"] = prepared.profile.profile_id
            result["selected_extrinsics_content_sha256"] = prepared.profile.content_sha256
            result["effective_boundary"] = "next accepted pair (zero-based sequence equals current row count)"
            alignment_path = effects.persist()
        except Exception:
            self._profile_restore_stack[:] = [
                state for state in self._profile_restore_stack if state is not previous_state
            ]
            # A persistence hook may re-enter another geometry owner. A past
            # rollback cannot overwrite that later publication or its cache.
            if effects.revision() == committed_revision:
                effects.restore(previous_state)
            raise
        result["session_alignment_path"] = str(alignment_path)
        return result

    def revert(self, effects):
        if self._busy:
            raise RuntimeError("Optical pose application is already in progress")
        self._busy = True
        try:
            previous = self.registry._active_selection
            before = effects.capture()
            state = self._profile_restore_stack[-1] if self._profile_restore_stack else None
            if state is not None:
                effects.restore(state)
            else:
                self.registry._active_selection = None
                self.registry._active_poses = None
            committed_revision = effects.revision()
            try:
                effects.invalidate()
                result = {
                    "schema": "optical-pose-selection-v1",
                    "state": "REVERTED",
                    "previous_selection_id": "" if previous is None else previous.selection_id,
                    "reverted_utc_ns": time.time_ns(),
                }
                effects.persist()
            except Exception:
                if effects.revision() == committed_revision:
                    effects.restore(before)
                raise
            if state is not None:
                self._profile_restore_stack[:] = [value for value in self._profile_restore_stack if value is not state]
            return result
        finally:
            self._busy = False
