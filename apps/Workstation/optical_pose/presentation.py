"""Detached optical-pose UI/evidence projection, with no live panel lookup."""

from __future__ import annotations
import copy
from typing import Any, Mapping
from Workstation.workstation_core.native_frame_store import NativeFrameArtifact
from Workstation.workstation_core.optical_pose import POSE_DOF_NAMES
from .candidates import candidate_mapping_with_prior


def measurement_evidence_snapshot(registry, review):
    """Detached candidate/selection values for Measurement's read boundary.

    Candidate/prior/selection dataclasses are immutable; nested proposal and
    rotation context mappings are copied. No authoritative dictionary escapes.
    """
    return {
        "active_selection": registry._active_selection,
        "candidates": dict(registry._candidates),
        "candidate_priors": dict(registry._candidate_priors),
        "candidate_drone_rotation_context": copy.deepcopy(registry._candidate_drone_rotation_context),
        "last_accepted_auto_suggestion": copy.deepcopy(review._last_accepted_auto_suggestion),
    }


def zoom(values: object, side: str) -> str:
    row = values.get(side, {}) if isinstance(values, Mapping) else {}
    index = row.get("index") if isinstance(row, Mapping) else None
    return "unknown" if index is None else str(index)


def optical_pose_snapshot(
    capture,
    registry,
    review,
    *,
    intrinsics,
    actual_zoom,
    selected_extrinsics,
    include_frames: bool = True,
) -> dict[str, Any]:
    active_profile_id = "" if registry._active_selection is None else registry._active_selection.selection_id
    active_candidate_id = "" if registry._active_selection is None else registry._active_selection.candidate_id
    geometry_allowed = bool(intrinsics["applicability"]["geometry_allowed"])
    current_profile_hash = str(intrinsics["applicability"]["profile_sha256"])
    candidate_rows: list[dict[str, Any]] = []
    for candidate in registry._candidates.values():
        row = candidate_mapping_with_prior(
            candidate,
            registry._candidate_priors[candidate.candidate_id],
        )
        expected = dict(candidate.input_hashes).get(
            "operating_intrinsics_profile_sha256",
            "",
        )
        row["applicable"] = geometry_allowed and (not expected or expected == current_profile_hash)
        if not row["applicable"]:
            row["state"] = "BLOCKED"
        candidate_rows.append(row)
    frozen_previews: dict[str, str] = {}
    frozen_native_frames: dict[str, dict[str, Any]] = {}
    frozen_frame_artifacts: dict[str, dict[str, Any]] = {}
    for side, row in capture._frozen_frames.items():
        artifact: NativeFrameArtifact = row["artifact"]
        frozen_frame_artifacts[side] = artifact.to_mapping()
        frozen_previews[side] = (
            f"{artifact.frame_id} · {artifact.width}×{artifact.height} · sha256:{artifact.sha256[:12]}"
        )
        if include_frames:
            frozen_native_frames[side] = {
                "image": row["image"].copy(),
                "frame_id": artifact.frame_id,
                "image_sha256": artifact.sha256,
                "metadata": copy.deepcopy(row["metadata"]),
                "relative_path": artifact.relative_path,
            }
    terrain_state = "NOT SET"
    if capture.artifacts.terrain_snapshot is not None:
        terrain_state = (
            "READY"
            if capture.artifacts.terrain_metadata and capture.artifacts.terrain_metadata.full_pnp_height_ready
            else "BLOCKED · vertical datum unknown"
        )
    mountain_uv_state = "REQUIRES paired frozen left/right u,v clicks"
    if any(str(row.get("method") or "") == "gnss-constrained-mountain-uv" for row in candidate_rows):
        mountain_uv_state = "USED · paired frozen left/right u,v clicks"
    active_candidate = next(
        (row for row in candidate_rows if str(row.get("candidate_id") or "") == active_candidate_id),
        None,
    )
    active_selected_dof = (
        {
            name.removesuffix("_deg").removesuffix("_m"): bool(getattr(registry._active_selection.selected_dofs, name))
            for name in POSE_DOF_NAMES
        }
        if registry._active_selection is not None
        else {}
    )
    return {
        "state": "ACTIVE" if active_profile_id else "CANDIDATE" if registry._candidates else "NO DATA",
        "active_profile_id": active_profile_id,
        "active_candidate_id": active_candidate_id,
        "active_selection_id": active_profile_id,
        "active_selected_dof": active_selected_dof,
        "active_candidate": active_candidate,
        "mountain_ridge_margin_fraction": review._mountain_ridge_margin_fraction,
        "mountain_max_points": review._mountain_max_points,
        "readiness": {
            "state": "ACTIVE" if active_profile_id else "CANDIDATE" if registry._candidates else "NO DATA",
            "operating_zoom": f"{zoom(actual_zoom, 'left')} / {zoom(actual_zoom, 'right')}",
            "intrinsics": (
                "READY · exact operating K,D"
                if geometry_allowed
                else "BLOCKED · " + ", ".join(intrinsics["applicability"]["reasons"])
            ),
            "base_extrinsics": "LOADED" if selected_extrinsics is not None else "NOT SELECTED",
            "yaw90_reference": (
                "GNSS Yaw90 · 촬영 시 고정 · "
                + str(capture._frozen_yaw90_reference.get("state"))
                + " · "
                + "; ".join(capture._frozen_yaw90_reference.get("blocking_reasons", ()))
                if capture._frozen_yaw90_reference
                else "촬영 요청 시 P1/GNSS 기준 필요"
            ),
            "terrain": terrain_state,
            "mountain_uv": mountain_uv_state,
            "target_reference": (
                "GNSS + Mountain: paired frozen left/right u,v + numeric GNSS baseline; "
                "Drone yaw: explicit target RTK + pixel/ray"
            ),
        },
        "intrinsics_applicability": intrinsics,
        "terrain_asset": copy.deepcopy(capture.artifacts.terrain_snapshot or {}),
        "frozen_previews": frozen_previews,
        # UI projection key.  QImages never enter the append-only event log.
        "frozen_frames": frozen_native_frames if include_frames else {},
        "frozen_native_frames": frozen_native_frames if include_frames else {},
        # Persistent identity/path projection; unlike the UI frame keys,
        # this is safe to store in session_static.json.
        "frozen_frame_artifacts": frozen_frame_artifacts,
        "yaw90_reference": copy.deepcopy(capture._frozen_yaw90_reference),
        "candidates": candidate_rows,
        "selection_history": [selection.selection_id for selection in registry._selection_history],
        # Automatic proposals are audit/UI evidence only.  They contain
        # native points and frame identities, never QImages or live-frame
        # references, and are not copied into ``annotations`` here.
        "auto_suggestions": [copy.deepcopy(value) for value in review._auto_suggestions.values()],
        "accepted_auto_suggestion": (
            None
            if review._last_accepted_auto_suggestion is None
            else copy.deepcopy(review._last_accepted_auto_suggestion)
        ),
        "annotation_review": (None if review._annotation_review is None else copy.deepcopy(review._annotation_review)),
        # Append-only Drone RTK capture-observation evidence (deep-copied
        # so a caller mutating the returned list/dicts cannot corrupt the
        # stored history). Solve/apply/candidate state above is entirely
        # unaffected by this key.
        "drone_rtk_capture_observations": [
            copy.deepcopy(observation) for observation in capture.observations._drone_rtk_observations
        ],
        # Latest operator target-pair revision per frozen burst. The
        # capture observation remains immutable; edits replace only this
        # derived binding while run_events preserves every revision.
        "drone_rtk_multiview_samples": [
            copy.deepcopy(sample)
            for sample in sorted(
                capture.observations._drone_rtk_multiview_samples.values(),
                key=lambda value: int(value.get("sequence", 0)),
            )
        ],
    }
