"""Immutable calibration artifacts and application plans; no widgets or live panel."""
from __future__ import annotations
import hashlib
import json
import os
import uuid
import time
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping
from .quality import CalibrationQuality
from Workstation.workstation_core.calibration import (
    StereoIntrinsicsResult, StereoExtrinsicsProfile, save_intrinsics_json,
    save_stereo_extrinsics_json, compose_stereo_calibration,
    intrinsics_content_sha256, independent_zoom_selection_mode,
    load_intrinsics_json, load_stereo_extrinsics_json, without_stereo_extrinsics,
)


@dataclass(frozen=True)
class AppliedCalibration:
    candidate_path: str
    selected_profile: object
    selected_path: str
    selected_hash: str
    checkerboard_candidate: object
    optical_candidate: object
    geometry_values: dict


@dataclass(frozen=True)
class SavedIntrinsicsAttempt:
    output: Path
    approved: bool
    result: StereoIntrinsicsResult
    metadata: Mapping[str, object]


@dataclass(frozen=True)
class SavedExtrinsicsAttempt:
    output: Path
    approved: bool


class CalibrationProfiles:
    def __init__(self, publish_geometry=None):
        self.publish_geometry = publish_geometry

    @staticmethod
    def read_source_metadata(path: Path) -> dict[str, object]:
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError, TypeError):
            return {}
        metadata = payload.get("metadata") if isinstance(payload, dict) else None
        return dict(metadata) if isinstance(metadata, dict) else {}

    def save_intrinsics_attempt(
        self, result, *, root: Path, zoom_level, quality_state: str,
        quality_reasons, metadata: Mapping[str, object],
        approved_output: Path | None, intrinsics_only: bool = False,
    ) -> SavedIntrinsicsAttempt:
        """Persist primary evidence before aliases or immutable publication.

        Callers may report this returned attempt as saved even if later alias
        publication fails. An acknowledged WARNING never becomes an approved
        saved profile; acknowledgement belongs to Apply's separate contract.
        """
        approved = CalibrationQuality.approved_for_profile_save(
            quality_state, capture_zoom_known=zoom_level is not None,
        )
        root = Path(root)
        root.mkdir(parents=True, exist_ok=True)
        if approved:
            if approved_output is None:
                raise ValueError("Approved intrinsics require a sampled output path")
            output = Path(approved_output)
        else:
            attempts = root / "attempts"
            attempts.mkdir(parents=True, exist_ok=True)
            zoom_token = "unknown" if zoom_level is None else f"{int(zoom_level):02d}"
            state_token = str(quality_state or "unassessed").lower()
            output = attempts / f"intrinsics_zoom_{zoom_token}_{state_token}_{time.time_ns()}.json"
        stored_metadata = deepcopy(dict(metadata))
        stored_metadata.update(
            profile_status="approved" if approved else "attempt_only",
            eligible_for_application=bool(approved), quality_state=quality_state,
            quality_reasons=list(quality_reasons),
        )
        stored_result = without_stereo_extrinsics(result) if intrinsics_only else result
        save_intrinsics_json(output, stored_result, metadata=stored_metadata)
        return SavedIntrinsicsAttempt(output, approved, stored_result, stored_metadata)

    def publish_saved_intrinsics(self, saved: SavedIntrinsicsAttempt, *, root: Path, latest_output: Path) -> Path:
        """Publish approved aliases after the primary attempt has been recorded."""
        if not saved.approved:
            raise ValueError("An attempt-only intrinsic result cannot publish approved aliases")
        latest_output = Path(latest_output)
        if saved.output != latest_output:
            save_intrinsics_json(latest_output, saved.result, metadata=dict(saved.metadata))
        return self._publish_immutable_intrinsics_profile(saved.result, metadata=saved.metadata, root=Path(root))

    def save_extrinsics_attempt(self, profile, *, output: Path, quality_state: str) -> SavedExtrinsicsAttempt:
        approved = CalibrationQuality.approved_for_profile_save(quality_state)
        output = Path(output)
        if not approved:
            attempts = output.parent / "attempts"
            attempts.mkdir(parents=True, exist_ok=True)
            output = attempts / f"{output.stem}_{quality_state.lower()}_{time.time_ns()}{output.suffix}"
        output.parent.mkdir(parents=True, exist_ok=True)
        save_stereo_extrinsics_json(output, profile)
        return SavedExtrinsicsAttempt(output, approved)

    @staticmethod
    def archive_loaded_intrinsics(source: Path, *, root: Path) -> Path:
        """Archive exact operator-selected bytes in the already chosen Session."""
        destination_dir = Path(root) / "loaded_intrinsics"
        destination_dir.mkdir(parents=True, exist_ok=True)
        destination = destination_dir / f"{time.time_ns()}-{source.name}"
        payload = source.read_bytes()
        temporary = destination.with_name(f".{destination.name}.{uuid.uuid4().hex}.tmp")
        try:
            with temporary.open("xb") as handle:
                handle.write(payload)
                handle.flush()
            temporary.replace(destination)
        finally:
            temporary.unlink(missing_ok=True)
        return destination

    def _publish_immutable_calibration_file(
        self,
        source: Path,
        *,
        root: Path,
        family: str,
        prefix: str,
    ) -> Path:
        """Publish exact profile bytes once under a content-addressed name."""

        payload = Path(source).read_bytes()
        digest = hashlib.sha256(payload).hexdigest()
        destination_dir = root / "profiles" / family
        destination_dir.mkdir(parents=True, exist_ok=True)
        destination = destination_dir / f"{prefix}-{digest}.json"
        if destination.exists():
            if destination.read_bytes() != payload:
                raise RuntimeError(
                    f"immutable calibration hash collision at {destination}"
                )
            return destination
        temporary = destination.with_name(f".{destination.name}.{uuid.uuid4().hex}.tmp")
        try:
            with temporary.open("xb") as handle:
                handle.write(payload)
                handle.flush()
                os.fsync(handle.fileno())
            try:
                os.link(temporary, destination)
            except FileExistsError:
                if destination.read_bytes() != payload:
                    raise RuntimeError(
                        f"immutable calibration destination changed at {destination}"
                    )
            descriptor = os.open(destination_dir, os.O_RDONLY)
            try:
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
        finally:
            temporary.unlink(missing_ok=True)
        return destination


    def _publish_immutable_intrinsics_profile(
        self,
        result: StereoIntrinsicsResult,
        *,
        metadata: Mapping[str, object],
        root: Path,
    ) -> Path:
        staging = root / "profiles" / ".staging"
        staging.mkdir(parents=True, exist_ok=True)
        temporary = staging / f"intrinsics-{uuid.uuid4().hex}.json"
        try:
            save_intrinsics_json(temporary, result, metadata=dict(metadata))
            return self._publish_immutable_calibration_file(
                temporary,
                root=root,
                family="intrinsics",
                prefix="intrinsics",
            )
        finally:
            temporary.unlink(missing_ok=True)


    def _publish_immutable_extrinsics_profile(
        self,
        profile: StereoExtrinsicsProfile,
        *, root: Path,
    ) -> Path:
        staging = root / "profiles" / ".staging"
        staging.mkdir(parents=True, exist_ok=True)
        temporary = staging / f"extrinsics-{uuid.uuid4().hex}.json"
        try:
            save_stereo_extrinsics_json(temporary, profile)
            return self._publish_immutable_calibration_file(
                temporary,
                root=root,
                family="extrinsics",
                prefix="extrinsics",
            )
        finally:
            temporary.unlink(missing_ok=True)



    load_intrinsics = staticmethod(load_intrinsics_json)
    load_extrinsics = staticmethod(load_stereo_extrinsics_json)

    def apply_bundle(self, *, root, result, zoom_level, candidate_path,
                     active_intrinsics, active_path, selected_profile,
                     selected_path, selected_hash, use_selected_extrinsics,
                     extrinsics_profile_override, quality_state, quality_reasons,
                     quality_warning_confirmed):
        if not callable(self.publish_geometry):
            raise RuntimeError("Profile application requires the active geometry owner")
        decision = CalibrationQuality.decide("Apply", quality_state, quality_reasons, warning_confirmed=quality_warning_confirmed)
        if not decision.allowed:
            raise ValueError(decision.message)
        checkerboard_candidate = optical_candidate = None
        candidate_path_text = str(candidate_path or "")
        # Lightweight legacy facades may not expose the candidate slot yet.
        # Reuse their pre-Apply marker only when no active matrix exists, so a
        # new candidate can never borrow an older active profile identity.
        if not candidate_path_text and active_intrinsics is None:
            candidate_path_text = str(active_path or "")
        current_intrinsics_path = Path(candidate_path_text)
        if (
            current_intrinsics_path.is_file()
            and current_intrinsics_path.parent.name == "intrinsics"
            and current_intrinsics_path.parent.parent.name == "profiles"
        ):
            immutable_intrinsics = current_intrinsics_path
        else:
            immutable_intrinsics = self._publish_immutable_intrinsics_profile(
                result,
                root=root,
                metadata={
                    "profile_status": "runtime_applied",
                    "eligible_for_application": True,
                    "zoom_level": zoom_level,
                    "capture_zoom_level": zoom_level,
                    "quality_state": quality_state,
                    "quality_reasons": list(quality_reasons),
                    "quality_warning_confirmed": bool(
                        quality_state == "WARNING" and quality_warning_confirmed
                    ),
                },
            )
        candidate_path = str(immutable_intrinsics)
        selected_profile = (
            selected_profile
            if use_selected_extrinsics
            else extrinsics_profile_override
        )
        if selected_profile is not None and not isinstance(
            selected_profile, StereoExtrinsicsProfile
        ):
            raise TypeError(
                "extrinsics_profile_override must be StereoExtrinsicsProfile or None"
            )
        if (
            selected_profile is None
            and use_selected_extrinsics
            and result.extrinsics is not None
        ):
            selected_profile = StereoExtrinsicsProfile(
                extrinsics=result.extrinsics,
                source_zoom_level=zoom_level,
                method="legacy_embedded_checkerboard",
                source_intrinsics_path=candidate_path,
                source_intrinsics_sha256=intrinsics_content_sha256(
                    result
                ),
                profile_id="legacy_embedded_extrinsics",
            )
            selected_path = (
                f"{candidate_path}#embedded-extrinsics"
            )
            selected_hash = selected_profile.content_sha256
        if selected_profile is not None:
            # Always derive the immutable path from the profile being applied.
            # Reusing the currently selected path can pair a fresh profile hash
            # with stale R,t bytes from an earlier candidate.
            immutable_extrinsics = self._publish_immutable_extrinsics_profile(
                selected_profile, root=root
            )
            if str(selected_profile.method or "").startswith("optical_pose/"):
                optical_candidate = selected_profile
            else:
                checkerboard_candidate = selected_profile
            selected_path = str(immutable_extrinsics)
            selected_hash = selected_profile.content_sha256
        geometry_values = {}
        geometry_values['active_intrinsics'] = compose_stereo_calibration(result, selected_profile)
        geometry_values['active_extrinsics_profile'] = selected_profile
        if selected_profile is None:
            geometry_values['checkerboard_extrinsics_profile'] = None
            geometry_values['optical_pose_extrinsics_profile'] = None
        elif str(selected_profile.method or '').startswith('optical_pose/'):
            geometry_values['optical_pose_extrinsics_profile'] = selected_profile
        else:
            geometry_values['checkerboard_extrinsics_profile'] = selected_profile
        geometry_values['_active_intrinsics_profile_path'] = str(immutable_intrinsics)
        geometry_values['_active_intrinsics_source_zoom_level'] = zoom_level
        geometry_values['_active_intrinsics_profile_hash'] = intrinsics_content_sha256(result)
        if selected_profile is None:
            geometry_values['_active_extrinsics_profile_path'] = ''
            geometry_values['_active_extrinsics_profile_hash'] = ''
            geometry_values['_active_extrinsics_source_zoom_level'] = None
            geometry_values['_active_calibration_selection_mode'] = 'intrinsics_only'
        else:
            geometry_values['_active_extrinsics_profile_path'] = selected_path or f"memory:{selected_profile.profile_id or 'stereo_extrinsics'}"
            geometry_values['_active_extrinsics_profile_hash'] = selected_profile.content_sha256
            geometry_values['_active_extrinsics_source_zoom_level'] = selected_profile.source_zoom_level
            geometry_values['_active_calibration_selection_mode'] = independent_zoom_selection_mode(zoom_level, selected_profile.source_zoom_level)
        if selected_profile is not None:
            geometry_values["selected_extrinsics_profile"] = selected_profile
            geometry_values["_selected_extrinsics_profile_path"] = selected_path
            geometry_values["_selected_extrinsics_profile_hash"] = selected_hash
        self.publish_geometry(geometry_values)
        return AppliedCalibration(
            candidate_path, selected_profile, selected_path, selected_hash,
            checkerboard_candidate, optical_candidate, geometry_values,
        )
